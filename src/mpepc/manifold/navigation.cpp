/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     navigation.cpp
 * \author   Collin Johnson
 *
 * Definition of NavigationTaskManifold.
 */

#include "mpepc/manifold/navigation.h"
#include "hssh/local_topological/local_topo_graph.h"
#include "hssh/local_topological/local_topo_map.h"
#include "mpepc/control/control_law_coordinates.h"
#include "mpepc/cost/obstacle_cost.h"
#include "mpepc/cost/quasi_static_cost.h"
#include "mpepc/grid/obstacle_distance_grid.h"
#include "mpepc/simulator/dynamic_object_trajectory.h"
#include "mpepc/trajectory/robot_trajectory_info.h"
#include "system/module_communicator.h"
#include "utils/ray_tracing.h"
#include "utils/timestamp.h"

namespace vulcan
{
namespace mpepc
{

NavigationTaskManifold::NavigationTaskManifold(const pose_t& target,
                                               bool isPoseTarget,
                                               const navigation_task_params_t& taskParams,
                                               const navigation_task_manifold_params_t& builderParams)
: TaskManifold()
, navGridBuilder_(builderParams.navGridBuilderParams)
, target_(target)
, isPoseTarget_(isPoseTarget)
, routeTarget_(target)
, haveNewNavGrid_(false)
, robotTrajectoryTimestamp_(-1)
, initialHeadingError_(0.0)
, debugDataUpdatePeriodUs_(1000000)
, lastDebugDataUpdateTimeUs_(0)
, taskParams_(taskParams)
, builderParams_(builderParams)
{
}


NavigationTaskManifold::NavigationTaskManifold(const NavigationGrid& navGrid,
                                               const navigation_task_params_t& taskParams,
                                               const navigation_task_manifold_params_t& builderParams)
: TaskManifold()
, navGrid_(navGrid)
, navGridBuilder_(builderParams.navGridBuilderParams)
, target_(navGrid.getGoalPose())
, routeTarget_(target_)
, haveNewNavGrid_(false)
, robotTrajectoryTimestamp_(-1)
, initialHeadingError_(0.0)
, debugDataUpdatePeriodUs_(1000000)
, lastDebugDataUpdateTimeUs_(0)
, taskParams_(taskParams)
, builderParams_(builderParams)
{
}


void NavigationTaskManifold::update(const planning_environment_t& env,
                                    const std::vector<dynamic_object_trajectory_t>& trajectories)
{
    assert(env.dists);

    updateTopoRoute(env);
    updateNavigationGrid(env, trajectories);
    updateStateMachine(*env.dists, env.robotState);
}


std::vector<dynamic_object_trajectory_t>
  NavigationTaskManifold::createTaskSpecificObjects(const planning_environment_t& env)
{
    assert(env.visibility);

    std::vector<dynamic_object_trajectory_t> objects;


    return objects;
}


void NavigationTaskManifold::updateTopoRoute(const planning_environment_t& env)
{
    routeGateways_.clear();
    if (env.ltm) {
        // Find the topological route from the start to the goal
        auto start = env.ltm->areaContaining(env.robotState.pose.toPoint());
        auto end = env.ltm->areaContaining(target_.toPoint());

        if (start && end) {
            auto startNode = std::make_pair(env.robotState.pose.toPoint(), start->id());
            auto goalNode = std::make_pair(target_.toPoint(), end->id());

            hssh::LocalTopoGraph graph(*env.ltm);
            auto route = graph.findPath(startNode, goalNode);

            std::cout << "Found route that visits " << route.size() << " areas.\n";

            // Go through the route and only lookahead up to 20m, since that's as far ahead as sensors are useful
            // and it saves unnecessary computation for getting to the goal
            if (route.size() > 0) {
                topoRoute_ = hssh::LocalTopoRoute();   // clean out the old route and add bits as we go along

                const double kMaxRouteDist = 20.0;
                double distAlongRoute = 0.0;
                for (auto& visit : route) {
                    topoRoute_.addVisit(visit);
                    distAlongRoute += visit.distance();

                    if (distAlongRoute >= kMaxRouteDist) {
                        break;
                    }
                }

                // Adjust the intermediate planning target if we don't reach the target b/c the final visit has
                // an exit gateway
                if (auto exitGwy = topoRoute_.back().exitGateway()) {
                    routeTarget_ = pose_t(exitGwy->center(), exitGwy->direction());
                }
                // Otherwise, just going to the overall target for the task
                else {
                    routeTarget_ = target_;
                }

            } else {
                // just move over the route to clear it out when empty
                topoRoute_ = std::move(route);
                routeTarget_ = target_;
            }

            // For each visited area, if it was entered via gateway, then that gateway should be added to the
            // intermediate nodes that must be visited for the area
            for (auto& area : topoRoute_) {
                auto entryGateway = area.entryGateway();
                if (entryGateway) {
                    routeGateways_.push_back(entryGateway->boundary());
                    std::cout << "Traveling to goal via: " << entryGateway->boundary() << '\n';
                }
            }
        }
    }
}


void NavigationTaskManifold::updateNavigationGrid(const planning_environment_t& env,
                                                  const std::vector<dynamic_object_trajectory_t>& trajectories)
{
    static int64_t totalNavGridTime_ = 0;
    static int numNavGrids = 0;

    // build navigation grid.
    // always do this - it usually takes a few milliseconds unless map is very, very large.
    int64_t ticUs = utils::system_time_us();

    costMap_.setTimestamp(env.dists->getTimestamp());
    costMap_.setId(env.dists->getId());
    costMap_.setGridSizeInCells(env.dists->getWidthInCells(), env.dists->getHeightInCells());
    costMap_.setBottomLeft(env.dists->getBottomLeft());
    costMap_.setMetersPerCell(env.dists->metersPerCell());
    costMap_.reset(builderParams_.useLearnedNorm && env.ltm ? kNonRouteCost : 0);
    costMap_.resetMaxCost();

    obstacle_distance_cost_params_t distCostParams = builderParams_.obstacleCostParams;
    distCostParams.obstacleCostRadius = distCostParams.robotRadius;
    distCostParams.alphaCost = 1.0f;   // costs three times as much to move right next to the wall
    distCostParams.betaExp = 2.0f;

    obstacle_distance_cost(distCostParams, env, costMap_);

    if (builderParams_.navGridBuilderParams.shouldAddQuasiStaticObjects) {
        createQuasiStaticPoints(trajectories);
        quasi_static_cost(quasiStaticPoints_, builderParams_.quasiStaticParams, env, costMap_);
        costMap_.setId(costMap_.getId() + 1);   // quasi-static will ALWAYS change the cost map
    }

    int64_t socialStart = utils::system_time_us();

    if (builderParams_.useLearnedNorm) {
        learnedNormDebug.pathSituations.clear();
        learnedNormDebug.placeSituations.clear();
        learnedNormDebug.agents.clear();

        learned_norm_cost(topoRoute_, builderParams_.learnedNormParams, env, trajectories, costMap_, learnedNormDebug);
    }

    std::cout << "Social build time: " << ((utils::system_time_us() - socialStart) / 1000) << "ms\n";

    haveNewNavGrid_ = navGridBuilder_.buildGrid(env.robotState.pose, routeTarget_, routeGateways_, costMap_, navGrid_);

    int64_t tocUs = utils::system_time_us();

    // timing info
    if (haveNewNavGrid_) {
        ++numNavGrids;
        totalNavGridTime_ += tocUs - ticUs;
        std::cout << "INFO: NavigationTaskManifold: Navigation Grid Updated (new map or new task): \n"
                  << "  Map ID: " << navGrid_.getId() << "  Processing time (ms): " << (tocUs - ticUs) / 1000
                  << " Avg (ms):" << (totalNavGridTime_ / numNavGrids / 1000) << " Total:" << numNavGrids << '\n';
    }
}


void NavigationTaskManifold::updateStateMachine(const ObstacleDistanceGrid& map, const motion_state_t& state)
{
    // state-machine to determine which cost and gradient information to use based on the inital pose
    // this has to be detemined only once per planning cycle.
    if (state.timestamp != robotTrajectoryTimestamp_) {
        // update timestamp
        robotTrajectoryTimestamp_ = state.timestamp;

        // check radial distance
        pose_t initialPose = state.pose;
        double radialDistanceToGoal = distance_between_points(initialPose.toPoint(), target_.toPoint());

        // is it very close to the goal?
        isVeryCloseToGoal_ = radialDistanceToGoal < taskParams_.useGoalOrientationRadius;

        // check approaching condition
        // use approach-to-goal mode only (1) if the robot is not already very close to the goal and (2) not too far
        // away from the goal and (3) if there is a clear line-of-sight between the inital point and the goal
        isApproachingGoal_ = !isVeryCloseToGoal_ && (radialDistanceToGoal < taskParams_.goalNeighborhoodRadius);
        if (isApproachingGoal_) {
            Line<float> lineToGoal(utils::global_point_to_grid_point(initialPose.toPoint(), map),
                                   utils::global_point_to_grid_point(target_.toPoint(), map));
            std::vector<Point<int>> lineCells;
            utils::find_cells_along_line(lineToGoal, map, std::back_inserter(lineCells));

            for (auto cell : lineCells) {
                isApproachingGoal_ &=
                  map.getObstacleDistance(cell) > builderParams_.navGridBuilderParams.robotShortRadius;
                ;
            }
        }

        // for all other cases, use the navigation function
        isUsingNavGrid_ = !isVeryCloseToGoal_ && !isApproachingGoal_;

        // determine when to use the gradient of the navigation function.
        isUsingNavGradient_ = false;
        if (isUsingNavGrid_) {
            // use the gradient of the navigation function only if when the initial error is large. Avoid artifacts of
            // grid computation.
            double initialHeadingError_ = angle_diff_abs(computeNavGradient(initialPose), initialPose.theta);
            double initialSpeed = state.velocity.linear;
            float initialDistanceToWalls = map.getObstacleDistance(initialPose.toPoint());
            isUsingNavGradient_ = initialHeadingError_ > 0.5 && initialSpeed < 0.2
              && initialDistanceToWalls > builderParams_.navGridBuilderParams.robotLongRadius + 0.02;
        }

#ifdef DEBUG_NAVIGATION_TASK
        // status messages
        if (isVeryCloseToGoal_) {
            std::cout << "INFO: NavigationTaskManifold: Status: Is close to the goal.\n";
        }

        if (isApproachingGoal_) {
            std::cout << "INFO: NavigationTaskManifold: Status: Approaching the goal.\n";
        }

        if (isUsingNavGrid_) {
            std::cout << "INFO: NavigationTaskManifold: Status: Using the navigation function.\n";
            if (isUsingNavGradient_) {
                std::cout << "INFO: NavigationTaskManifold: Status: Using the gradient of the navigation function.\n";
            }
        }

        // additional info
        control_law_coordinates_t coords(initialPose, target_);
        std::cout << "INFO: Params: kPhi: " << taskParams_.kPhi << " , kDelta : " << taskParams_.kDelta << ".\n";
        std::cout << "INFO: Target Pose: " << target_ << ".\n";
        std::cout << "INFO: Radial Distance: " << coords.r << ".\n";

        std::cout << "INFO: Egocentric Polar Coords: (" << coords.r << ',' << coords.theta << ',' << coords.delta
                  << ")\n";
        std::cout << "INFO: NF Distance: " << navGrid_.getCostToGo(state.pose.toPoint()) << ".\n";
        std::cout << "INFO: Initial Cost-to-go: " << getCostToGo(state) << ".\n";
#endif
    }
}


void NavigationTaskManifold::createQuasiStaticPoints(const std::vector<dynamic_object_trajectory_t>& trajectories)
{
    quasiStaticIndices_.clear();
    if (!trajectories.empty()) {
        float quasiStaticLengthLeft = builderParams_.navGridBuilderParams.quasiStaticLength;
        float quasiStaticTiming = builderParams_.navGridBuilderParams.quasiStaticTiming;

        int quasiStaticStateIdx = 0;
        while (quasiStaticLengthLeft > -0.0001) {
            quasiStaticStateIdx = static_cast<int>(quasiStaticTiming / trajectories.begin()->timestep);

            if (quasiStaticStateIdx < 0)   // protect from over/undershoot
            {
                quasiStaticStateIdx = 0;
            } else if (static_cast<size_t>(quasiStaticStateIdx) >= trajectories.begin()->states.size()) {
                quasiStaticStateIdx = trajectories.begin()->states.size() - 1;
                quasiStaticTiming = trajectories.begin()->timestep * trajectories.begin()->states.size();
            }

            quasiStaticIndices_.push_back(quasiStaticStateIdx);

            // add points every half second
            quasiStaticTiming -= 0.5;
            quasiStaticLengthLeft -= 0.5;
        }
    }

    quasiStaticPoints_.clear();
    for (const auto& objTrj : trajectories) {
        for (std::size_t index : quasiStaticIndices_) {
            dynamic_object_state_t quasiStaticState = objTrj.states[index];
            quasiStaticPoints_.emplace_back(quasiStaticState.x, quasiStaticState.y);
        }
    }
}


void NavigationTaskManifold::calculateNegativeRewardsOverTrajectory(const std::vector<motion_state_t>& trajectory,
                                                                    float timestep,
                                                                    std::size_t stride,
                                                                    std::vector<float>& rewards) const
{
    rewards.clear();   // ensure all rewards are being pushed to the correct index
    stride = std::max(stride, std::size_t(1));

    // compute spatial reward as a measure of progress
    float timeBetweenSamples = stride * timestep;

    for (size_t index = stride; index < trajectory.size(); index += stride) {
        pose_t endPose = trajectory[index].pose;
        bool isEndPointInMap = navGrid_.isPointInGrid(endPose.toPoint(), 0.1);

        double piecewiseProgress = 0.0;
        if (isEndPointInMap) {
            piecewiseProgress =
              piecewiseNegativeReward(trajectory[index - stride], trajectory[index], timeBetweenSamples);
        }

        rewards.push_back(piecewiseProgress);
    }
}


double NavigationTaskManifold::piecewiseNegativeReward(const motion_state_t& startState,
                                                       const motion_state_t& endState,
                                                       float timestep) const
{
    // stuff to be computed
    double negativeProgress =
      0.0;   // this measures difference in the cost-to-go from the start pose to the end pose. negative if progressing.
    double negativeDwellTime = 0.0;   // amount of time the robot stays at the goal pose times -1
    double negativeReward = 0.0;      // total reward. the more negative the better.

    // compute cost-to-go and heading error
    double endCost = getCostToGo(endState);
    double startCost = getCostToGo(startState);

    // spatial progress
    negativeProgress = endCost - startCost;

    // compute temporal reward as a measure of stay at the goal
    // Note that by construction, the negative reward and the dwell time weights are NEVER active at the same time.
    if (taskParams_.shouldUseDwellTime) {
        bool isConverged =
          distance_between_points(endState.pose.toPoint(), target_.toPoint()) < 0.5 * taskParams_.convergenceRadius;

        if (isConverged && isPoseTarget_) {
            isConverged =
              isConverged && (angle_diff_abs(endState.pose.theta, target_.theta) < 0.5 * taskParams_.convergenceAngle);
        }

        if (isConverged) {
            negativeDwellTime = -timestep;
        }
    }

    // final reward is the sum of all gains.
    negativeReward = negativeProgress + taskParams_.dwellTimeWeight * negativeDwellTime;

    return negativeReward;
}


double NavigationTaskManifold::getCostToGo(const motion_state_t& state) const
{
    double costToGo = 0.0;
    double headingError = 0.0;

    if (isVeryCloseToGoal_) {
        costToGo =
          distance_between_points(state.pose.toPoint(), target_.toPoint());   // ignore orientation when very close

        if (isPoseTarget_) {
            headingError = taskParams_.orientationHeuristicWeight
              * angle_diff_abs(target_.theta, state.pose.theta);   // use goal orientation
        } else {
            headingError = 0.0;
        }
    } else if (isApproachingGoal_) {
        // use non-holonomic distance when appropriate
        if (taskParams_.shouldUseNonholonomicDistNearGoal && isPoseTarget_) {
            control_law_coordinates_t coords(state.pose, target_);
            double coordsR = coords.r;
            double coordsTheta = coords.theta;
            double coordsDelta = coords.delta;

            double deltaStar;
            if (!taskParams_.shouldUseSmoothControlHeading) {
                // be numerically safe!
                if (fabs(coordsR * sin(coordsTheta)) < taskParams_.useGoalOrientationRadius) {
                    deltaStar = atan(-taskParams_.kPhi * coordsTheta);
                } else {
                    assert(coordsR != 0.0);
                    deltaStar = atan(-taskParams_.kPhi * coordsTheta / coordsR / coordsR);   // gradient descent
                }
            } else {
                deltaStar = atan(-taskParams_.kPhi * coordsTheta);   // smooth descent
            }

            costToGo = sqrt(coordsR * coordsR + taskParams_.kPhi * taskParams_.kPhi * coordsTheta * coordsTheta);
            headingError = taskParams_.kDelta * angle_diff_abs(deltaStar, coordsDelta);
        } else if (taskParams_.shouldUseNonholonomicDistNearGoal && !isPoseTarget_) {
            costToGo = distance_between_points(state.pose.toPoint(), target_.toPoint());
            headingError = 0.0;
        } else {
            costToGo = getCostToGoFromNavGrid(state);
            headingError = 0.0;
        }
    } else if (isUsingNavGrid_) {
        // default option is to use navgrid
        costToGo = getCostToGoFromNavGrid(state);

        if (isUsingNavGradient_) {
            double gradientHeading = computeNavGradient(state.pose);
            headingError = taskParams_.orientationHeuristicWeight * angle_diff_abs(gradientHeading, state.pose.theta);
        }
    } else {
        std::cerr << "ERROR: NavigationTaskManifold: No valid cost-to-go:\n * Not very close to goal.\n"
                     " * Not approaching goal.\n * Not using navigation grid.\n";
        assert(!"Must have valid cost-to-go value.\n");
    }

    return costToGo + headingError;
}


double NavigationTaskManifold::getCostToGoFromNavGrid(const motion_state_t& state) const
{
    return navGrid_.getCostToGo(state.pose.toPoint());
}


double NavigationTaskManifold::calculateHeuristicCost(const robot_trajectory_info_t& /*robotTrajectoryInfo*/) const
{
    // angle heuristic for the navigation function, as in Park et al. (IROS-12).
    // NOTE: This really is an ad-hoc heuristic to amend the fact that the grid-based
    //       navigation function is a valid path instruction ONLY in 2D configuration
    //       space (that of a circular robot). Should get rid of this by better
    //       estimating the cost-to-go to the goal.

    // NOTE: Now this is updated to reflect how a trajectory is aligned w.r.t.
    //       the underlying vector field (the gradient of the navigation function).

    return 0.0;   // temporary turn off

    //     // compute this only when specified
    //     if(!taskParams_.shouldUseOrientationHeuristic)
    //     {
    //         return 0.0;
    //     }
    //
    //     // distance-to-vector field (the heading error) at the beginning and at the end
    //     // compute and store the heading error at the beginning, which is common to all
    //     // trajectories evaluated within a single cycle, which all have the same timestamp.
    //     pose_t startPose = robotTrajectoryInfo.states.front().pose;
    //
    //     double distanceToGoalAtPose   = distance_between_points(startPose.toPoint(), target_.toPoint());
    //     bool   isGoalWithinSafeRadius = distanceToGoalAtPose <= fmin(taskParams_.goalNeighborhoodRadius,
    //     minDistanceToWallsFromTarget_); bool   isUsingNonholonomicDistance =
    //     taskParams_.shouldUseNonholonomicDistNearGoal && isPoseTarget_ && isGoalWithinSafeRadius;
    //
    //     // compute this only when needed, i.e. when initial heading error is large
    //     // or when using non-holonomic distance
    //     if(!isUsingNonholonomicDistance && initialHeadingError_ < 1.0)
    //     {
    //         // don't need to compute this if the goal is far away, or heading error is small enough
    //         return 0.0;
    //     }
    //
    //     // also don't use this info if the end trajectory is outside the map.
    //     pose_t endPose = robotTrajectoryInfo.states.back().pose;
    //     bool isEndPointInMap  = navGrid_.isPointInGrid(endPose.toPoint(), 0.1);
    //     if(!isEndPointInMap)
    //     {
    //         // check if the end pose is within the map. If not in the map, then this
    //         // heuristic is irrelevant altogether and we can exit early.
    //         return 0.0;
    //     }
    //
    //     // progress made toward aligning to the vector field (finalError - initialError) // the more negative the
    //     better float  finalHeadingError = angle_diff_abs(computeNavGradient(endPose), endPose.theta); double
    //     progressToVectorField = finalHeadingError - initialHeadingError_;
    //
    //     // adaptive weight for faster convergence to the goal
    //     return taskParams_.orientationHeuristicWeight * progressToVectorField;
}


double NavigationTaskManifold::computeNavGradient(const pose_t& pose) const
{
    // in other places follow the gradient of the navigation function, comupted by central gradient.
    // point to cell introduces error up to half grid size, which probably is tolerable.

    // NOTE: this gradient is mathmatically undefined at the goal position.
    Point<int> cell = navGrid_.positionToCell(pose.toPoint());

    float dy =
      (navGrid_.getCostToGo(Point<int>(cell.x, cell.y - 1)) - navGrid_.getCostToGo(Point<int>(cell.x, cell.y + 1)));
    float dx =
      (navGrid_.getCostToGo(Point<int>(cell.x - 1, cell.y)) - navGrid_.getCostToGo(Point<int>(cell.x + 1, cell.y)));

    return atan2(dy, dx);
}


void NavigationTaskManifold::sendDebugInfo(system::ModuleCommunicator& transmitter)
{
    if (lastDebugDataUpdateTimeUs_ + debugDataUpdatePeriodUs_ < utils::system_time_us()) {
        if (haveNewNavGrid_ && builderParams_.shouldDebugGrid) {
            transmitter.sendMessage(navGrid_);
            haveNewNavGrid_ = false;
        }

        if (builderParams_.shouldDebugGrid) {
            transmitter.sendMessage(costMap_);
        }

        lastDebugDataUpdateTimeUs_ = utils::system_time_us();
    }

    transmitter.sendMessage(learnedNormDebug);
}

}   // namespace mpepc
}   // namespace vulcan

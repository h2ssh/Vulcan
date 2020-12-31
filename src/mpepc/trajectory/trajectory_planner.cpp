/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file      trajectory_planner.cpp
* \author    Jong Jin Park
*
* Definition of TrajectoryPlanner.
*/

#include "mpepc/trajectory/trajectory_planner.h"
#include "mpepc/trajectory/trajectory_planner_info.h"
#include "mpepc/trajectory/params.h"
#include "mpepc/grid/obstacle_distance_grid.h"
#include "utils/timestamp.h"
#include <memory>

// #define DEBUG_TRAJECTORY_PLANNER
// #define DEBUG_COLLISION_MODEL

#ifdef DEBUG_COLLISION_MODEL
    #include "mpepc/collision/robot_collision_model.h"
#endif

namespace vulcan
{

namespace mpepc
{

TrajectoryPlanner::TrajectoryPlanner(const trajectory_planner_params_t& params)
: simulator_(params.simulatorParams)
, evaluator_(params.evaluatorParams, params.robotBodyParams)
, optimizer_(params.optimizerParams)
, poseFollower_(params.optimizerParams)
, params_(params)
, isInPoseFollowingMode_(false)
{
    // let optimizer know which simulator and evaluator to work with.
    optimizer_.setup(simulator_, evaluator_);
    poseFollower_.setup(simulator_,evaluator_);
}

trajectory_planner_output_t TrajectoryPlanner::run(const TaskManifold&                             task,
                                                   const motion_state_t                     robotState,
                                                   const ObstacleDistanceGrid&                     map,
                                                   const std::vector<dynamic_object_trajectory_t>& objects,
                                                   const motion_target_t&                          previousMotionTarget,
                                                   int64_t                                         planReleaseTimeUs,
                                                   trajectory_planner_debug_info_t&                debugInfo)
{
    trajectory_planner_output_t plannerOutput;

    int64_t tic = utils::system_time_us();

//     // save goal information
//     debugInfo.haveGoalPose = true;
//     debugInfo.goalPose = task.target();

    // initialize simulator state and get debug info
    debugInfo.trajectoryToInitialState = simulator_.setupForOptimization(robotState,
                                                                         previousMotionTarget,
                                                                         planReleaseTimeUs);

    // setup evaluator with the task, map and estimated object trajecotries
    evaluator_.setTaskEnvironment(task, map, objects);

    // NOTE: Now the robot collision model lives within evaluator only. Maybe it
    //       is a good idea to have the simluator to have it as well and detect
    //       collisoin there, in order to make trajectories stop grow after a
    //       collision is detected. How it will effect the cost sufrace, and
    //       weather it will evntually help the optimizer find the optimal solution,
    //       is not studied yet.

    // give optimizer initial guesses to work with. Namely, the goal and the previous optimium.
    optimizer_.setInitialGuesses(task, previousMotionTarget);
    poseFollower_.setInitialGuesses(task, previousMotionTarget);

    // actual optimization routine. It will populate the debug info during the process.
    // iteration count and the vector of evaluated trajectories are updated within the optimizer.

    motion_target_t optimizedTarget;
    if(isInPoseFollowingMode_)
    {
        optimizedTarget = poseFollower_.runOptimizer(debugInfo);
        plannerOutput.motionTarget = optimizedTarget;
        plannerOutput.isGood       = poseFollower_.haveFoundSolution();
    }
    else
    {
        optimizedTarget = optimizer_.runOptimizer(debugInfo);
        plannerOutput.motionTarget = optimizedTarget;
        plannerOutput.isGood       = optimizer_.haveFoundSolution();
    }

    int64_t toc = utils::system_time_us();

    // save separate copy of the planned trajectory and its evaluation result. This does not increase the iteration count.
    robot_trajectory_info_t plannedTrajectory;
    simulator_.estimateRobotTrajectory(optimizedTarget, plannedTrajectory); // simulate
    evaluator_.evaluateTrajectory(plannedTrajectory); // evaluate
    debugInfo.plannedTrajectory = robot_trajectory_debug_info_t(plannedTrajectory); // save to debug info

    // check initial segment first for safety
    if(debugInfo.trajectoryToInitialState.size() > 0)
    {
        robot_trajectory_info_t initialSegmentInfo;
        initialSegmentInfo.timestep = plannedTrajectory.timestep;
        initialSegmentInfo.states = debugInfo.trajectoryToInitialState;
        evaluator_.evaluateTrajectory(initialSegmentInfo);

        if(initialSegmentInfo.hasDynamicCollision || initialSegmentInfo.hasStaticCollision)
        {
            plannerOutput.isGood = false;
        }
    }

    // if in any case the optimizer returns an output that involves collision, manually set velocity gain to zero so the robot stops.
    while( (plannedTrajectory.hasDynamicCollision || plannedTrajectory.hasStaticCollision || !plannerOutput.isGood)
            && optimizedTarget.velocityGain > 0.0)
    {
        optimizedTarget.velocityGain -= 0.3;

        if(optimizedTarget.velocityGain < 0.0)
        {
            optimizedTarget.velocityGain = 0.0;
        }

        plannedTrajectory.clear();
        simulator_.estimateRobotTrajectory(optimizedTarget, plannedTrajectory); // simulate
        evaluator_.evaluateTrajectory(plannedTrajectory); // evaluate
        debugInfo.plannedTrajectory.assign(plannedTrajectory); // save to debug info

        std::cout<<"WARNING!: TrajectoryPlanner: \n"
                <<"    Optimizer have failed to find a good solution. Reducing the velocity gain.\n";
    }

    plannerOutput.motionTarget = optimizedTarget;

    std::string costType = params_.optimizerParams.useChanceConstraint ? "raw" : "expected";

    std::cout<<"INFO: TrajectoryPlanner: \n"
             <<"    Number of trajectories evaluated: "<< debugInfo.iteration
             <<"  Processing time (ms): "<< (toc - tic) / 1000 <<'\n'
             <<"    Final "<<costType<<" cost from the optimizer : "<< plannedTrajectory.expectedCost <<'\n'
             <<"    Optimized motion target (x,y,theta,gain): ("<< optimizedTarget.pose.x << ',' << optimizedTarget.pose.y << ',' << optimizedTarget.pose.theta << ',' << optimizedTarget.velocityGain << ")\n";

    // populate debug info and output
    debugInfo.updateStartTimeUs    = tic;
    debugInfo.updateEndTimeUs      = toc;
    debugInfo.planReleaseTimeUs    = planReleaseTimeUs;
    debugInfo.receivedMotionState  = robotState;
    debugInfo.clearanceToStaticObs = evaluator_.getClearanceToStaticObstacles(robotState);
    debugInfo.clearanceToDynObs    = evaluator_.getClearanceToDynamicObjects(robotState);

#ifdef DEBUG_TRAJECTORY_PLANNER
    std::cout<<"DEBUG MPEPC INFO: Optimial Trajectory: Initial pose           : "<<debugInfo.trajectoryToInitialState.back().pose<<"\n";

    std::cout<<"DEBUG MPEPC INFO: Optimial Trajectory: Expected cost          : "<<debugInfo.plannedTrajectory.expectedCost <<"\n";
    std::cout<<"DEBUG MPEPC INFO: Optimial Trajectory: Total survivability    : "<<debugInfo.plannedTrajectory.totalSurvivability <<"\n";
    std::cout<<"DEBUG MPEPC INFO: Optimial Trajectory: Expected progress      : "<<debugInfo.plannedTrajectory.expectedProgress <<"\n";
    std::cout<<"DEBUG MPEPC INFO: Optimial Trajectory: Expected collision cost: "<<debugInfo.plannedTrajectory.expectedCollisionCost <<"\n";
    std::cout<<"DEBUG MPEPC INFO: Optimial Trajectory: Expected action cost   : "<<debugInfo.plannedTrajectory.expectedActionCost <<"\n";

    std::cout<<"DEBUG MPEPC INFO: Optimial Trajectory: Piecewise survivability values: (";
    for(auto sampleIt = debugInfo.plannedTrajectory.piecewiseSurvivability.begin(), sampleEnd = debugInfo.plannedTrajectory.piecewiseSurvivability.end();
        sampleIt != sampleEnd;
        sampleIt++)
    {
        std::cout<<*sampleIt<<',';
    }
    std::cout<<")\n";

    std::cout<<"DEBUG MPEPC INFO: Optimial Trajectory: Piecewise progress values: (";
    for(auto sampleIt = debugInfo.plannedTrajectory.piecewiseRawProgress.begin(), sampleEnd = debugInfo.plannedTrajectory.piecewiseRawProgress.end();
        sampleIt != sampleEnd;
        sampleIt++)
    {
        std::cout<<*sampleIt<<',';
    }
    std::cout<<")\n";

    std::cout<<"DEBUG MPEPC INFO: Optimial Trajectory: Piecewise collision cost values: (";
    for(auto sampleIt = debugInfo.plannedTrajectory.piecewiseCollisionCost.begin(), sampleEnd = debugInfo.plannedTrajectory.piecewiseCollisionCost.end();
        sampleIt != sampleEnd;
        sampleIt++)
    {
        std::cout<<*sampleIt<<',';
    }
    std::cout<<")\n";

    std::cout<<"DEBUG MPEPC INFO: Optimial Trajectory: Piecewise action cost values: (";
    for(auto sampleIt = debugInfo.plannedTrajectory.piecewiseActionCost.begin(), sampleEnd = debugInfo.plannedTrajectory.piecewiseActionCost.end();
        sampleIt != sampleEnd;
        sampleIt++)
    {
        std::cout<<*sampleIt<<',';
    }
    std::cout<<")\n";

    std::cout<<"DEBUG MPEPC INFO: Optimial Trajectory: Clearance to static obstacles :"<<debugInfo.clearanceToStaticObs<<"\n";
    std::cout<<"DEBUG MPEPC INFO: Optimial Trajectory: Clearance to dynamic obstacles:"<<debugInfo.clearanceToDynObs<<"\n";

    std::cout<<"DEBUG MPEPC INFO: Optimial Trajectory: Distance to static obstacles from robot center:"<< map.getObstacleDistance(robotState.pose.toPoint()) <<"\n";
#endif

#ifdef DEBUG_COLLISION_MODEL
    // Debugging collision model
    // if not in collision with those crude check walk along boundary and find accurate distance to obstacles
    Point<float> A(0.55, 0.34);
    Point<float> B(0.55, -0.34);
    Point<float> C(-0.50, -0.34);
    Point<float> D(-0.55, 0.34);

    std::vector<Point<float>> boundaryVector {A, B, C, D};

    math::Polygon<float> boundary(boundaryVector);
    Point<float> robotPosition(robotState.pose.toPoint());
    float distanceToObstacle = map.getMaxObstacleDistance();
    float stepSize = map.metersPerCell(); // default step size

    // transform vertices in robot frame to global coordinates
    math::Polygon<float> boundaryInGlobalFrame(boundary);

    boundaryInGlobalFrame.rotate(robotState.pose.theta, Point<float>(0,0));
    boundaryInGlobalFrame.translate(robotState.pose.x, robotState.pose.y);

    // walk along the boundary
    for(auto vertexIt = boundaryInGlobalFrame.begin(); vertexIt != boundaryInGlobalFrame.end()-1; vertexIt++)
    {
        Point<float> stepDirection = *(vertexIt+1) - *vertexIt; // direction to the next vertex
        float              lengthToWalk  = distance_between_points(*vertexIt, *(vertexIt+1));
        Point<float> currentPoint = *vertexIt;

        while(lengthToWalk > stepSize) // move onto next vertex if the distance left to vertex is less than step size.
        {
            float closestObstacleFromCurrentPoint = map.getObstacleDistance(currentPoint);

            std::cout<<"DEBUG: CollisionModelInTrajectoryPlanner: at point: ("<<currentPoint.x<<", "<<currentPoint.y<<"), clearance: " <<closestObstacleFromCurrentPoint<<"\n";

            if(closestObstacleFromCurrentPoint < distanceToObstacle)
            {
                distanceToObstacle = closestObstacleFromCurrentPoint;
            }

            if(distanceToObstacle < 0.001f)
            {
                break;
            }

            // step to next point and compute length left to walk to the next vertex
            Point<float> step(stepDirection.x*stepSize, stepDirection.y*stepSize);
            currentPoint += step;

            lengthToWalk = distance_between_points(currentPoint, *(vertexIt+1));
        }
    }
#endif

    return plannerOutput;
}

} // mpepc
} // vulcan

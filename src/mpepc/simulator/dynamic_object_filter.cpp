/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     dynamic_object_filter.cpp
 * \author   Collin Johnson and Jong Jin Park
 *
 * Definition of DynamicObjectFilter.
 */

#include "mpepc/simulator/dynamic_object_filter.h"
#include "core/motion_state.h"
#include "mpepc/grid/obstacle_distance_grid.h"
#include "tracker/dynamic_object_collection.h"
#include "tracker/objects/person.h"
#include "tracker/objects/rigid.h"
#include "tracker/objects/unclassified.h"
#include "utils/timestamp.h"

#include <boost/variant/static_visitor.hpp>

namespace vulcan
{
namespace mpepc
{

struct object_goal_visitor : public boost::static_visitor<pose_t>
{
    position_t objPosition;
    double objHeading;
    double goalHeading;

    object_goal_visitor(position_t objPosition, double objHeading, double goalHeading)
    : objPosition(objPosition)
    , objHeading(objHeading)
    , goalHeading(goalHeading)
    {
    }

    pose_t operator()(const Line<double>& goal);
    pose_t operator()(const Point<double>& goal);
};


DynamicObjectFilter::DynamicObjectFilter(const dynamic_object_filter_params_t& params) : params_(params)
{
}


std::vector<dynamic_object_trajectory_t>
  DynamicObjectFilter::filterObjects(const tracker::DynamicObjectCollection& objects,
                                     const motion_state_t& robotState,
                                     const ObstacleDistanceGrid& map,
                                     int64_t startTimeUs)
{
    std::vector<dynamic_object_trajectory_t> filteredObjects;

    for (auto& object : objects) {
        // read the state of a tracked object and form a candidate object
        if (startTimeUs - object->timeLastSeen() > params_.staleObjectTimeUs) {
            std::cout << "WARNING!: DynamicObjectFilter: Timestamp of the tracked object is old. This data is ignored \
                        as it may be stale.\n";
        } else {
            dynamic_object_state_t objState = createObjectState(*object);

            // Ignore objects that are far from the robot or too close to walls to matter
            if (isNearRobot(objState, robotState) && isFarFromWalls(objState, map)) {
                // TODO: bunch of heuristic here to make things work. Clean it up!
                // special treatment for things behind the robot
                if (params_.shouldSlowdownObjectsBehindRobot) {
                    objState = slowdownObjectBehindRobot(objState, robotState);
                }

                // cap object speeds
                float speed = std::sqrt(objState.xVel * objState.xVel + objState.yVel * objState.yVel);
                if (speed > params_.maxObjectSpeed) {
                    objState.xVel = objState.xVel / speed * params_.maxObjectSpeed;
                    objState.yVel = objState.yVel / speed * params_.maxObjectSpeed;
                }

                // set start state for trajectory estimation
                float timeToForward = utils::usec_to_sec(startTimeUs - object->timeLastSeen());
                objState.x += objState.xVel * timeToForward;
                objState.y += objState.yVel * timeToForward;

                // initialize dynamic object trajectory and push to storage
                dynamic_object_trajectory_t trajectory;
                trajectory.type = DynamicObjectType::pedestrian;
                trajectory.timestamp = startTimeUs;
                trajectory.priorProbability = 1.0;
                trajectory.laserObject = object->clone();
                trajectory.states.push_back(objState);
                trajectory.goal = estimateGoal(*object);
                trajectory.preferredVel = estimatePreferredVelocity(objState, trajectory.goal);

                std::cout << "Created object at " << objState.x << ',' << objState.y << " Vel: " << objState.xVel << ','
                          << objState.yVel << " Goal:" << trajectory.goal << " Pref vel:" << trajectory.preferredVel
                          << '\n';

                filteredObjects.push_back(trajectory);
            }
        }
    }

    return filteredObjects;
}


void DynamicObjectFilter::visitPerson(const tracker::Person& person)
{
    std::cerr << "ERROR!!: DynamicObjectFilter: Unable to handle person model.\n\n";
    assert(false);
}


void DynamicObjectFilter::visitUnclassified(const tracker::UnclassifiedObject& object)
{
    initialObjectState_ = object.motionState();
}


void DynamicObjectFilter::visitRigid(const tracker::RigidObject& object)
{
    using msi = tracker::MotionStateIndex;

    // ignore spurious velocity estimate based on its uncertainty
    // velocity uncertainty
    auto motionStateWithUncertainty = object.slowMotionState();

    // get the largest eigenvalue of the covariance on velocity
    Matrix velocityCov =
      motionStateWithUncertainty.getCovariance().submat(msi::velXIndex, msi::velXIndex, msi::velYIndex, msi::velYIndex);
    Vector eigVal = arma::eig_sym(velocityCov);

    initialObjectState_ = object.motionState();
    double maxStdDev = std::sqrt(eigVal(1));   // eigenvalues are in ascending order, and is in covariance so scale them
                                               // to get the standard deviation

    if (maxStdDev > params_.maxTrustedVelocityStd) {
        initialObjectState_.xVel = 0.0;
        initialObjectState_.yVel = 0.0;
    } else if (maxStdDev > params_.startUntrustedVelocityStd) {
        double uncertainVelocityScale = 1.0
          - ((maxStdDev - params_.startUntrustedVelocityStd)
             / (params_.maxTrustedVelocityStd - params_.startUntrustedVelocityStd));
        initialObjectState_.xVel *= uncertainVelocityScale;
        initialObjectState_.yVel *= uncertainVelocityScale;
    }
}


void DynamicObjectFilter::visitPivotingObject(const tracker::PivotingObject& door)
{
    std::cerr << "ERROR!!: DynamicObjectFilter: Unable to handle pivoting object model.\n\n";
    assert(false);
}


void DynamicObjectFilter::visitSlidingObject(const tracker::SlidingObject& door)
{
    std::cerr << "ERROR!!: DynamicObjectFilter: Unable to handle sliding object model.\n\n";
    assert(false);
}


dynamic_object_state_t DynamicObjectFilter::createObjectState(const tracker::DynamicObject& trackedObject)
{
    trackedObject.accept(*this);
    return initialObjectState_;
}


bool DynamicObjectFilter::isNearRobot(const dynamic_object_state_t& objectState, const motion_state_t& robotState)
{
    return true;   // temporary turn-off

    // TODO: the distance threshod and the lookahead time perhaps should be a function of maximum velocity of the robot
    // and the planning horizon.
    const float DISTANCE_THRESHOLD = 7.5f;   // meters
    const float LOOKAHEAD_TIME = 5.0f;       // second

    // relative distance, speed and orientation
    float relativeX = objectState.x - robotState.pose.x;
    float relativeY = objectState.y - robotState.pose.y;
    float relativeDistance = std::sqrt(relativeX * relativeX + relativeY * relativeY);

    float normalizedRelativeX = relativeX / relativeDistance;
    float normalizedRelativeY = relativeY / relativeDistance;

    //     // relative orientation
    //     float lineOfSightOrientation = atan2(objectState.y - robotState.pose.y, objectState.x - robotState.pose.x);
    //     float relativeOrientation    = wrap_to_pi(lineOfSightOrientation - robotState.pose.theta);

    // relative speed
    float xVelRelative = objectState.xVel - (robotState.velocity.linear * cos(robotState.pose.theta));
    float yVelRelative = objectState.yVel - (robotState.velocity.linear * sin(robotState.pose.theta));

    // inner product of the relative velocity of an object toward the robot and the negative of the direction of the
    // line of sight gives the approach speed of the object toward the robot.
    float approachSpeed = (xVelRelative * -normalizedRelativeX) + (yVelRelative * -normalizedRelativeY);

    return (relativeDistance - (approachSpeed * LOOKAHEAD_TIME)) < DISTANCE_THRESHOLD;
}


bool DynamicObjectFilter::isFarFromWalls(const dynamic_object_state_t& objectState, const ObstacleDistanceGrid& map)
{
    return true;   // temporary turn-off

    //     Point<int> objectLocationInCell = map.positionToCell(Point<float>(objectState.x, objectState.y));
    //
    //     return map.getObstacleDistance(objectLocationInCell) < 0.1; // TODO: remove this hard coded constant to
    //     config!
}


dynamic_object_state_t DynamicObjectFilter::slowdownObjectBehindRobot(const dynamic_object_state_t& objectState,
                                                                      const motion_state_t& robotState)
{
    dynamic_object_state_t slowedState = objectState;

    // relative orientation in global reference frame (angle of line of sight)
    float lineOfSightOrientation = std::atan2(objectState.y - robotState.pose.y, objectState.x - robotState.pose.x);

    // relative distance and orientation in robot frame
    float distToObject = distance_between_points(objectState.x, objectState.y, robotState.pose.x, robotState.pose.y);
    // is 0 when the object is directly behind the robot.
    float headingToObject = M_PI - std::abs(wrap_to_pi(lineOfSightOrientation - robotState.pose.theta));

    // Is the object in the slowdown cone?
    if ((headingToObject < params_.slowdownObjectConeAngle) && (distToObject < 10.0)) {
        // underestimate velocities of objects within the slowdown cone
        float slowdownFactor = (0.5 * headingToObject) / (params_.slowdownObjectConeAngle + 0.5);
        slowedState.xVel *= slowdownFactor;
        slowedState.yVel *= slowdownFactor;

        // push back objects directly behind the robot by some amount so that it doesn't scare the robot.
        if (distToObject < params_.ignoreObjectConeRadius) {
            slowedState.x += params_.ignoreObjectConeRadius * std::cos(lineOfSightOrientation);
            slowedState.y += params_.ignoreObjectConeRadius * std::sin(lineOfSightOrientation);
        }
    }

    return slowedState;
}


pose_t DynamicObjectFilter::estimateGoal(const tracker::DynamicObject& trackedObject)
{
    auto objectGoal = trackedObject.goals().bestGoal();

    // Do we trust this goal?
    if (objectGoal.probability() > params_.minGoalProbability) {
        object_goal_visitor goalVisitor(trackedObject.position(),
                                        std::atan2(trackedObject.velocity().y, trackedObject.velocity().x),
                                        objectGoal.heading());
        return objectGoal.destination().apply_visitor(goalVisitor);
    }
    // Stick with ballistic velocity estimate
    else {
        auto state = trackedObject.motionState();
        return pose_t(state.x + (state.xVel * 10.0), state.y + (state.yVel * 10.0), std::atan2(state.yVel, state.xVel));
    }
}


Point<float> DynamicObjectFilter::estimatePreferredVelocity(const dynamic_object_state_t& objectState,
                                                            const pose_t& objectGoal)
{
    Point<float> preferredVelocity;

    // The preferred velocity has the same heading as the goal state and the same magnitude as the object state
    double speed = std::sqrt((objectState.xVel * objectState.xVel) + (objectState.yVel * objectState.yVel));

    // TODO: preferred velocity vector should be computed using uncertainty weights, accelerations, and maybe slow
    // states.
    preferredVelocity.x = speed * std::cos(objectGoal.theta);
    preferredVelocity.y = speed * std::sin(objectGoal.theta);

    return preferredVelocity;
}


pose_t object_goal_visitor::operator()(const Line<double>& goal)
{
    // Assume that the object will travel through the right side of the gateway line. The right side is
    // determined by the goal heading

    Line<double> headingLine;
    headingLine.a = objPosition;
    headingLine.b.x = headingLine.a.x + (std::cos(goalHeading) * 1000);
    headingLine.b.y = headingLine.a.y + (std::sin(goalHeading) * 1000);

    double dx = goal.b.x - goal.a.x;
    double dy = goal.b.y - goal.a.y;

    const int kNumSteps = 10;
    const double kOffset = 0.1;
    const double kStepX = dx * (1.0 - 2.0 * kOffset) / kNumSteps;
    const double kStepY = dy * (1.0 - 2.0 * kOffset) / kNumSteps;

    // Consider one of three possible goals along the boundary line
    // The agent is assumed to be going to the closest of the three
    std::vector<Point<double>> goals;
    for (int n = 0; n < kNumSteps; ++n) {
        goals.emplace_back(goal.a.x + (dx * kOffset) + (kStepX * n), goal.a.y + (dy * kOffset) + (kStepY * n));
    }

    pose_t goalPose;
    goalPose.theta = goalHeading;

    auto bestGoalIt = std::min_element(goals.begin(), goals.end(), [&](const auto& lhs, const auto& rhs) {
        return angle_diff_abs(angle_to_point(objPosition, lhs), objHeading)
          < angle_diff_abs(angle_to_point(objPosition, rhs), objHeading);
    });

    goalPose.x = bestGoalIt->x;
    goalPose.y = bestGoalIt->y;

    return goalPose;
}


pose_t object_goal_visitor::operator()(const Point<double>& goal)
{
    return pose_t(goal, goalHeading);
}

}   // namespace mpepc
}   // namespace vulcan

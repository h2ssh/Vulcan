/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     dynamic_object_simulator.cpp
* \author   Jong Jin Park
*
* Definition of DynamicObjectSimulator
*
*/

#include <mpepc/simulator/dynamic_object_simulator.h>
#include <mpepc/grid/obstacle_distance_grid.h>
#include <core/motion_state.h>
#include <tracker/dynamic_object_collection.h>
#include <tracker/objects/person.h>
#include <tracker/objects/rigid.h>
#include <tracker/objects/unclassified.h>
#include <tracker/objects/pivoting_object.h>
#include <tracker/objects/sliding_object.h>
#include <utils/timestamp.h>

// #define DEBUG_DYNAMIC_OBJECT_SIMULATOR_TIME
#ifdef DEBUG_DYNAMIC_OBJECT_SIMULATOR_TIME
    #include <utils/timestamp.h>
#endif

namespace vulcan
{
namespace mpepc
{

bool have_reached_goal(const dynamic_object_state_t& objectState, const pose_t& objectGoal);
double goal_alignment_score(const dynamic_object_state_t& objectState, const pose_t& objectGoal);
double velocity_alignment_score(const dynamic_object_state_t& objectState, const Point<float>& goalVelocity);


DynamicObjectSimulator::DynamicObjectSimulator(const dynamic_object_simulator_params_t& params)
: simulatorTimeStep_(0.05)
, trajectoryTimeLength_(5.0)
, shouldIgnoreObjectVelocities_(false)
, params_(params)
{
}


void DynamicObjectSimulator::estimateObjectTrajectories(std::vector<dynamic_object_trajectory_t>& objects,
                                                        const motion_state_t&              robotState,
                                                        const ObstacleDistanceGrid&               map,
                                                        int64_t                                   startTimeUs)
{
    // initialize robot state
    dynamic_object_state_t robotObjectState(robotState.pose.x,
                                            robotState.pose.y,
                                            robotState.velocity.linear * std::cos(robotState.pose.theta),
                                            robotState.velocity.linear * std::sin(robotState.pose.theta),
                                            0.0,
                                            0.0);

    // propagate object states over specified duration
    const int numSteps = std::lrint(trajectoryTimeLength_ / simulatorTimeStep_);

    // Create the robot trajectory
    predictedRobotTraj_.clear();
    for(int n = 1; n < numSteps; ++n)
    {
        // propagate robot state (as a dynamic object)
        float timeToForward = simulatorTimeStep_;
        if(n == 1)
        {
            timeToForward = utils::usec_to_sec(startTimeUs - robotState.timestamp);
        }

        // only the robot state needs variable time-forwarding because it is not initalized in the initializeSimulatorStates...
        // note that the object states are propagated using robot state information that is one step behind.
        robotObjectState = propagateObjectState(robotObjectState,
                                                0.5,
                                                map,
                                                nullptr,
                                                Point<float>(robotObjectState.xVel, robotObjectState.yVel),
                                                nullptr,
                                                simulatorTimeStep_ * n,
                                                timeToForward);

//         double speed = std::sqrt(std::pow(robotObjectState.xVel, 2.0) + std::pow(robotObjectState.yVel, 2.0));
//         if (speed > 0.0)
//         {
//             robotObjectState.xVel /= speed;
//             robotObjectState.yVel /= speed;
//         }

//         robotObjectState.x = robotState.pose.x;
//         robotObjectState.y = robotState.pose.y;
//         robotObjectState.xVel = robotState.velocity.linear * std::cos(robotState.pose.theta),
//         robotObjectState.yVel = robotState.velocity.linear * std::sin(robotState.pose.theta),

        predictedRobotTraj_.push_back(robotObjectState);
    }

    // propagate object states
    for(auto& obj : objects)
    {
        for(int n = 1; n < numSteps; ++n)
        {
            dynamic_object_state_t nextState = propagateObjectState(obj.states.back(),
                                                                    obj.radius,
                                                                    map,
//                                                                     nullptr,
                                                                    &(predictedRobotTraj_[n]),
                                                                    obj.preferredVel,
                                                                    &obj.goal,
                                                                    simulatorTimeStep_ * n,
                                                                    simulatorTimeStep_);
            obj.states.push_back(nextState);
        }
    }
}


dynamic_object_state_t DynamicObjectSimulator::propagateObjectState(const dynamic_object_state_t& objectState,
                                                                    float objectRadius,
                                                                    const ObstacleDistanceGrid& map,
                                                                    const dynamic_object_state_t* robotState,
                                                                    const Point<float>& preferredVel,
                                                                    const pose_t* objectGoal,
                                                                    float timeElapsed,
                                                                    float timeStep)
{
    dynamic_object_state_t nextState = objectState;

    // propagate states of object under a controller and a plant model
    // predictive velocity controller for pedestrian motion decisions.
    if(params_.shouldPredictObjectVelocities)// && (timeElapsed >= params_.reactionTime))
    {
        auto nextVel = objectVelocityDecider(objectState,
                                             objectRadius,
                                             map,
                                             robotState,
                                             preferredVel,
                                             objectGoal,
                                             timeElapsed,
                                             timeStep);
        nextState.xVel = nextVel.x;
        nextState.yVel = nextVel.y;
    }

    //constant-velocity model of pedestrian motion dynamics.
    nextState.x += nextState.xVel * timeStep;
    nextState.y += nextState.yVel * timeStep;

    return nextState;
}


// object controller model (velocity decider)
Point<float> DynamicObjectSimulator::objectVelocityDecider(const dynamic_object_state_t& objectState,
                                                                 float objectRadius,
                                                                 const ObstacleDistanceGrid& map,
                                                                 const dynamic_object_state_t* robotState,
                                                                 const Point<float>& preferredVel,
                                                                 const pose_t* objectGoal,
                                                                 float timeElapsed,
                                                                 float timeStep)
{
    Point<float> nextVel(objectState.xVel, objectState.yVel);

    if(timeElapsed < 0.1)
    {
        return nextVel;
    }

    timeStep *= 5;

    // initial distance at the beginning
    float initialDistToStatic = map.getObstacleDistance(Point<float>(objectState.x, objectState.y));

    // if initial distance is already too small simply reduce speed and exit
    // keeps objects from shooting into walls
    if(initialDistToStatic < 0.05)
    {
        nextVel.x *= 0.5f;
        nextVel.y *= 0.5f;
        return nextVel;
    }

    // check decisions and costs
    float kRobotRadius = 0.5; // assuming circular robot

    // action space definition
    const float kMinRotation = -M_PI * 3.0 / 180.0;
    const float kMaxRotation = -kMinRotation;
    const int   kRotationSteps = 25;
    const float kRotationIncr = (kMaxRotation - kMinRotation) / kRotationSteps;

    // uncertainty estimate
    float kUncertaintyStd = 0.02;

    // cost definition
    const double kActionWeight = 0.1;
    const double kCollisionCostWeight = 0.25;
//     const double kMinHeadingWeight = 0.5;

    // If nothing good found, just stick with previous velocity
    float optXVel = objectState.xVel;
    float optYVel = objectState.yVel;
    float optCost = 10000.0;
//     float optTheta = 0.0f;
    dynamic_object_state_t optState = objectState;

    bool haveReachedGoal = !objectGoal || have_reached_goal(objectState, *objectGoal);
    double initialGoalScore = !objectGoal ? 0.0 : goal_alignment_score(objectState, *objectGoal);

    // Speed is unchanged by the direction changes here
    float speedNormalizer = std::sqrt(std::pow(objectState.xVel, 2.0) + std::pow(objectState.yVel, 2.0));
    if (speedNormalizer > 0.0)
    {
        speedNormalizer = 1.0 / speedNormalizer;
    }

    // Where is robot relative to object?
    Point<float> robotDirVector;
    if(robotState)
    {
        robotDirVector.x = robotState->x - objectState.x;
        robotDirVector.y = robotState->y - objectState.y;
        double norm = point_norm(robotDirVector);
        if(norm > 0.0)
        {
            robotDirVector.x /= norm;
            robotDirVector.y /= norm;
        }
    }

    // turning options
//     std::cout << "Next step from " << objectState.x << ',' << objectState.y << ":\n";
    for(int step = 0; step < kRotationSteps; ++step)
    {
        float theta = kMinRotation + (step * kRotationIncr);
        dynamic_object_state_t possibleState = objectState;

        // rotate the velocity by the rotation amount
        possibleState.xVel = (objectState.xVel * std::cos(theta)) - (objectState.yVel * std::sin(theta));
        possibleState.yVel = (objectState.xVel * std::sin(theta)) + (objectState.yVel * std::cos(theta));

        // object state after lookahead time
        possibleState.x += possibleState.xVel * timeStep;
        possibleState.y += possibleState.yVel * timeStep;

        // squared action cost
        float actionCost = kActionWeight * kActionWeight * theta * theta;

        // dynamic collision cost
        double dynamicCollisionProb = 0.0;
        // TODO: Currently only considering collisions with the robot
        if(robotState)
        {
            double distToRobot = distance_between_points(possibleState.x,
                                                               possibleState.y,
                                                               robotState->x,
                                                               robotState->y);
            distToRobot = std::max(distToRobot - kRobotRadius - objectRadius, 0.0);
            dynamicCollisionProb = std::exp(-std::pow(distToRobot / kUncertaintyStd, 2.0));

            // If not already hitting the robot, then weight based on the relative position of the robot
//             if(distToRobot > 0.0)
//             {
//                 // heading looks at whether object is pointed at the robot, which increases probability
//                 // of collision -- basically the ship navigation paradigm
//                 // range [0 = same direction, 1 = head on opposite direction]
//                 // Dot product will be -1 if aimed right at each other
//                 double headingDotProd = (robotDirVector.x * possibleState.xVel * speedNormalizer)
//                     + (robotDirVector.y * possibleState.yVel * speedNormalizer);
//                 // Need to flip the range from [-1, 1] to [1, 0]
//                 double robotRelativeHeading = std::max((0.5 * (1.0 - headingDotProd)), kMinHeadingWeight);
//
//                 dynamicCollisionProb *= robotRelativeHeading;
//             }
        }

        // static collision cost
        float distToStatic = map.getObstacleDistance(Point<float>(possibleState.x, possibleState.y));
        distToStatic = std::max(distToStatic - objectRadius, 0.0f);
        double staticCollisionProb = std::exp(-std::pow(distToStatic / kUncertaintyStd, 2.0));

        // compute the probabiliy of collision and the collision cost
        double collisionCost = (staticCollisionProb + dynamicCollisionProb) * kCollisionCostWeight;

        // estimated expected progress
        double progress = 0.0;
        if(haveReachedGoal)
        {
            // If at the goal, then use the preferred velocity to determine the progress
            // Reward alignment with the prefered velocity
            progress = velocity_alignment_score(possibleState, preferredVel);
        }
        else
        {
            progress = initialGoalScore - goal_alignment_score(possibleState, *objectGoal);
        }

        progress *= (1.0 - staticCollisionProb) * (1.0 - dynamicCollisionProb);

        double cost = actionCost + collisionCost - progress;

        if((cost < optCost) && (staticCollisionProb < 1.0) && (dynamicCollisionProb < 1.0))
        {
            optXVel = possibleState.xVel;
            optYVel = possibleState.yVel;
            optCost = cost;
//             optTheta = theta;
            optState = possibleState;

//             std::cout << "opt: " << optCost << " theta: " << optTheta << " pos:" << optState.x << ',' << optState.y
//                 << '\n';
        }
    }

    nextVel.x = optXVel;
    nextVel.y = optYVel;

//     if (objectGoal)
//     {
//         std::cout << "Optimal: at goal: " << haveReachedGoal << " initial: " << initialGoalScore << " final:"
//             << goal_alignment_score(optState, *objectGoal) << " cost:" << optCost << " theta: " << optTheta << '\n';
//     }

    return nextVel;
}


bool have_reached_goal(const dynamic_object_state_t& objectState, const pose_t& objectGoal)
{
    double angleToObject = angle_diff_abs(angle_to_point(objectGoal.toPoint(),
                                                                     Point<float>(objectState.x,
                                                                                        objectState.y)),
                                                objectGoal.theta);
    // Have we crossed the plane defined by the goal and aren't really far away (where the goal might be inconsistent
    return (angleToObject < M_PI_2)
        && distance_between_points(objectState.x, objectState.y, objectGoal.x, objectGoal.y) < 10.0;
}


double goal_alignment_score(const dynamic_object_state_t& objectState, const pose_t& objectGoal)
{
    // Just use sum of heading and distance
    double distToGoal = distance_between_points(objectState.x, objectState.y, objectGoal.x, objectGoal.y);
    double headingErr = angle_diff_abs(std::atan2(objectState.yVel, objectState.xVel), objectGoal.theta);

    return distToGoal + (0.02 * headingErr);
}


double velocity_alignment_score(const dynamic_object_state_t& objectState, const Point<float>& goalVelocity)
{
    return (goalVelocity.x * objectState.xVel) + (goalVelocity.y * objectState.yVel);
}

} // planner
} // vulcan

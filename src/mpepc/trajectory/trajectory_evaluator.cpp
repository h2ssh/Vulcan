/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     trajectory_evaluator.cpp
 * \author   Jong Jin Park
 *
 * Definition of TrajectoryEvaluator.
 */

#include "mpepc/trajectory/trajectory_evaluator.h"
#include "mpepc/grid/obstacle_distance_grid.h"
#include "mpepc/manifold/task_manifold.h"
#include "mpepc/trajectory/robot_trajectory_info.h"
#include "robot/model/params.h"
#include <iostream>

// #define DEBUG_TRAJECTORY_EVALUATOR

namespace vulcan
{

namespace mpepc
{

TrajectoryEvaluator::TrajectoryEvaluator(const trajectory_evaluator_params_t& params,
                                         const robot::collision_model_params_t& robotParams)
: robot_(create_robot_collision_model(robotParams))
, params_(params)
{
}


void TrajectoryEvaluator::setTaskEnvironment(
  const TaskManifold& taskManifold,
  const ObstacleDistanceGrid& obstacleDistanceGrid,
  const std::vector<dynamic_object_trajectory_t>& estimatedObjectTrajectories)
{
    task_ = &taskManifold;
    map_ = &obstacleDistanceGrid;
    objects_ = estimatedObjectTrajectories;
}


float TrajectoryEvaluator::getClearanceToStaticObstacles(const motion_state_t& robotState)
{
    float clearance;

    robot_->setRobotPose(robotState.pose);
    clearance = robot_->closestObstacle(*map_);

    return clearance;
}


float TrajectoryEvaluator::getClearanceToDynamicObjects(const motion_state_t& robotState)
{
    float clearance = 10.0;

    robot_->setRobotPose(robotState.pose);

    for (auto& obj : objects_) {
        // find distance to object assuming circlular objects
        // orientation of object unimportant for circle model
        pose_t objectPose(obj.laserObject->position(), 0.0);
        float distanceToObjectFromRobot = robot_->distanceToObject(objectPose, obj.radius);
        clearance = std::min(clearance, distanceToObjectFromRobot);
    }

    return clearance;   // if there is no object just return some large value
}


double TrajectoryEvaluator::evaluateTrajectory(robot_trajectory_info_t& robotTrajectory)
{
    // discrete time increment for trajectory evaluation (with sub-sampling)
    size_t increment = std::max(1.0f, params_.timeBetweenSamples / robotTrajectory.timestep);
    float timestep = params_.timeBetweenSamples;

    ////////////////////////////////////////////////////////////////////////////
    // compute survivability, and collision cost over the trajectory
    double survivability = 1.0;   // survivability at each trajectory segment (cumulative).
    double collisionCost = 0.0;   // expected collision cost at each trajectory segment.

    // initializing collision indicators
    float timeElapsed = 0.0f;
    robotTrajectory.hasStaticCollision = false;
    robotTrajectory.hasDynamicCollision = false;
    robotTrajectory.staticCollisionTime = -1.0f;
    robotTrajectory.dynamicCollisionTime = -1.0f;

    // initializing robot and object uncertainties
    staticObjectUncertaintySigma_ = params_.staticUncertaintyStdMin;
    for (auto& objectTrajectoryIt : objects_) {
        objectTrajectoryIt.uncertaintyStd = 4.0 * params_.dynamicUncertaintyStdMin;
    }

    for (size_t index = increment; index < robotTrajectory.states.size(); index += increment) {
        // time elapsed
        timeElapsed = index * timestep;

        // set robot pose
        robot_->setRobotPose(robotTrajectory.states[index].pose);

        ////////////////////////////////////////////////////////////////////////
        // piecewise survivability with respect to static obstacles
        double piecewiseStaticSurvivability;
        double proximityToStaticObj;
        if (robotTrajectory.hasStaticCollision) {
            // don't need to check for collision if the robot is already in collision!
            piecewiseStaticSurvivability = 0.0;
            proximityToStaticObj = 0.0;   // this is incorrect but this value is not used further, so it's OK.
        } else {
            // normal operation
            piecewiseStaticSurvivability = 1.0
              - piecewiseCollisionProbability(robotTrajectory.states[index].pose,
                                              robotTrajectory.states[index].velocity,
                                              timestep,
                                              &staticObjectUncertaintySigma_,
                                              &proximityToStaticObj);
        }

        ////////////////////////////////////////////////////////////////////////
        // piecewise survivability with respect to dynamic objects
        double piecewiseDynamicSurvivability = 1.0;
        double proximityToDynamicObj = 100.0;
        if (robotTrajectory.hasDynamicCollision) {
            // don't need to check for collision if the robot is already in collision!
            piecewiseDynamicSurvivability = 0.0;
            proximityToDynamicObj = 0.0;
        } else {
            // normal opertation which iterates over all relevant dynamic objects
            for (auto objectTrajectoryIt = objects_.begin(); objectTrajectoryIt != objects_.end();
                 ++objectTrajectoryIt) {
                double objectRadius =
                  params_.useFixedObjectRadius ? params_.fixedObjectRadius : objectTrajectoryIt->laserObject->radius();
                objectRadius = fmax(params_.fixedObjectRadius, objectRadius);

                double distanceToIndividualDynamicObj = 100.0;
                double objectCollisionProb = piecewiseCollisionProbability(
                  robotTrajectory.states[index].pose,
                  robotTrajectory.states[index].velocity,
                  staticObjectUncertaintySigma_,
                  objectTrajectoryIt->states[index],
                  objectRadius,   // this may need to get expanded into laserObject for more sophisticated stuff.
                  timestep,
                  &((*objectTrajectoryIt).uncertaintyStd),
                  &distanceToIndividualDynamicObj);
                // low-probability objects have less probability of collision
                objectCollisionProb *= objectTrajectoryIt->priorProbability;

                if (params_.shouldConsiderOnlyTheClosestObject) {
                    piecewiseDynamicSurvivability = std::min(piecewiseDynamicSurvivability, 1.0 - objectCollisionProb);
                } else {
                    piecewiseDynamicSurvivability *= 1.0 - objectCollisionProb;
                }

                // the distance to the nearest dynamic object
                proximityToDynamicObj = std::min(proximityToDynamicObj, distanceToIndividualDynamicObj);
            }   // end iteration over dynamic objects
        }

        ////////////////////////////////////////////////////////////////////////
        // compute the survivability for this trajectory segment
        // NOTE: IMPORTANT! the survivability is multiplied through!
        if (params_.shouldConsiderOnlyTheClosestObject) {
            // This version means we only care about the most threatening object at each trajectory segment, either
            // static or dynamic
            survivability *= std::min(piecewiseStaticSurvivability, piecewiseDynamicSurvivability);
        } else {
            // This considers everything at the same time, making it more conservative.
            survivability *= piecewiseStaticSurvivability * piecewiseDynamicSurvivability;
        }


        ////////////////////////////////////////////////////////////////////////
        // compute the expected cost of collision
        collisionCost = piecewiseCollisionCost(robotTrajectory.states[index], params_.timeBetweenSamples);
        robotTrajectory.piecewiseCollisionCost.push_back((1.0 - survivability) * collisionCost);   // expected value


        ////////////////////////////////////////////////////////////////////////
        // update collision indicators
        if ((piecewiseStaticSurvivability < 0.001) && (robotTrajectory.staticCollisionTime < 0.0f)
            && (!robotTrajectory.hasStaticCollision)) {
            robotTrajectory.hasStaticCollision = true;
            robotTrajectory.staticCollisionTime = timeElapsed;
        }

        if ((piecewiseDynamicSurvivability < 0.001) && (robotTrajectory.dynamicCollisionTime < 0.0f)
            && (!robotTrajectory.hasDynamicCollision)) {
            robotTrajectory.hasDynamicCollision = true;
            robotTrajectory.dynamicCollisionTime = timeElapsed;
        }


        ////////////////////////////////////////////////////////////////////////
        // save to robot trajectory info
        robotTrajectory.distanceToStaticObj.push_back(proximityToStaticObj);   // distance to static object
        robotTrajectory.distanceToDynamicObj.push_back(
          proximityToDynamicObj);                                          // distance to the nearest dynamic object
        robotTrajectory.piecewiseSurvivability.push_back(survivability);   // survivability

#ifdef DEBUG_TRAJECTORY_EVALUATOR
        if (index == increment) {
            pose_t objectPose =
              pose_t(objectTrajectoryIt->states[index].x,
                     objectTrajectoryIt->states[index].y,
                     atan2(objectTrajectoryIt->states[index].yVel, objectTrajectoryIt->states[index].xVel));
            std::cerr << "Start pose:  " << robotTrajectory.states[index].pose << " Object pose: " << objectPose
                      << " Object velocity: " << objectTrajectoryIt->states[index].xVel << " , "
                      << objectTrajectoryIt->states[index].yVel << " Object speed: "
                      << sqrt(objectTrajectoryIt->states[index].xVel * objectTrajectoryIt->states[index].xVel
                              + objectTrajectoryIt->states[index].yVel * objectTrajectoryIt->states[index].yVel)
                      << " Distance-0: " << robot_->distanceToObject(objectPose, 0)
                      << " Distance-R: " << robot_->distanceToObject(objectPose, objectRadius)
                      << " Prob: " << piecewiseDynamicSurvivability << std::endl;

        } else if (index >= (robotTrajectory.states.size() - increment)) {
            pose_t objectPose =
              pose_t(objectTrajectoryIt->states[index].x,
                     objectTrajectoryIt->states[index].y,
                     atan2(objectTrajectoryIt->states[index].yVel, objectTrajectoryIt->states[index].xVel));
            std::cerr << "End pose:  " << robotTrajectory.states[index].pose << " Object pose: " << objectPose
                      << " Object velocity: " << objectTrajectoryIt->states[index].xVel << " , "
                      << objectTrajectoryIt->states[index].yVel << " Object speed: "
                      << sqrt(objectTrajectoryIt->states[index].xVel * objectTrajectoryIt->states[index].xVel
                              + objectTrajectoryIt->states[index].yVel * objectTrajectoryIt->states[index].yVel)
                      << " Distance-0: " << robot_->distanceToObject(objectPose, 0)
                      << " Distance-R: " << robot_->distanceToObject(objectPose, objectRadius)
                      << " Prob: " << piecewiseDynamicSurvivability << std::endl;
        }
#endif
    }

    ////////////////////////////////////////////////////////////////////////////
    // compute raw progress over the trajectory
    task_->calculateNegativeRewardsOverTrajectory(robotTrajectory.states,
                                                  robotTrajectory.timestep,
                                                  increment,
                                                  robotTrajectory.piecewiseRawProgress);   // populates raw progress
    robotTrajectory.heuristicCost =
      task_->calculateHeuristicCost(robotTrajectory) * survivability;   // convert to expected cost

    ////////////////////////////////////////////////////////////////////////////
    // action cost, path length, and travel cost
    double actionCost = 0.0;   // expected action cost at each trajectory segment.
    double pathLength = 0.0;   // Integrated length of the path, not considering collision.
    double pathCost = 0.0;     // Integrated cost of travel along the path, not considering collision.
    for (size_t index = increment; index < robotTrajectory.states.size(); index += increment) {
        // action cost
        actionCost = piecewiseActionCost(robotTrajectory.states[index].velocity,
                                         robotTrajectory.states[index].acceleration,
                                         params_.timeBetweenSamples);
        robotTrajectory.piecewiseActionCost.push_back(actionCost);   // expected value

        // path length and travel cost
        for (size_t decrement = increment; decrement > 0; decrement--) {
            integratePathLength(pathLength, robotTrajectory.states[index - decrement].velocity.linear, timestep);
            integratePathCost(pathCost, robotTrajectory.states[index - decrement].velocity, timestep);
        }
    }


    ////////////////////////////////////////////////////////////////////////////
    // integrating over the trajectory for expected total costs
    double totalProgress =
      0.0;   // total expected progress over the entire trajectory (with additive heuristic cost, if one exists.)
    double totalCollisionCost = 0.0;      // total expected collision cost over the entire trajectory.
    double totalActionCost = 0.0;         // total expected action cost over the entire trajectory.
    double timeDecayFactorGamma = 0.97;   // 0.97^25 ~ 0.467  0.98^25 ~ 0.60 ~ 0.78
    for (size_t i = 0; i < robotTrajectory.piecewiseRawProgress.size(); i++) {
        double timeDecayFactor = pow(timeDecayFactorGamma, i);

        if (params_.useChanceConstraint)   // using chance constraint ignores the survivability and the expected cost of
                                           // constraint violation
        {
            totalProgress += timeDecayFactor * robotTrajectory.piecewiseRawProgress[i];
            totalActionCost += timeDecayFactor * robotTrajectory.piecewiseActionCost[i];

            // add high penalty if survivability is lower than the specified threshold
            if (robotTrajectory.piecewiseSurvivability[i] < params_.minAcceptableSurvivability) {
                totalCollisionCost += 0.5;
            }
        } else   // normal case
        {
            totalProgress +=
              timeDecayFactor * robotTrajectory.piecewiseSurvivability[i] * robotTrajectory.piecewiseRawProgress[i];
            totalCollisionCost += timeDecayFactor
              * robotTrajectory.piecewiseCollisionCost[i];   // already in expected cost form (1-p_s)*collision
            totalActionCost += timeDecayFactor * robotTrajectory.piecewiseActionCost[i];

            // add high penalty if survivability is zero
            if (robotTrajectory.piecewiseSurvivability[i] <= 0.01) {
                totalCollisionCost += 0.5;
            }
        }
    }
    totalProgress += robotTrajectory.heuristicCost;   // add heuristic cost to progress term

    //     if(params_.useChanceConstraint)
    //     {
    //         if(survivability < params_.minAcceptableSurvivability)
    //         {
    //             totalCollisionCost += 100.0;
    //         }
    //     }

    double totalExpectedCost = totalProgress + totalCollisionCost + totalActionCost;

    ////////////////////////////////////////////////////////////////////////////
    // final trajectory info
    robotTrajectory.totalSurvivability = survivability;   // Final survivability at the end pose of the trajectory
    robotTrajectory.expectedProgress = totalProgress;     // total expected progress
    robotTrajectory.expectedCollisionCost = totalCollisionCost;   // total collision cost
    robotTrajectory.expectedActionCost = totalActionCost;         // total action cost
    robotTrajectory.expectedCost = totalExpectedCost;             // total expected cost for this trajectory

    robotTrajectory.pathLength = pathLength;   // Integrated length of the path, not considering collision
    robotTrajectory.pathCost = pathCost;       // Integrated cost of travel along the path, not considering collision

#ifdef DEBUG_TRAJECTORY_EVALUATOR
    std::cout << "DEBUG: TrajectoryEvaluator: ExpectedProgress: " << totalProgress
              << ", ExpectedActionCost: " << totalActionCost << ", ExpectedCollisionCost: " << totalCollisionCost
              << '\n';
#endif

    return totalExpectedCost;
}


double TrajectoryEvaluator::piecewiseCollisionProbability(const pose_t& endPose,
                                                          const velocity_t& velocities,
                                                          float timestep,
                                                          double* robotUncertaintySigma,
                                                          double* distance)
{
    // distance to the object
    *distance = static_cast<double>(robot_->closestObstacle(*map_));

    // set preferred distance based on current velocity
    const double kClearancePerMeter = 0.1;
    const double kClearancePerRad = 0.1;
    double preferredClearance = params_.minStaticPassingRadius
      + fmax(fabs(velocities.linear) - 0.2, 0.0) * kClearancePerMeter
      + fmax(fabs(velocities.angular) - 0.3, 0.0) * kClearancePerRad;

    // underestimate distance to observe absolute clearance
    *distance = std::max(0.0, *distance - preferredClearance);

    // speed-based exapnsion of uncertainty
    double motionBasedUncertaintyStd =
      sqrt((params_.linearVelocityUncertaintyStdWeight * params_.linearVelocityUncertaintyStdWeight
            * static_cast<double>(velocities.linear * velocities.linear))
           + (params_.angularVelocityUncertaintyStdWeight * params_.angularVelocityUncertaintyStdWeight
              * static_cast<double>(velocities.angular * velocities.angular)));

    *robotUncertaintySigma += motionBasedUncertaintyStd * timestep;

    // probability of collision to static objects
    const double kSigmaCap = 0.06;
    double adaptiveUncertaintyStd =
      (*robotUncertaintySigma > kSigmaCap) ? kSigmaCap : *robotUncertaintySigma;   // capping
    double adaptiveCollisionProbability =
      exp(-(*distance / adaptiveUncertaintyStd) * (*distance / adaptiveUncertaintyStd));

    return adaptiveCollisionProbability;
}


double TrajectoryEvaluator::piecewiseCollisionProbability(const pose_t& endPose,
                                                          const velocity_t& velocities,
                                                          double robotUncertaintySigma,
                                                          const dynamic_object_state_t& objectState,
                                                          float objectRadius,
                                                          float timestep,
                                                          double* objectUncertaintySigma,
                                                          double* distance)
{
    // ignore objects that are too far from robot center, to avoid more expensive distance-from-boundary calculations.
    const double kObjectIsTooFarThreshold = 4.0;
    if (fabs(objectState.x - endPose.x) > kObjectIsTooFarThreshold
        || fabs(objectState.y - endPose.y) > kObjectIsTooFarThreshold) {
        return 0.0;   // we don't care about things that are too far away from robot center.
    }

    // estimate object pose from position and velocity
    float objectSpeed = sqrt(objectState.xVel * objectState.xVel + objectState.yVel * objectState.yVel);
    float objectHeading = (objectSpeed > 0.2) ? atan2(objectState.yVel, objectState.xVel) : endPose.theta;
    pose_t objectPose = pose_t(objectState.x, objectState.y, objectHeading);

    // distance to the object
    *distance = robot_->distanceToObject(objectPose, objectRadius);

    // set preferred distance based on current velocity
    const double kClearancePerMeter = 0.1;
    const double kClearancePerRad = 0.0;
    double preferredClearance = params_.minDynamicPassingRadius
      + fmax(fabs(velocities.linear) - 0.2, 0.0) * kClearancePerMeter
      + fmax(fabs(velocities.angular) - 0.3, 0.0) * kClearancePerRad;

    // underestimate distance to observe absolute clearance
    *distance = std::max(0.0, *distance - preferredClearance);

    // expand object uncertainty sigma along the axis of object motion
    double motionBasedUncertaintyStd = 4.0 * params_.linearVelocityUncertaintyStdWeight * fabs(objectSpeed);
    *objectUncertaintySigma += motionBasedUncertaintyStd * timestep;

    // capping
    const double kSigmaCap = 1.0;
    double longAxisUncertaintyStd = (*objectUncertaintySigma > kSigmaCap) ? kSigmaCap : *objectUncertaintySigma;
    double shortAxisUncertaintyStd = params_.dynamicUncertaintyStdMin;

    // half-elliptical approximation (uncertainty bubble is an ellipse if the object is in front of the robot; otherwise
    // it is a circle)
    double adaptiveUncertaintyStd;

    pose_t objectPoseInRobotFrame = objectPose.transformToNewFrame(endPose);
    if (objectPoseInRobotFrame.x < 0 && velocities.linear > 0.1) {
        // use circle approximation if the object is behind the robot and the robot is already moving
        adaptiveUncertaintyStd = shortAxisUncertaintyStd;
    } else {
        // use elliptical approximation otherwise
        pose_t robotPoseInObjectFrame =
          endPose.transformToNewFrame(objectPose);   // positive x-axis lies along the direction of object linear speed
        float lineOfSightAngleInObjectFrame =
          atan2(robotPoseInObjectFrame.y,
                robotPoseInObjectFrame.x);   // is zero or +-M_PI when the robot is on the direction of object motion

        double x = longAxisUncertaintyStd * cos(lineOfSightAngleInObjectFrame);
        double y = shortAxisUncertaintyStd * sin(lineOfSightAngleInObjectFrame);
        adaptiveUncertaintyStd = sqrt(x * x + y * y);
    }

    // total unceratinty std for the long axis of the uncertainty ellipse
    adaptiveUncertaintyStd += robotUncertaintySigma;

    double adaptiveCollisionProbability =
      exp(-(*distance / adaptiveUncertaintyStd) * (*distance / adaptiveUncertaintyStd));

    return adaptiveCollisionProbability;
}


double TrajectoryEvaluator::piecewiseActionCost(const velocity_t& velocity,
                                                const acceleration_t& acceleration,
                                                float timestep)
{
    // action cost for a trajectory segment
    double actionCost = ((params_.linearVelocityActionWeight * velocity.linear * velocity.linear)
                         + (params_.angularVelocityActionWeight * velocity.angular * velocity.angular)
                         + (params_.linearAccelerationActionWeight * acceleration.linear * acceleration.linear)
                         + (params_.angularAccelerationActionWeight * acceleration.angular * acceleration.angular))
      * timestep;

    if (velocity.linear < 0) {
        actionCost *= params_.negativeVelocityActionCostMultiplier;   // harshly penalize backing up motion!
    }

    return actionCost;
}


double TrajectoryEvaluator::piecewiseCollisionCost(const motion_state_t& endState, float timestep)
{
    // cost of collision for a trajectory segment.
    double totalSpeed = static_cast<double>(fabs(endState.velocity.linear) + fabs(endState.velocity.angular));
    double collisionCost =
      (params_.baseCostOfCollision + params_.velocityWeightOfCollision * totalSpeed) * static_cast<double>(timestep);

    // adding collision area
    if (params_.shouldConsiderCollisionArea) {
        collisionCost += params_.collisionAreaWeight * robot_->collisionArea(*map_) * timestep;
    }

    return collisionCost;
}


double TrajectoryEvaluator::socialDistanceCost(const pose_t& endPose,
                                               const dynamic_object_state_t& objectState,
                                               float objectRadius,
                                               float timestep)
{
    double proximityCost = 0.0;

    // NOTE: temporary turn-off

    //     double kSocialDistanceThresholdX = 0.6; // the distance in x (forward) where the cost becomes zero
    //     double kSocialDistanceThresholdY = 0.3; // the distance in y (lateral) where the cost becomes zero
    //     double kMaxSocialDistanceCost    = 2.0; // the maximum value of the cost
    //
    //     // pose (and speed) of the object
    //     pose_t objectPose = pose_t(objectState.x, objectState.y, atan2(objectState.yVel, objectState.xVel));
    //
    //     // distance to the object
    //     double distance = robot_->distanceToObject(objectPose, objectRadius);
    //
    //     // relative orientation in robot frame
    //     double relativeOrientation = atan2(objectState.y - endPose.y, objectState.x - endPose.x);
    //     relativeOrientation        = wrap_to_pi(relativeOrientation - endPose.theta);
    //
    //     // modified object position in robot frame. This does not exactly correspond to relative position due to
    //     distanceToObject function. double relativePositionXModified = distance*cos(relativeOrientation); // positive
    //     x is pointing to robot forward double relativePositionYModified = distance*sin(relativeOrientation);

    //     if(fabs(relativePositionYModified) < kSocialDistanceThresholdY)
    //     {
    //         proximityCost = std::max((kSocialDistanceThresholdX - relativePositionXModified), 0.0) /
    //         kSocialDistanceThresholdX * kMaxSocialDistanceCost * timestep;
    //     }

    return proximityCost;
}


void TrajectoryEvaluator::integratePathLength(double& pathLength, float linearVelocity, float timestep)
{
    // add length of the trajectory segment to the path length.
    pathLength += fabs(linearVelocity) * timestep;
}


void TrajectoryEvaluator::integratePathCost(double& pathCost, const velocity_t& velocity, float timestep)
{
    // add distance-like travel cost of the trajectory segment to the path cost.
    pathCost += sqrt((velocity.linear * velocity.linear)
                     + params_.angularTravelWeightForPathCost * (velocity.angular * velocity.angular))
      * timestep;
}


}   // namespace mpepc
}   // namespace vulcan

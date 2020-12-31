/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     kinematic_control_law.cpp
* \author   Jong Jin Park
*
* Definition of KinematicControlLaw.
*/

#include "mpepc/control/kinematic_control_law.h"
#include "core/pose.h"
#include "core/angle_functions.h"

// #define DEBUG_KINEMATIC_CONTROL_LAW

namespace vulcan
{

namespace mpepc
{

const float kControlConvergeceThresholodReductionFactor = 0.8f;
const float kTurnInPlacePseudoKappaValue = 1.0f;
// mathematically when turning in place kappa (curvature) is infinite.
// but keeping it at some modest value is more useful in practice.

KinematicControlLaw::KinematicControlLaw(const kinematic_control_law_params_t& params)
: previousAngularVelocityCommand_(0.0f)
, convergenceStartTimeUs_(0)
, params_(params)
{
}


control_law_output_t KinematicControlLaw::computeOutput(const   pose_t&   robotPose,
                                                        const   motion_target_t& motionTarget,
                                                        int64_t currentTimeUs)
{
    control_law_output_t output;

    params_.k1 = motionTarget.k1;

    if(currentTimeUs <=0)
    {
        output.haveReachedTarget = false;
        resetInternalStates();
    }
    else
    {
        output.haveReachedTarget = haveReachedTarget(robotPose, motionTarget.pose, currentTimeUs);
    }

    control_law_coordinates_t coords(robotPose, motionTarget.pose, motionTarget.direction);
    output.coords = coords;

    // HACK: made the converged_radius for control smaller than the one used for determining the convergence (haveReachedTarget()).
    // This will ensure the robot always go inside the convergence bound, thereby help removing twitching at the threshold due to noise, etc.
    float angleError = angle_diff(motionTarget.pose.theta, robotPose.theta);
    bool  controlRadiusIsConverged = fabs(coords.r)   < kControlConvergeceThresholodReductionFactor*params_.convergenceRadius;
    bool  controlAngleIsConverged  = fabs(angleError) < kControlConvergeceThresholodReductionFactor*params_.convergenceAngle;

    if(output.haveReachedTarget) // don't do anything if reached target previouly.
    {
        output.referenceHeading = motionTarget.pose.theta;
        output.referenceKappa   = kTurnInPlacePseudoKappaValue;
        output.linearVelocity   = 0.0;
        output.angularVelocity  = 0.0;
    }
    else if(controlRadiusIsConverged && controlAngleIsConverged) // don't do anything if reached target now.
    {
        output.referenceHeading = motionTarget.pose.theta;
        output.referenceKappa   = kTurnInPlacePseudoKappaValue;
        output.linearVelocity   = 0.0;
        output.angularVelocity  = 0.0;
    }
    else if(controlRadiusIsConverged) // simple rotation when radius is small enough but angle is large
    {
        output.referenceHeading = motionTarget.pose.theta;
        output.referenceKappa   = kTurnInPlacePseudoKappaValue;
        output.linearVelocity   = 0.0;

        float turnInPlaceVelocity = fmin(params_.angularVelocityTurnInPlace, 0.7*motionTarget.velocityGain);

        if(fabs(angleError) > 2.0) // A heuristic to reduce chattering when trying to make a very small s-curve to an anti-aligned target pose (larger than 2.0 rad error)
        {
            output.angularVelocity = copysign(turnInPlaceVelocity, previousAngularVelocityCommand_); // make the robot turn in the same direction as before.
        }
        else
        {
            output.angularVelocity = copysign(turnInPlaceVelocity, angleError);
        }
    }
    else // normal update
    {
        double kKappaThresh = 4.0;
//         double kSmallRadius = 0.15;
//         double kLargeRadius = 2.0;
//         double kMinLinearVelocity = 0.15;
//         double kMaxLinearVelocity  = motionTarget.velocityGain;

        double minimumForwardSpeed = 0.15;

        double referenceDelta  = atan(-params_.k1 * coords.theta); // robot reference heading in controller coordinates.

        // determination of the control based on error kinematics
        double propotionalControlTerm = params_.k2 * angle_diff(coords.delta, referenceDelta); // I usually prefer (reference - current) for the error, but well this is how it was published.
        double feedforwardControlTerm = sin(coords.delta) * (1.0 + (params_.k1 / (1.0 + pow(params_.k1*coords.theta, 2.0))));

        double radiusTimesKappa = -(propotionalControlTerm + feedforwardControlTerm);
        double referenceKappa   =  radiusTimesKappa / fabs(coords.r);

        double lambda = params_.lambda;
        double beta   = params_.beta;

        if(fabs(referenceKappa) > kKappaThresh)
        {
            lambda = 1.0;
            beta   = beta * pow(kKappaThresh, (params_.lambda - lambda));
        }

        double kappaVelocity          = motionTarget.velocityGain / (1.0 + (beta * pow(fabs(referenceKappa), lambda)));
        double slowdownLinearVelocity = motionTarget.velocityGain * fabs(coords.r) / params_.slowdownRadius + minimumForwardSpeed;

//         double slowdownLinearVelocity = kMinLinearVelocity + kMaxLinearVelocity * fabs(kMaxLinearVelocity - kMinLinearVelocity) *  dfabs(coords.r - fabs(kSmallRadius)) / fabs(kLargeRadius - kSmallRadius);

        bool shouldUseSlowdownVelocity = fabs(coords.r) < params_.slowdownRadius && fabs(slowdownLinearVelocity) < fabs(kappaVelocity); // slow down when the robot is near the target but has not converged yet

        // compute outputs
        output.referenceHeading = wrap_to_pi(referenceDelta - coords.theta + motionTarget.pose.theta); // reference heading in the global reference frame.
        output.referenceKappa   = radiusTimesKappa / fabs(coords.r);                             // desired signed path curvature.
        output.linearVelocity   = shouldUseSlowdownVelocity ? slowdownLinearVelocity : kappaVelocity;
        output.angularVelocity  = output.referenceKappa * output.linearVelocity;

        if(coords.r < 0.0) // Flip reference heading and linear velocity if r < 0, since desired motion is backward and the control law coordinates were computed from flipped robot and target pose.
        {
            output.linearVelocity *= -1;
            output.referenceHeading = angle_sum(output.referenceHeading, M_PI);
        }
    }

//     else if(fabs(coords.r) < params_.slowdownRadius) // slow down when the robot is near the target but has not converged yet
//     {
//         output = normalUpdate(coords, motionTarget.velocityGain, motionTarget.pose.theta);
//
//         float slowdownLinearVelocity = coords.r * motionTarget.velocityGain / params_.slowdownRadius; // this is negative if coords.r is negative
//         if(fabs(slowdownLinearVelocity) < fabs(output.linearVelocity)) // take the slower of the normal velocity or the slowdown velocity.
//         {
//             output.linearVelocity  = slowdownLinearVelocity;
//             output.angularVelocity = output.referenceKappa * output.linearVelocity;
//         }
//     }
//     else // normal update
//     {
//         output = normalUpdate(coords, motionTarget.velocityGain, motionTarget.pose.theta);
//     }

    previousAngularVelocityCommand_ = output.angularVelocity; // update command history

    // send out warning if the output exceeds the specified maximum. (Just warnings for now)
    if((fabs(output.linearVelocity) > params_.maxLinearVelocity) || (fabs(output.angularVelocity) > params_.maxAngularVelocity))
    {
        std::cout<<"WARNING!!: Velocity command output ("<<output.linearVelocity<<','<<output.angularVelocity
                 <<") is larger than the specified bounds ("<<params_.maxLinearVelocity<<','<<params_.maxAngularVelocity<<")!!\n";
    }

    return output;
}


control_law_output_t KinematicControlLaw::computeOutput(const   pose_t&   robotPose,
                                                        const   motion_target_t& motionTarget,
                                                        int64_t currentTimeUs,
                                                        float   previousAngularVelocityCommand)
{
    this->previousAngularVelocityCommand_ = previousAngularVelocityCommand;

    return computeOutput(robotPose, motionTarget, currentTimeUs);
}


control_law_output_t KinematicControlLaw::normalUpdate(const control_law_coordinates_t& coords, float velocityGain, float targetTheta)
{
    control_law_output_t output;

    float referenceDelta  = atan(-params_.k1 * coords.theta); // robot reference heading in controller coordinates.

    // determination of the control based on error kinematics
    float propotionalControlTerm = params_.k2 * angle_diff(coords.delta, referenceDelta); // I usually prefer (reference - current) for the error, but well this is how it was published.
    float feedforwardControlTerm = sin(coords.delta) * (1.0f + (params_.k1 / (1.0f + pow(params_.k1*coords.theta, 2))));

    float radiusTimesKappa = -(propotionalControlTerm + feedforwardControlTerm);

    // compute outputs
    output.referenceHeading = wrap_to_pi(referenceDelta - coords.theta + targetTheta); // reference heading in the global reference frame.
    output.referenceKappa   = radiusTimesKappa / fabs(coords.r);                             // desired signed path curvature.
    output.linearVelocity   = copysign(velocityGain / (1.0f + (params_.beta * pow(fabs(output.referenceKappa), params_.lambda))), coords.r); // linear velocity command to issue
    output.angularVelocity  = output.referenceKappa * output.linearVelocity;                                                               // angular velocity command to issue

    if(coords.r < 0) // Flip reference heading and linear velocity if r < 0, since desired motion is backward and the control law coordinates were computed from flipped robot and target pose.
    {
        output.referenceHeading = angle_sum(output.referenceHeading, M_PI);
    }

    return output;
}


bool KinematicControlLaw::haveReachedTarget(const pose_t& robotPose, const pose_t& targetPose, int64_t currentTimeUs)
{
    // Reaching the target means both the distance to the target and the orientation offset have become small numbers.
    float distance  = distance_between_points(robotPose.toPoint(), targetPose.toPoint());
    float angleDiff = fabs(angle_diff(targetPose.theta, robotPose.theta));

    bool isRadiusConverged = distance  < params_.convergenceRadius;
    bool isAngleConverged  = angleDiff < params_.convergenceAngle;
    bool isPoseConverged   = isRadiusConverged && isAngleConverged;

    // If the robot is in the convergence zone, then start the convergence timer. This allows the robot to push further inward
    // after reaching threshold. It is better to overshoot a bit rather than stopping at threshold and get confused.
    // convergenceStartTimeUs_ resets to -1 whenever the pose is not deemed to be within the convergence threshold.
    if(!isPoseConverged)
    {
        // if pose has not converged simply reset to initial condition, indicating for the next step
        // that the robot has not converged previously
        convergenceStartTimeUs_ = -1;
    }
    else if((convergenceStartTimeUs_ == -1) || (currentTimeUs < convergenceStartTimeUs_))
    {
        // if the robot is sufficiently close to the target pose, and if
        // (1) the robot was not close to the target in the previous run, OR
        // (2) it looks like time has been revesed due to simulation (during optimization, etc.),
        // mark the current time as the starting time for the convergence.
        convergenceStartTimeUs_ = currentTimeUs;
    }

    bool hasConvergedForTime = (convergenceStartTimeUs_ == -1) ? false : (currentTimeUs - convergenceStartTimeUs_) > params_.convergenceTimeUs;

#ifdef DEBUG_KINEMATIC_CONTROL_LAW
    if(isPoseConverged && !hasConvergedForTime)
    {
        std::cout<<"DEBUG KINEMATIC CONTROL LAW: Pose "<<targetPose<<" converged to target "<<robotPose<<'\n';
    }
    if(hasConvergedForTime)
    {
        std::cout<<"DEBUG KINEMATIC CONTROL LAW: Pose "<<targetPose<<" converged to target "<<robotPose<<" for "<<(currentTimeUs - convergenceStartTimeUs_)/1000<<" ms.\n";
    }
#endif

    return hasConvergedForTime;
}

} // namespace mpepc
} // namespace vulcan

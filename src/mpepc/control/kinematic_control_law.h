/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file   kinematic_control_law.h
* \author Jong Jin Park
*
* Declaration of KinematicControlLaw.
*/
#ifndef MPEPC_KINEMATIC_CONTROL_LAW_H
#define MPEPC_KINEMATIC_CONTROL_LAW_H

#include "mpepc/control/control_law_coordinates.h"
#include "mpepc/control/params.h"

/*
* More on the controller implementation:
* 
* The current implementation of the MotionController is composed of two major submodules:
* (1) a pose-stabilizing kinematic controller for computing reference heading and/or
*     reference velocities that gurantees global stabilization to a target pose with
*     guranteed smoothness.
* (2) a velocity-tracking joystick controller which stabilizes the robot quickly to the reference velocities.
* 
* NOTE: Theoretically, this type of control approach is known as time-scale decomposition.
*       The idea is to find a virtual control which stabilizes some low-dimensional part
*       of the system, which defines a slow manifold where the remaining part of the system
*       will quickly converge to. The key step here is to ensure the faster subsystem is
*       indeed faster than the slow subsystem so that the overall system do converge to a
*       small neighborhood of origin.
* NOTE: The Pose-stabilizing controller used here (Park and Kuipers, ICRA-11) is already
*       time-scale decomposed, where the slow subsystem is defined in the position-space
*       to compute the reference heading and the fast subsystem is defined in the steering
*       (orientation) so that the actual vehicle quickly converges to the reference heading.
*       The implementation of the controller also converts the reference heading to reference
*       velocities via comfort constraints (freely) imposed by the user.
* NOTE: In the real vehicle, we have additional dynamic feedback controller for joystick,
*       which needs to stabilize velocities to the reference quickly.
*/


namespace vulcan
{
    
namespace mpepc
{

class KinematicControlLaw
{   
public:
    
    /**
    * Constructor for the KinematicControlLaw
    */
    KinematicControlLaw(const kinematic_control_law_params_t& params);
    
    // NOTE: The base kinematic control law (Park and Kuipers, ICRA-11) is a function of poses, but this implementation,
    //       KinematicControlLaw, requires current time information and also previous angular velocity at initialization.
    //       This is to deal with the sensor uncertainties and to better cope with the singularity
    //       introduced by the hard thresholding near the target.
    /**
    * computeOutput applies the kinematic control law (Park and Kuipers, ICRA-11) to the current pose and time of the robot givan a motion target.
    * \param    robotPose                      Cuurent pose of the robot.
    * \param    motionTarget                   The motion target (pose, direction and velocity gain) to converge to.
    * \param    currentTimeUs                  Current time (in microseconds). If <=0, this will reset internal states that it uses to check convergence.
    * \param    previousAngularVelocityCommand If specified, this will override the previous angular velocity command remembered by the class.
    * \return   Desired velocities, heading and curvature that guarantees convergence to the motion target.
    */
    control_law_output_t computeOutput(const   pose_t&   robotPose,
                                       const   motion_target_t& motionTarget,
                                       int64_t currentTimeUs = 0);
    control_law_output_t computeOutput(const   pose_t&   robotPose,
                                       const   motion_target_t& motionTarget,
                                       int64_t currentTimeUs,
                                       float   previousAngularVelocityCommand);
    
    
    /**
    * haveReachedTarget chekcs if the robot has reached a target pose by checking if the robot has been within a sufficiently small neighborhood of the target
    * for sufficient amount of time.
    * \param    robotPose       Current pose of the robot.
    * \param    targetPose      Target pose of interest.
    * \param    currentTimeUs   Current time (in microseconds).
    * \return   Indicator for reaching the target
    */
    bool haveReachedTarget(const pose_t& robotPose, const pose_t& targetPose, int64_t currentTimeUs);
    
    /**
    * setParams sets the paramters for the kinematic control law.
    * \param    newParams       New parameters to use.
    */
    void setParams(const kinematic_control_law_params_t& newParams) { this->params_ = newParams; };
    
    /**
    * resetInternalStates initializes previous angular velocity command and convergence start time, which is needed in simulation routines.
    * \param    previousAngularVelocityCommand  Previous angular velocity command.
    * \param    convergenceStartTimeUs          Convergence start time.
    */
    void resetInternalStates(float previousAngularVelocityCommand = 0.0f, int64_t convergenceStartTimeUs = -1)
    {
        this->previousAngularVelocityCommand_ = previousAngularVelocityCommand;
        this->convergenceStartTimeUs_         = convergenceStartTimeUs;        
    }; // need to be reset at initalization phase, e.g. every time a simulator starts

    /// accessors
    float   getPreviousAngularVelocityCommand(void) { return previousAngularVelocityCommand_; };
    int64_t getConvergenceStartTimeUs(void) { return convergenceStartTimeUs_; };
    
private:
    
    /**
    * in normalUpdate, the following (abbreviated) equations extends the graceful controller from ICRA-11 to admit negative velocity for back up motion.
    * 
    * kappa = 1/abs(r)*(propotional_term + feedforward_term);
    * linearVelocity  = copysign(v_max/(1+abs(kappa)^beta), r);
    * angularVelocity = kappa*linearVelocity;
    * 
    * referenceHeading = referenceDelta - coords.theta + target.theta ( + M_PI if r < 0).
    * 
    * The velocity computed by normalUpdate is automatically bounded in velocities. It is also bounded and smooth in accelerations and jerks if the target is stationary.
    */
    control_law_output_t normalUpdate(const control_law_coordinates_t& coords, float velocityGain, float targetTheta);

    // smallnote: It makes sense to keep previous velocity command and convergence history as an internal state. Those are the stuff produced by this class. Other variables like pose and time are not.
    float   previousAngularVelocityCommand_;
    int64_t convergenceStartTimeUs_;
    
    kinematic_control_law_params_t params_;
};

} // mpepc
} // vulcan

#endif // MPEPC_KINEMATIC_CONTROL_LAW_H
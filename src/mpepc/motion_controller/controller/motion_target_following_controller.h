/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     motion_target_following_controller.h
* \author   Jong Jin Park
*
* Declaration of MotionTargetFollowingController.
*/

#ifndef MPEPC_MOTION_TARGET_FOLLOWING_CONTROLLER_H
#define MPEPC_MOTION_TARGET_FOLLOWING_CONTROLLER_H

#include <mpepc/motion_controller/controller/motion_controller.h>
#include <mpepc/motion_controller/task/target.h>
#include <mpepc/motion_controller/params.h>
#include <mpepc/control/kinematic_control_law.h>
#include <mpepc/control/joystick_control_law.h>
#include <robot/commands.h>

namespace vulcan
{

namespace mpepc
{

const std::string MOTION_TARGET_FOLLOWING_CONTROLLER_TYPE("motion_target_following");

/**
* MotionTargetFollowingController is the new implementation of the GracefulMotionController, with time-dependent convergence,
* ability to move backward, and the feedforward and feedback control structure. TODO: update the description.
*/
class MotionTargetFollowingController : public MotionController
{
    
public:
    
    /**
    * Constructor for MotionTargetFollowingController.
    *
    * \param    params          Parameters for controlling the behavior of the controller
    */
    MotionTargetFollowingController(const motion_target_following_controller_params_t& params);

    /**
    * Destructor for MotionTargetFollowingController.
    */
    virtual ~MotionTargetFollowingController(void) { }

    // MotionController interface
    virtual bool                    canHandleTask (const std::shared_ptr<MotionControllerTask>& task); /* override */
    virtual void                    assignTask    (const std::shared_ptr<MotionControllerTask>& task); /* override */
    virtual bool                    isTaskComplete(const motion_controller_data_t& data); /* override */
    virtual robot::motion_command_t updateCommand (const motion_controller_data_t& data); /* override */
    virtual void                    pauseCommand  (int64_t timeUs); /* override */
    virtual void                    resumeCommand (void); /* override */
    
private:
    
    KinematicControlLaw  kinematicControlLaw_;
    JoystickControlLaw   joystickControlLaw_;
    
    bool            haveTarget_;
    motion_target_t target_;
    int64_t         targetTimestamp_;
    int64_t         targetTimeout_;
    
    bool    isPaused_;
    int64_t pauseStartTimeUs_;
    int64_t pauseEndTimeUs_;

    motion_target_following_controller_params_t params_;
};
    

} // mpepc
} // vulcan

#endif // MPEPC_MOTION_TARGET_FOLLOWING_CONTROLLER_H
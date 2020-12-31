/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     wheelchair.h
* \author   Collin Johnson and Jong Jin Park
*
* Declaration of Wheelchair interface and create_wheelchair factory.
*/

#ifndef ROBOT_WHEELCHAIR_H
#define ROBOT_WHEELCHAIR_H

#include <boost/shared_ptr.hpp>
#include "utils/thread.h"
#include "robot/commands.h"

namespace vulcan
{
namespace robot
{

struct wheelchair_params_t;

/**
* wheelchair_joystick_calibration_t defines the calibration parameters for converting the wheelchair
* joystick commands to linear and angular velocity.
*/
struct wheelchair_joystick_calibration_t
{
    double positiveLinearVelocityRatio;
    double negativeLinearVelocityRatio;

    double translatingPositiveAngularVelocityRatio;
    double translatingNegativeAngularVelocityRatio;

    double turnInPlacePositiveAngularVelocityRatio;
    double turnInPlaceNegativeAngularVelocityRatio;
};


/**
* Wheelchair is the interface used for controlling a wheelchair. The wheelchair is controlled by providing
* a motor command that contains a target linear and angular velocity or joystick position to command.
* The interface works using four methods:
*
*   void setMotorCommandTarget(const motor_command_t& command)
*       - The target command for the wheelchair.
*
*   commanded_velocity_t updateCommandVelocity(void)
*       - Return the currently commanded velocities
*
*   motion_command_t getWheelchairMotionCommand(void)
*       - grabs the command the wheelchair joystick is currently issuing. Returned motion command contains
*         the joystick command and associated metric velocity commands
*
*   velocity_command_t getWheelchairVelocityCommand(void)
*       - grabs the command the wheelchair joystick is currently issuing. The command is a joystick
*         command converted to metric velocity
*
*   joystick_command_t getJoystickCommand(void)
*       - grabs the most recent joystick command from the wheelchair driver
*
* 
* NOTE: All commands being issued to the Wheelchair must have a current and valid timestamp. Commands with an
* invalid timestamp will be ignored as being stale. A current command is a command issued within 500000us of
* the time returned by utils::system_time_us().
*/
class Wheelchair : public utils::Threadable
{
public:

    /**
    * Constructor for Wheelchair.
    */
    Wheelchair(void);

    virtual ~Wheelchair(void) { }

    /**
    * setMotorCommandTarget sets the target joystick position for the wheelchair to attain.
    */
    void setMotorCommandTarget(const motion_command_t& command);

    /**
    * updateCommandedVelocities changes the commanded velocities for the wheelchair, moving them toward the
    * target velocities.
    *
    * \return   Velocities that are currently being sent to the wheelchair.
    */
    commanded_velocity_t updateCommandedVelocities(void);

    /**
    * updateCommandedJoystick changes the commanded joystick for the wheelchair.
    *
    * \return   Joystick positoins that are currently being sent to the wheelchair.
    */
    commanded_joystick_t updateCommandedJoystick(void);

    /**
    * getWheelchairMotionCommand retrieves the metric velocity and joystick command associated with the current joystick position.
    * A non-zero velocity indicates a manual command is currently being issued to the wheelchair.
    *
    * \return   Current velocity command from the wheelchair joystick.
    */
    motion_command_t getWheelchairMotionCommand(void) const;

    /**
    * getWheelchairVelocityCommand retrieves the metric velocity of the current joystick position. A non-zero velocity
    * indicates a manual command is currently being issued to the wheelchair.
    *
    * \return   Current motion command from the wheelchair joystick.
    */
    velocity_command_t getWheelchairVelocityCommand(void) const;

    /**
    * getJoystickCommand retrieves the most recent joystick command delivered to the wheelchair. If the joystick
    * hasn't moved, the position is (0, 0), but the timestamp will change to indicate that the no command has
    * been received within the latest timeframe.
    *
    * \return   Most recent joystick command, or lack of a command, read by the wheelchair.
    */
    joystick_command_t getJoystickCommand(void) const;

    /**
    * setJoystickCommand issues a low-level joystick command to be issued to the Quantum6000.
    */
    void setJoystickCommand(const robot::joystick_command_t& command);

protected:

    /**
    * convertJoystickCommandToCommandedVelocity converts the joystick command issued to the wheelchair into
    * linear and angular velocity to inform the other processes running on the robot of the current motion.
    *
    * \param    joystick        Joystick command issued to the controller
    * \return   Translation of the joystick command in a commanded_velocity_t.
    */
    virtual commanded_velocity_t convertJoystickCommandToCommandedVelocity(const joystick_command_t& joystick) const = 0;

    /**
    * convertVelocityCommandToJoystickCommand should convert linear and angular velocity command into the (x,y,gain)
    * that will be ultimately transmitted to the wheelchair controller.
    *
    * \param    command         Velocity command to be translated
    * \return   Joystick command to be issued to the wheelchair controller
    */
    virtual joystick_command_t convertVelocityCommandToJoystickCommand(const velocity_command_t& command) const = 0;

    /**
    * isJoystickCentered should check to see if the provided joystick command is centered according to the design of
    * the wheelchair's particular joystick controller.
    *
    * \param    joystick            Command to check for centering
    */
    virtual bool isJoystickCentered(const joystick_command_t& joystick) const = 0;

    /**
    * waitForMessages has the wheelchair driver sit and wait until a controller or joystick message arrives.
    */
    virtual void waitForMessages(void) = 0;

    /**
    * processMessages process messages has the wheelchair drive handle all active messages.
    *
    * \param    canUseRobotCommand
    */
    virtual void processMessages(bool canUseRobotCommand) = 0;

    joystick_command_t humanJoystickCommand;
    joystick_command_t robotJoystickCommand;

private:

    // utils::Threadable interface
    virtual int run(void);

    bool isRobotControlAllowed(void) const;

    motion_command_t     commandRecieved;
    bool                 humanIsDriver;
    mutable utils::Mutex commandLock;
};

/**
* create_wheelchair is a factory function for creating instances of Wheelchair.
*
* NOTE: If the type is invalid, the function will fail hard with an assert.
*
* \param    params          Parameters for the wheelchair being created
* \return   Instance of Wheelchair.
*/
std::unique_ptr<Wheelchair> create_wheelchair(const wheelchair_params_t& params);

}
}

#endif // ROBOT_WHEELCHAIR_H

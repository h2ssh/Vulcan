/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     invacare_atm.h
* \author   Phil McKenna, Collin Johnson
* Additional changes by R Gaynier - I've attempted to make this file match the Quantum_6000.h file as closely as possible to 
* aid my own understanding and analysis. I've noted those changes where they occur.
*
* Declaration of InvacareATM class for interfacing with the Invacare wheelchair.
*/

#ifndef ROBOT_INVACARE_ATM_H
#define ROBOT_INVACARE_ATM_H

#include "robot/wheelchair.h"
#include <sys/select.h>
#include "utils/serial.h"
#include "utils/mutex.h"

namespace vulcan
{
namespace robot
{

const std::string INVACARE_ATM_TYPE("invacare");

class InvacareATM : public Wheelchair
{
public:

    /**
    * Additional Constructor for InvacareATM, following the example of quantum_6000.h, which may not be necessary. 
    * It is minus the 'calibration' parameter. R Gaynier 12/04/12 
    */
    InvacareATM(const std::string& joystickPort, const std::string& controllerPort);

    /**
    * Constructor for InvacareATM.
    *
    * \param    calibration         Calibration values for converting between joystick and metric units
    * \param    joystickPort        Port to which the joystick is connected
    * \param    controllerPort      Port to which the controller is connected
    */
    InvacareATM(const wheelchair_joystick_calibration_t& calibration, const std::string& joystickPort, const std::string& controllerPort);

    /**
    * Destructor for InvacareATM.
    */
    virtual ~InvacareATM(void);

private:

    // Wheelchair interface
    virtual commanded_velocity_t convertJoystickCommandToCommandedVelocity(const joystick_command_t& joystick) const;
    virtual joystick_command_t   convertVelocityCommandToJoystickCommand  (const velocity_command_t& command) const;
    virtual bool isJoystickCentered(const joystick_command_t& joystick) const;
    virtual void waitForMessages(void);
    virtual void processMessages(bool canUseRobotCommand);

    void handleJoystickMessage(bool canUseRobotCommand);
    void handleControllerMessage(void);

    utils::SerialConnection joystick; 
    utils::SerialConnection controller;

    wheelchair_joystick_calibration_t calibration;

    fd_set readSet;

    utils::Mutex messageLock;

    void convertJoystickCommandToJoystickMessage(const joystick_command_t& joystick);

    unsigned char controlMessage[10];
    unsigned char responseToJoystick[6];
    unsigned char handshakeResponse[4];
    unsigned char joystickMessage[10];
    unsigned char controllerMessage[6];

    short gotResponse;
    bool controlEnabled;

};

}
}

#endif // ROBOT_INVACARE_ATM_H

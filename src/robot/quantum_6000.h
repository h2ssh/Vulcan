/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     quantum_6000.h
* \author   Collin Johnson
*
* Definition of Quantum6000.
*/

#ifndef ROBOT_QUANTUM_6000_H
#define ROBOT_QUANTUM_6000_H

#include <robot/wheelchair.h>
#include <robot/commands.h>
#include <utils/canbus.h>
#include <utils/thread.h>
#include <utils/mutex.h>
#include <map>
#include <string>
#include <sys/select.h>


namespace vulcan
{
namespace utils { class CanBus; }
namespace robot
{

const std::string QUANTUM_6000_TYPE("quantum_6000");

/**
* Quantum6000 is the driver for controlling the Quantum6000 wheelchair. The Quantum6000 has two
* components, a controller and a joystick. These components are connected via a CAN-Bus. Many messages
* are relayed between the two devices, most of them are unimportant for the actual driving of the
* wheelchair and are heartbeats, and that sort of thing.
*
* This Quantum6000 driver works by intercepting the messages sent from the controller and from the
* joystick. Most of the messages are just relayed through the program. The joystick position command,
* though, is replaced by a robot-derived command. Thus, the Quantum6000 is driven by pretending to
* be the joystick.
*
* The Quantum6000 uses two USB-to-CAN adapters to read the controller and joystick messages. The
* ports of these adapters are provided in the constructor. The driver determines which bus is which
* and will begin commanding the wheelchair once it is attached to a thread and executed.
*/
class Quantum6000 : public Wheelchair
{
public:

    /**
    * Constructor for Quantum6000.
    */
    Quantum6000(const std::string& controllerPort, const std::string& joystickPort);

    /**
    * Constructor for Quantum6000.
    */
    Quantum6000(const wheelchair_joystick_calibration_t& calibration, const std::string& controllerPort, const std::string& joystickPort);

    virtual ~Quantum6000(void);

private:

    void init(void);

    // Wheelchair interface
    virtual commanded_velocity_t convertJoystickCommandToCommandedVelocity(const joystick_command_t& joystick) const;
    virtual joystick_command_t   convertVelocityCommandToJoystickCommand(const velocity_command_t& command) const;
    virtual bool isJoystickCentered(const joystick_command_t& joystick) const;
    virtual void waitForMessages(void);
    virtual void processMessages(bool canUseRobotCommand);

    void handleJoystickBus(bool canUseRobotCommand);
    void handleControllerBus(void);

    velocity_command_t   targetVelocities;
    commanded_velocity_t commandedVelocities;
    joystick_command_t   targetJoystickCommand;
    joystick_command_t   internalJoystickCommand;

    bool humanIsDriver;

    std::map<uint8_t, int> joystickMessageCounts;
    std::map<uint8_t, int> controllerMessageCounts;

    int64_t lastDisplayTime;

    utils::CanBus* joystickBus;
    utils::CanBus* controllerBus;

    int64_t firstJoystickMessageTime;
    bool    haveReceivedPositionMessage;

    wheelchair_joystick_calibration_t calibration;

    TPCANMsg   externalJoystickMessage;
    TPCANRdMsg internalJoystickMessage;
    TPCANRdMsg controllerMessage;

    fd_set readSet;

    int joystickFd;
    int controllerFd;

    int64_t previousDataTimestamp;
    int64_t currentDataTimestamp;

    mutable utils::Mutex messageLock;
};

}
}

#endif // ROBOT_QUANTUM_6000_H

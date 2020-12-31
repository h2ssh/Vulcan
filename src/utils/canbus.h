/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     canbus.h
 * \author   Collin Johnson, Philip McKenna
 *
 * Declaration of CanBus.
 */

#ifndef UTILS_CANBUS_H
#define UTILS_CANBUS_H

#include <pcan.h>
#include <string>

namespace vulcan
{
namespace utils
{

/**
 * CanBus is a wrapper around the libpcan library for communication with PEAK Systems series
 * of PC CANBUS adapters.
 */
class CanBus
{
public:
    /**
     * Constructor for CanBus.
     *
     * \param    device          Device to connect to
     * \param    baud            Baudrate for the connection
     */
    CanBus(const std::string& device, int baud);

    /**
     * Destructor for CanBus.
     */
    ~CanBus(void);

    void sendMessage(TPCANMsg& msg);
    void readMessage(TPCANMsg& rdmsg);
    void readMessageTime(TPCANRdMsg& rdmsg);
    void readTimeout(TPCANRdMsg& rdmsg, int usec);
    void sendTimeout(TPCANMsg& msg, int usec);
    void status(void);

    /**
     * getFileHandle retrieves the resource handle for the underlying CanBus to allow
     * for reading from multiple CanBus instances using select().
     */
    int getFileHandle(void) const;

private:
    void openCanBus(int baud);
    void closeCanBus(void);

    std::string device;   // sets which port to open for CAN
    void* busHandle;      // hardware port
};

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_CANBUS_H

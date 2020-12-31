/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     canbus.cpp
* \author   Collin Johnson, Philip McKenna
*
* Definition of CanBus.
*/

#include "utils/canbus.h"
#include <cassert>
#include <iostream>
#include <libpcan.h>
#include <cstdlib>
#include <fcntl.h>    // O_RDWR

namespace vulcan
{
namespace utils
{

void print_error_message(int errorNumber);


CanBus::CanBus(const std::string& dev, int baud)
    : device(dev)
    , busHandle(0)
{
    openCanBus(baud);
}


CanBus::~CanBus(void)
{
    closeCanBus();
}


void CanBus::sendMessage(TPCANMsg& msg)
{
    int error = LINUX_CAN_Write_Timeout(busHandle, &msg, -1);
    print_error_message(error);
}


void CanBus::readMessage(TPCANMsg& rdmsg)
{
    int error = CAN_Read(busHandle, &rdmsg);
    print_error_message(error);
}


void CanBus::readMessageTime(TPCANRdMsg& rdmsg)
{
    int error = LINUX_CAN_Read(busHandle, &rdmsg);
    print_error_message(error);
}


void CanBus::readTimeout(TPCANRdMsg& rdmsg, int usec)
{
    int error = LINUX_CAN_Read_Timeout(busHandle, &rdmsg, usec);
    print_error_message(error);
}


void CanBus::sendTimeout(TPCANMsg& msg, int usec)
{
    int error = LINUX_CAN_Write_Timeout(busHandle, &msg, usec);
    print_error_message(error);
}


void CanBus::status(void)
{
    int status = CAN_Status(busHandle);
    std::cout << "receivetest: pending CAN status 0x" << status << " read.\n";
}


int CanBus::getFileHandle(void) const
{
    return LINUX_CAN_FileHandle(busHandle);
}


void CanBus::openCanBus(int baud)
{
    busHandle = LINUX_CAN_Open(device.c_str(), O_RDWR);

    int error = CAN_Init(busHandle, baud, CAN_INIT_TYPE_EX);
    print_error_message(error);

    assert(busHandle);
}


void CanBus::closeCanBus(void)
{
    int error = CAN_Close(busHandle);
    print_error_message(error);
}


void print_error_message(int errorNumber)
{
	switch(errorNumber)
	{
    case CAN_ERR_OK:
        //std::cout << "NO error!\n";
        break;
    case CAN_ERR_XMTFULL:
        std::cout << "transmit buffer full\n";
        break;
    case CAN_ERR_OVERRUN:
        std::cout << "overrun in receive buffer\n";
        break;
    case CAN_ERR_BUSLIGHT:
        std::cout << "bus error, errorcounter limit reached\n";
        break;
    case CAN_ERR_BUSHEAVY:
        std::cout << "bus error, errorcounter limit reached\n";
        break;
    case CAN_ERR_BUSOFF:
        std::cout << "bus error, 'bus off' state entered\n";
        break;
    case CAN_ERR_QRCVEMPTY:
        std::cout << "receive queue is empty\n";
        break;
    case CAN_ERR_QOVERRUN:
        std::cout << "receive queue overrun\n";
        break;
    case CAN_ERR_QXMTFULL:
        std::cout << "transmit queue full\n";
        break;
    case CAN_ERR_REGTEST:
        std::cout << "test of controller registers failed\n";
        break;
    case CAN_ERR_NOVXD:
        std::cout << "Win95/98/ME only\n";
        break;
    case CAN_ERR_RESOURCE:
        std::cout << "can't create resource\n";
        break;
    case CAN_ERR_ILLPARAMTYPE:
        std::cout << "illegal parameter\n";
        break;
    case CAN_ERR_ILLPARAMVAL:
        std::cout << "value out of range\n";
        break;
    case CAN_ERRMASK_ILLHANDLE:
        std::cout << "wrong handle, handle error\n";
        break;
    default:
        std::cout << "Unknown Error!\n";
        break;
	}
	return;
}

} // namespace utils
} // namespace vulcan

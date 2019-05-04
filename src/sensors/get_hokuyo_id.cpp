/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     get_hokuyo_id.cpp
* \author   Collin Johnson
*
* get_hokuyo_id is a simple program to print the id of a Hokuyo to the terminal to allow
* a udev rule to be written for assigning a constant id.
*/

#include <sensors/hokuyo_urg_laser.h>
#include <sstream>
#include <iostream>

using namespace vulcan;

/**
* get_hokuyo_id opens the Hokuyo at the specified port, connects to it, reads the serial number,
* and prints it to the terminal. This program is intended for use with a udev rule to assign a
* constant device id to a Hokuyo laser, which does not provide enough information via udev itself
* to uniquely identify it.
*
* To run the program, get_hokuyo_id 'device port number'. i.e. 0 for /dev/ttyACM0
*/
int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cerr<<"ERROR: Expected command line: get_hokuyo_id 'device port'\n";
        return -1;
    }

    sensors::HokuyoURGLaser hokuyo{argv[1], 0, false, false};

    std::cout<<hokuyo.getSerialNumber()<<'\n';

    return 0;
}

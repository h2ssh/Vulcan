/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     main.cpp
* \author   Collin Johnson
*
* Definition of the main file to get the vgo_bridge module running.
*/

#include <robot/vgo_bridge/bridge.h>

using namespace vulcan;

int main(int argc, char** argv)
{
    robot::VGoBridge bridge;

    // The bridge just sits there and does its thing, nothing else needs to happen.
    while(true)
    {
        sleep(1);
    }

    return 0;
}

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     motion_checker.cpp
* \author   Collin Johnson
*
* Definition of create_motion_checker factory.
*/

#include <robot/motion_checker.h>
#include <robot/params.h>
#include <robot/proximity_checker.h>
#include <cassert>
#include <iostream>
#include <string>

namespace vulcan
{
namespace robot
{

std::unique_ptr<MotionChecker> create_motion_checker(const motion_checker_params_t& params)
{
    if(params.checkerType == PROXIMITY_CHECKER_TYPE)
    {
        return std::unique_ptr<MotionChecker>(new ProximityChecker(params.proximityParams));
    }

    std::cerr<<"ERROR: create_motion_checker: Unknown checkerType: "<<params.checkerType<<std::endl;
    assert(false);

    return std::unique_ptr<MotionChecker>();
}

}
}

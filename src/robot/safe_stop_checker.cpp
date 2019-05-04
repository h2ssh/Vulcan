/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     safe_stop_checker.cpp
* \author   Collin Johnson
*
* Definition of SafeStopChecker.
*/

#include <robot/safe_stop_checker.h>

namespace vulcan
{
namespace robot
{

SafeStopChecker::SafeStopChecker(const safe_stop_checker_params_t& params)
    : params(params)
{

}


motion_command_t SafeStopChecker::adjustCommandIfNeeded(const motion_command_t&                command,
                                                        const std::vector<Point<float>>& scanPoints,
                                                        proximity_warning_indices_t&           warningIndices)
{

}

}
}

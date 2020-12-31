/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     safe_stop_checker.h
* \author   Collin Johnson
* 
* Declaration of SafeStopChecker.
*/

#ifndef CONTROL_ROBOT_CONTROLLER_SAFE_STOP_CHECKER_H
#define CONTROL_ROBOT_CONTROLLER_SAFE_STOP_CHECKER_H

#include "robot/motion_checker.h"
#include "robot/params.h"

namespace vulcan
{
namespace robot
{
    
const std::string SAFE_STOP_CHECKER_TYPE("safe_stop");

/**
* SafeStopChecker
*/
class SafeStopChecker : public MotionChecker
{
public:
    
    /**
    * Constructor for SafeStopChecker.
    * 
    * \param    params          Parameters defining the safe stopping distances
    */
    SafeStopChecker(const safe_stop_checker_params_t& params);
    
    // MotionChecker interface
    virtual motion_command_t adjustCommandIfNeeded(const motion_command_t&                command,
                                                   const std::vector<Point<float>>& scanPoints,
                                                   proximity_warning_indices_t&           warningIndices);

private:
    
    
    safe_stop_checker_params_t params;
};

}
}

#endif // CONTROL_ROBOT_CONTROLLER_SAFE_STOP_CHECKER_H

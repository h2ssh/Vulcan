/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     motion_checker.h
* \author   Collin Johnson
*
* Declaration of the MotionChecker interface and create_motion_checker factory.
*/

#ifndef ROBOT_MOTION_CHECKER_H
#define ROBOT_MOTION_CHECKER_H

#include <string>
#include <vector>
#include <memory>
#include <core/point.h>

namespace vulcan
{
namespace robot
{

struct velocity_command_t;
struct proximity_warning_indices_t;
struct motion_checker_params_t;

/**
* MotionChecker is an interface for classes that adjust the motion commands of the robot based
* on current sensor information. These motion checkers are like reflexes for the robot. A
* MotionChecker receives the current motion command and current relevant sensor information
* and returns a modified motion command that can then be issued to the robot.
*/
class MotionChecker
{
public:

    virtual ~MotionChecker(void) { }

    /**
    * adjustCommandIfNeeded makes any adjustments to the motion command that may be needed to
    * satisfy the safety or performance requirements of a MotionChecker implementation.
    *
    * NOTE: Currently a single laser scan is provided. In the future, this will change. Be aware.
    *
    * \param    command             Command under consideration
    * \param    scanPoints          Laser scan points around the robot in the robot's reference frame
    * \param    warningIndices      Structure containing the indices that fired a proximity warning (output)
    */
    virtual velocity_command_t adjustCommandIfNeeded(const velocity_command_t&              command,
                                                     const std::vector<Point<float>>& scanPoints,
                                                     proximity_warning_indices_t&           warningIndices) = 0;
    /**
    * hasAdjustCommand checks if the command has been adjusted.
    */
    virtual bool hasAdjustedCommand(void) = 0;
};


/**
* create_motion_checker is a factory function for creating an instance of the MotionChecker interface.
*
* NOTE: If type is not a valid MotionChecker type, then the factory will assert failure.
*
* \param    params          Parameters for the motion checker
* \return   Instance of MotionChecker.
*/
std::unique_ptr<MotionChecker> create_motion_checker(const motion_checker_params_t& params);

}
}

#endif // ROBOT_MOTION_CHECKER_H

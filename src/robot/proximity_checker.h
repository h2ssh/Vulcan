/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     proximity_checker.h
* \author   Collin Johnson
*
* Declaration of ProximityChecker subclass of MotionChecker.
*/

#ifndef ROBOT_PROXIMITY_CHECKER_H
#define ROBOT_PROXIMITY_CHECKER_H

#include <core/laser_scan.h>
#include <robot/params.h>
#include <robot/motion_checker.h>

namespace vulcan
{
namespace robot
{

const std::string PROXIMITY_CHECKER_TYPE("proximity");

/**
* ProximityChecker looks to see if any obstacles are within some safety rectangle defined around the robot.
*
* The ProximityChecker uses the following approach for determining how much to adjust the motion command:
*
*   0) Select all points within the warning boundary rectangle surrounding the robot.
*   1) Calculate the robot pose transform given the motion command.
*   2) Transform all points of interest by the calculated pose transform.
*   3) Select only those points that get closer to the robot after being transformed.
*   4) For each selected point, calculate a scaling factor based on the distance of the transformed
*      point to the robot: minFactor + (maxFactor-minFactor)*(maxDist-dist)/maxDist.
*   5) Select the highest slowdown factor and scale the motion command by that amount.
*/
class ProximityChecker : public MotionChecker
{
public:

    /**
    * Constructor for ProximityChecker.
    */
    ProximityChecker(const proximity_checker_params_t& params);

    virtual ~ProximityChecker(void);

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
                                                     proximity_warning_indices_t&           warningIndices);
    /**
    * hasAdjustCommand checks if the command has been adjusted.
    */
    virtual bool hasAdjustedCommand(void) { return slowdownFactorIsActive; };

private:
    
    bool                       slowdownFactorIsActive;
    math::Rectangle<float>     warningBoundary;
    proximity_checker_params_t params;
};

}
}

#endif // ROBOT_PROXIMITY_CHECKER_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     tilt_calibration.h
* \author   Collin Johnson
* 
* Declaration of calibrate_laser_pitch, which estimates the pitch of the laser, assuming its roll is known and the wall
* it is observing is straight.
*/

#ifndef CALIBRATION_LASER_PITCH_CALIBRATION_H
#define CALIBRATION_LASER_PITCH_CALIBRATION_H

#include "core/laser_scan.h"
#include <vector>

namespace vulcan
{
namespace calibration
{
    
/**
* tilt_t
*/
struct tilt_t
{
    double pitch;
    double roll;
    
    explicit tilt_t(double pitch = 0.0, double roll = 0.0)
    : pitch(pitch)
    , roll(roll)
    {
    }
};

/**
* tilt_calibration_results_t stores the results of a calibration. The calibration returns the best estimated tilt
* parameter -- either pitch or roll depending on function called, along with every estimated pitch and its resultant 
* error.
*/
struct tilt_calibration_results_t
{
    using tilt_error_pair = std::pair<tilt_t, double>;  // .first = pitch, .second = error
    
    tilt_t bestTilt;
    std::vector<tilt_error_pair> tiltErrors;
};


/**
* calibrate_laser_pitch estimates the pitch of a laser using a simple calibration procedure.
* 
* 1) Point the laser perpendicular to a wall that's reasonably long and record a bunch of data with the robot sitting still.
* 2) Identify the start and end rays for the wall.
* 3) Calibrate via calibrate_laser_pitch:
*   - Fit a line to all measurements taken of the wall starting at pitch = 0. Calculate MSE of points and line.
*   - Start adjusting the pitch and keep calculating new MSE based on the original fit line.
* 
* \param    beginLaser          Iterator to the first laser scan containing calibration data
* \param    endLaser            End iterator for the calibration data
* \param    lineStartIndex      Starting index of the rays hitting the line to use for fitting
* \param    lineEndIndex        One-past-the-end index of the rays hitting the line to use for fitting
* \param    minStepSize         Minimum step size to use for the calibration, i.e. max calibration error
*                               (optional, default = 1e-3)
* \param    maxPitch            Maximum pitch magnitude to consider (optional, default = 0.075 radians)
* \pre  endLaser > beginLaser (there's at least one measurement)
* \pre  lineEndIndex > lineStartIndex = 5 (there's at least 5 scan points hitting each the wall)
* \return   Results of the calibration stored in a struct for further plotting.
*/
tilt_calibration_results_t calibrate_laser_pitch(std::vector<polar_laser_scan_t>::const_iterator beginLaser,
                                                 std::vector<polar_laser_scan_t>::const_iterator endLaser,
                                                 int lineStartIndex,
                                                 int lineEndIndex,
                                                 double minStepSize = 1e-3,
                                                 double maxPitch = 0.075);

/**
* calibrate_laser_roll estimates the roll of a laser using a simple calibration procedure.
* 
* 1) Point the laser parallel to a wall that's reasonably long and record a bunch of data with the robot sitting still.
* 2) Identify the start and end rays for the wall.
* 3) Calibrate via calibrate_laser_roll:
*   - Fit a line to all measurements taken of the wall starting at roll = 0. Calculate MSE of points and line.
*   - Start adjusting the roll and keep calculating new MSE based on the original fit line.
* 
* \param    beginLaser          Iterator to the first laser scan containing calibration data
* \param    endLaser            End iterator for the calibration data
* \param    lineStartIndex      Starting index of the rays hitting the line to use for fitting
* \param    lineEndIndex        One-past-the-end index of the rays hitting the line to use for fitting
* \param    minStepSize         Minimum step size to use for the calibration, i.e. max calibration error
*                               (optional, default = 1e-3)
* \param    maxRoll             Maximum roll magnitude to consider (optional, default = 0.075 radians)
* \pre  endLaser > beginLaser (there's at least one measurement)
* \pre  lineEndIndex > lineStartIndex = 5 (there's at least 5 scan points hitting each the wall)
* \return   Results of the calibration stored in a struct for further plotting.
*/
tilt_calibration_results_t calibrate_laser_roll(std::vector<polar_laser_scan_t>::const_iterator beginLaser,
                                                std::vector<polar_laser_scan_t>::const_iterator endLaser,
                                                int lineStartIndex,
                                                int lineEndIndex,
                                                double minStepSize = 1e-3,
                                                double maxRoll = 0.075);

} // namespace calibration
} // namespace vulcan

#endif // CALIBRATION_LASER_PITCH_CALIBRATION_H

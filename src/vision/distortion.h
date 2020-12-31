/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_DISTORTION_H
#define SENSORS_VISION_DISTORTION_H

#include <vector>
#include "core/point.h"

namespace vulcan
{
class Image;

namespace vision
{

/**
* camera_calibration_t contains the parameters for a camera that allows for undistorting
* an image.
*/
struct camera_calibration_t
{
    double focalLengths[2];
    double principalPoints[2];
    double skew;

    double distortionCoeffs[4];
};

// I/O operators for camera_calibration_t
std::istream& operator>>(std::istream& in,  camera_calibration_t& calibration);
std::ostream& operator<<(std::ostream& out, camera_calibration_t& calbiration);

/**
* undistort takes an image and undistorts it using the provided calibration parameters. The
* undistorted image will be the same size as the distorted image. Pixels whose undistorted
* coordinates fall outside the bounds of the distorted image are discarded. Pixels within the
* bounds of the distorted image which no longer have a value are set to black.
*
* NOTE: This function is currently unimplemented.
*
* \param    distorted           Distorted source image
* \param    calibration         Calibration parameters
* \param    undistorted         Image in which to store the undistorted image (output)
*/
void undistort(const Image& distorted, const camera_calibration_t& calibration, Image& undistorted);

/**
* undistort takes a collection of points from an image and undistorts them using the provided calibration matrix.
* The undistorted points are stored in the undistorted vector. This vector can be the same as the distorted vector.
*
* \param    distorted           Distorted points from source image
* \param    calibration         Calibration parameters
* \param    undistorted         Undistorted points (output)
*/
void undistort(const std::vector<Point<int16_t>>& distorted,
               const camera_calibration_t&               calibration,
               std::vector<Point<int16_t>>&       undistorted);

}
}

#endif // SENSORS_VISION_DISTORTION_H

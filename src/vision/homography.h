/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_HOMOGRAPHY_H
#define SENSORS_VISION_HOMOGRAPHY_H

#include <iosfwd>
#include "core/point.h"

namespace vulcan
{
namespace vision
{

/**
* homography_matrix_t contains the matrix values for computing the perspective transformation
* between two planes. The intended use for this ability is converting between image and world
* coordinates, allowing pixels to be mapped into the world frame and metric points to be mapped
* to a pixel in an image.
*/
struct homography_matrix_t
{
    // The following naming convention exists here:
    //  - wi  : world->image matrix
    //  - iw  : image->world matrix
    // Ugly no doubt
    double wi00;
    double wi01;
    double wi02;
    double wi10;
    double wi11;
    double wi12;
    double wi20;
    double wi21;
    double wi22;

    double iw00;
    double iw01;
    double iw02;
    double iw10;
    double iw11;
    double iw12;
    double iw20;
    double iw21;
    double iw22;
};

// IO operators for homography_matrix_t
std::istream& operator>>(std::istream& in, homography_matrix_t&        matrix);
std::ostream& operator<<(std::ostream& out, const homography_matrix_t& matrix);

/**
* world_to_image_coordinates converts a world coordinate to pixel coordinates in the camera using the provided
* homography matrix.
*/
Point<float> world_to_image_coordinates(const Point<float>& world, const homography_matrix_t& homography);

/**
 * image_to_world_coordinates converts an image coordinate to world coordinates in the camera using the provided
 * homography matrix.
*/
Point<float> image_to_world_coordinates(const Point<int16_t>& image, const homography_matrix_t& homography);

}
}

#endif // SENSORS_VISION_HOMOGRAPHY_H

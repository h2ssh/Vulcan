/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_IMAGE_SEGMENT_H
#define SENSORS_VISION_IMAGE_SEGMENT_H

#include "core/point.h"
#include "math/geometry/polygon.h"
#include <stdint.h>
#include <vector>

namespace vulcan
{
namespace vision
{

/**
 * image_segment_t represents a connected component in an image based on some
 * sameness criterion. An image segment is represented by:
 *
 *   - a set of pixels
 *   - a convex hull polygon boundary, which provides a loose bound on the region
 *   - average color of the pixels in the segment
 */
struct image_segment_t
{
    image_segment_t(void) { averageColor[0] = averageColor[1] = averageColor[2] = 0.0f; }

    int64_t timestamp = -1;
    int32_t imageNumber = -1;

    uint32_t segmentId = 0;

    std::vector<Point<int16_t>> pixels;
    std::vector<Point<int16_t>> boundaryPixels;
    math::Polygon<int16_t> boundingPolygon;

    // For MONO images, all three values are the same
    float averageColor[3];
};

}   // namespace vision
}   // namespace vulcan

#endif   // SENSORS_VISION_IMAGE_SEGMENT_H

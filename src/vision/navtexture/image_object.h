/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_NAVTEXTURE_IMAGE_OBJECT_H
#define SENSORS_VISION_NAVTEXTURE_IMAGE_OBJECT_H

#include <core/point.h>
#include <vision/pixel_histograms.h>
#include <vision/spin_image.h>

namespace vulcan
{
namespace vision
{

/**
* image_object_t represents an object extracted from the current image. An object
* contains a feature descriptor consisting of a color histogram and spin image. 
* This combination of features provides color and texture information about the
* surface from which the object was extracted.
* 
* An image object contains the following information:
* 
*   - center            : center pixel of the object
*   - average color     : average color of the interior pixels
*   - boundary pixels   : indices of the pixels on the boundary of the object
*   - histogram         : color histogram of pixels in the object
*   - spin image        : spin image of the intensities around the center of the object
*   - ground flag       : 0 = unknown, -1 = not ground, 1 = ground
*   - has agent flag    : 1 = agent on segment, 0 = no agent, -1 = segment *is* agent
*
* NOTE: Histograms and spin images are only calculated for segments for whom hasAgent=1
*/
struct image_object_t
{
    Point<int16_t>               center;
    std::vector<Point<int16_t>> boundaryPixels;
    float                              averageColor[3];
    
    boost::shared_ptr<PixelHistogram> histogram;
    SpinImage                         spin;
    
    int8_t groundPlane;
    int8_t hasAgent;
};

}
}

#endif // SENSORS_VISION_NAVTEXTURE_IMAGE_OBJECT_H

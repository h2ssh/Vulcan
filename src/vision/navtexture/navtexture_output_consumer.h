/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_NAVTEXTURE_NAVTEXTURE_OUTPUT_CONSUMER_H
#define SENSORS_VISION_NAVTEXTURE_NAVTEXTURE_OUTPUT_CONSUMER_H

#include <vector>
#include <vision/image_segment.h>

namespace vulcan
{
namespace vision
{
    
/**
* NavTextureOutputConsumer is an interface for classes that will be handling the
* output produced by the navtexture module.
*
* The current output is:
*
* TODO: Fill in the blanks here
*/
class NavTextureOutputConsumer
{
public:
    
    virtual ~NavTextureOutputConsumer(void) { }
    
    virtual void handleImageSegments(const std::vector<image_segment_t>& segments) = 0;
};

}
}

#endif // SENSORS_VISION_NAVTEXTURE_NAVTEXTURE_OUTPUT_CONSUMER_H

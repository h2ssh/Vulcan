/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_NAVTEXTURE_NAVTEXTURE_INPUT_CONSUMER_H
#define SENSORS_VISION_NAVTEXTURE_NAVTEXTURE_INPUT_CONSUMER_H

#include <string>
#include <vector>

namespace vulcan
{
struct pose_t;
namespace laser
{
struct polar_laser_scan_t;
}

namespace vision
{

class Image;

/**
 * NavTextureInputConsumer is an interface for classes handling data coming into the
 * module. Current data being used:
 *
 * 0) Image           : raw images from wheelchair-mounted camera
 * 1) Robot pose      : pose of the robot from the LPM
 * 2) Dynamic objects : objects being tracked by the laser
 */
class NavTextureInputConsumer
{
public:
    virtual ~NavTextureInputConsumer(void) { }

    // Handlers for the types of data arriving
    virtual void handleData(const Image& image, const std::string& channel) = 0;
    virtual void handleData(const polar_laser_scan_t& scan, const std::string& channel) = 0;
    virtual void handleData(const pose_t& pose, const std::string& channel) = 0;
    virtual void handleData(const tracker::DynamicObjectCollection& objects, const std::string& channel) = 0;
    virtual void handleData(const laser::dynamic_laser_points_t& dynamicPoints, const std::string& channel) = 0;
};

}   // namespace vision
}   // namespace vulcan

#endif   // SENSORS_VISION_NAVTEXTURE_NAVTEXTURE_INPUT_CONSUMER_H

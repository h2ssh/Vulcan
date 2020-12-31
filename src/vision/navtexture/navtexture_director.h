/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_NAVTEXTURE_NAVTEXTURE_DIRECTOR_H
#define SENSORS_VISION_NAVTEXTURE_NAVTEXTURE_DIRECTOR_H

#include <fstream>
#include "system/director.h"
#include "utils/condition_variable.h"
#include "utils/mutex.h"
#include "core/image.h"
#include "core/laser_scan.h"
#include "laser/dynamic_laser_points.h"
#include "vision/navtexture/image_object_identifier.h"
#include "vision/navtexture/image_object.h"
#include "vision/navtexture/navtexture_params.h"
#include "vision/navtexture/navtexture_input_consumer.h"
#include "vision/navtexture/navtexture_output_consumer.h"

namespace vulcan
{

namespace laser
{
    struct dynamic_laser_points_t;
}

namespace vision
{

class NavTextureOutputConsumer;

/**
* NavTextureDirector is responsible for organizing the processing of incoming data. The incoming data:
*
* 0) Raw image
* 1) Robot pose
* 2) Dynamic objects identified in the map
*
* The raw image is segments and the textures in the segments are classified. The ground plane segments
* adjacent to dynamic objects have an increased likelihood of being navigable.
*
* The output is:
*
* 0) Ground navigability classification
*/
class NavTextureDirector : public system::Director<NavTextureOutputConsumer>,
                           public NavTextureInputConsumer
{
public:

    /**
    * Constructor for NavTextureDirector.
    */
    NavTextureDirector(const navtexture_params_t& params);

    /**
    * Destructor for NavTextureDirector.
    */
    virtual ~NavTextureDirector(void);

    // Handlers for the input consumer interface
    virtual void handleData(const Image&                                image,         const std::string& channel);
    virtual void handleData(const polar_laser_scan_t&            scan,          const std::string& channel);
    virtual void handleData(const pose_t&                        pose,          const std::string& channel);
    virtual void handleData(const laser::dynamic_laser_points_t&        dynamicPoints, const std::string& channel);
    virtual void handleData(const tracker::DynamicObjectCollection& objects,       const std::string& channel);

private:

    // Director interface implementation
    virtual void waitForData(void);
    virtual void processAvailableData(void);
    virtual void transmitCalculatedOutput(void);

    bool haveRequiredDataForCalculation(void);

    ImageObjectIdentifier         objectIdentifier;
    Image                         currentImage;
    polar_laser_scan_t     currentScan;
    laser::dynamic_laser_points_t dynamicPoints;
    std::vector<image_object_t>   imageObjects;

    navtexture_params_t params;

    int imageNumber;

    bool haveImage;
    bool haveLaser;
    bool haveDynamic;

    utils::Mutex             dataLock;
    utils::ConditionVariable dataTrigger;
};

}
}

#endif // SENSORS_VISION_NAVTEXTURE_NAVTEXTURE_DIRECTOR_H

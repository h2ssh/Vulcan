/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_NAVTEXTURE_NAVTEXTURE_COMMUNICATOR_H
#define SENSORS_VISION_NAVTEXTURE_NAVTEXTURE_COMMUNICATOR_H

#include "system/communicator.h"
#include "vision/navtexture/navtexture_output_consumer.h"

namespace vulcan
{
namespace vision
{

class NavTextureInputConsumer;

/**
 * NavTextureCommunicator handles all communications with other modules. Data required
 * by this module is received from other modules, and processed data is pushed out to
 * other modules that will use it.
 */
class NavTextureCommunicator
: public system::Communicator
, public NavTextureOutputConsumer
{
public:
    /**
     * Constructor for NavTextureCommunicator.
     */
    NavTextureCommunicator(void);

    /**
     * addInputConsumer adds a new consumer for incoming data.
     */
    void addInputConsumer(NavTextureInputConsumer* consumer);

    // Handlers for output consumer interface
    virtual void handleImageSegments(const std::vector<image_segment_t>& segments);
};

}   // namespace vision
}   // namespace vulcan

#endif   // SENSORS_VISION_NAVTEXTURE_NAVTEXTURE_COMMUNICATOR_H

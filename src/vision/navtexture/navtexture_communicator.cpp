/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "vision/navtexture/navtexture_communicator.h"
#include "vision/navtexture/navtexture_input_consumer.h"

namespace vulcan
{
namespace vision
{

NavTextureCommunicator::NavTextureCommunicator(void)
{
}


void NavTextureCommunicator::addInputConsumer(NavTextureInputConsumer* consumer)
{
    receiver.subscribeToMessage<Image>(consumer);
    receiver.subscribeToMessage<polar_laser_scan_t>(consumer);
    receiver.subscribeToMessage<laser::dynamic_laser_points_t>(consumer);
    receiver.subscribeToMessage<tracker::DynamicObjectCollection>(consumer);
    receiver.subscribeToMessage<pose_t>(consumer);
}


void NavTextureCommunicator::handleImageSegments(const std::vector<image_segment_t>& segments)
{
    transmitter.sendMessage(segments);
}

}   // namespace vision
}   // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     event_detector.cpp
* \author   Collin Johnson
* 
* Implementation of EventDetector.
*/

#include "hssh/local_topological/event_detector.h"
#include "hssh/local_topological/event_detection/area_event_detector.h"
#include "hssh/local_topological/params.h"
#include <iterator>

namespace vulcan
{
namespace hssh
{

EventDetector::EventDetector(const utils::ConfigFile& config)
{
    event_detector_params_t params(config);
    detectors_ = create_area_event_detectors(params.detectorTypes, params);
}


EventDetector::~EventDetector(void)
{
    // Just need the definition here to unique_ptr is valid
}


LocalAreaEventVec EventDetector::detectEvents(const LocalTopoMap& map)
{
    LocalAreaEventVec allEvents;

    for(auto& detector : detectors_)
    {
        auto&& events = detector->detectEvents(map);
        
        allEvents.insert(allEvents.end(), 
                         std::make_move_iterator(events.begin()), 
                         std::make_move_iterator(events.end()));
    }
    
    return allEvents;
}


void EventDetector::addPose(const LocalPose& pose)
{
    for(auto& detector : detectors_)
    {
        detector->addPose(pose);
    }
}

}
}

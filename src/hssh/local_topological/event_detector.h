/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     event_detector.h
* \author   Collin Johnson
* 
* Definition of EventDetector.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_EVENT_DETECTOR_H
#define HSSH_LOCAL_TOPOLOGICAL_EVENT_DETECTOR_H

#include <hssh/local_topological/area.h>
#include <hssh/local_topological/event.h>
#include <vector>

namespace vulcan
{
namespace utils { class ConfigFile; }
namespace hssh
{
    
class AreaEventDetector;
class LocalPose;
class LocalTopoMap;

/**
* EventDetector organizes the event detection process by providing implementations of the AreaEventDetector interface
* with information about the current understanding of the local topology. These event detectors will themselves handle
* the actual detection based on the current state of the world.
* 
* The event detector needs information about the pose of the robot to work. Poses should be added via the addPose method
* at least as often as the local topology is recalculated. Adding every calculated pose is fine.
* 
* The configuration parameters for the EventDetector are:
* 
*   [EventDetectorParameters]
*   detectors = comma-separated list of event detectors to be initialized
*/
class EventDetector
{
public:
    
    /**
    * Constructor for EventDetector.
    * 
    * \param    config          ConfigFile containing parameters for the EventDetector
    */
    EventDetector(const utils::ConfigFile& config);
    
    /**
    * Destructor for EventDetector.
    */
    ~EventDetector(void);
    
    /**
    * detectEvents detects local topological events occurring in the robot's local surround. The current area the robot is
    * in, along with all tracked areas are provided to the method. The types of events detected are determined by the
    * loaded detectors, specified in the parameters for EventDetector.
    * 
    * \param    map                 Map in which the robot is currently located
    * \return   The events found in the current map since the last check
    */
    LocalAreaEventVec detectEvents(const LocalTopoMap& map);

    /**
    * addPose adds a new pose to the event detector's trace of poses to be used for deciding where the robot is
    * in the environment.
    *
    * \param    pose            Pose to be added
    */
    void addPose(const LocalPose& pose);
    
private:
    
    std::vector<std::unique_ptr<AreaEventDetector>> detectors_;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_EVENT_DETECTOR_H

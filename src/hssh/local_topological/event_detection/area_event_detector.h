/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_event_detector.h
* \author   Collin Johnson
* 
* Definition of AreaEventDetector interface and create_area_event_detectors factory.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_EVENTS_AREA_EVENT_DETECTOR_H
#define HSSH_LOCAL_TOPOLOGICAL_EVENTS_AREA_EVENT_DETECTOR_H

#include <hssh/local_topological/event.h>
#include <memory>
#include <string>
#include <vector>

namespace vulcan
{
namespace robot { class pose_t; }
namespace hssh
{

class  AreaEventDetector;
class  LocalTopoMap;
struct event_detector_params_t;
    
/**
* create_area_event_detectors is a factory that loads a collection of event detectors using the string identifiers.
* 
* \param    detectors           Detectors to be created, as loaded from a config file
* \param    params              Parameters for the event detectors
* \return   Instances of AreaEventDetector.
*/
std::vector<std::unique_ptr<AreaEventDetector>> create_area_event_detectors(const std::vector<std::string>& detectors,
                                                                            const event_detector_params_t&  params);

/**
* AreaEventDetector is an interface for detecting events occurring within the local surround. These events
* correspond to things like entering or exiting an area or turning around on a path.
* 
* The interface for an AreaEventDetector consists of two parts:
* 
*   1) Checking if any events have occurred (hasEventOccurred) 
*   2) Adding those events to a collection of events (addEvents)
* 
* These methods should be used in conjunction with one another. The events are determined by comparing the state of
* the robot and environment in the previous call to hasEventOccurred with the current state. As a result, if events
* have occurred, addEvents must be called before another call to hasEventOccurred, otherwise the events will be wiped
* out.
*/
class AreaEventDetector
{
public:
    
    virtual ~AreaEventDetector(void) { }
    
    /**
    * detectEvents checks to see if any events have occurred since the previous call to AreaEventDetector.
    * The exact conditions for the events depend on the implementations of the interface.
    * 
    * \param    map         Map of areas in the LPM
    * \return   Events found by the detector.
    */
    virtual LocalAreaEventVec detectEvents(const LocalTopoMap& map) = 0;

    /**
    * addPose adds a new pose to the event detector's trace of poses to be used for deciding where the robot is
    * in the environment.
    *
    * \param    pose            Pose to be added
    */
    virtual void addPose(const LocalPose& pose) = 0;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_EVENTS_AREA_EVENT_DETECTOR_H

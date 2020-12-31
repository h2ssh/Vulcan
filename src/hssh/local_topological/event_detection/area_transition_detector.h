/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_transition_detector.h
* \author   Collin Johnson
* 
* Definition of AreaTransitionDetector.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_EVENTS_AREA_TRANSITION_DETECTOR_H
#define HSSH_LOCAL_TOPOLOGICAL_EVENTS_AREA_TRANSITION_DETECTOR_H

#include "hssh/local_topological/event_detection/area_event_detector.h"
#include "hssh/local_topological/gateway.h"
#include "hssh/local_topological/params.h"
#include "math/geometry/rectangle.h"
#include "utils/pose_trace.h"
#include "utils/timed_sequence.h"
#include <fstream>

namespace vulcan
{
namespace hssh
{
    
class LocalArea;

    
const std::string kAreaTransitionDetectorType("area_transition");

/**
* AreaTransitionDetector detects transitions between one area and another area. An area transition occurs
* when the robot crosses a gateway boundary between adjacent areas.
*/
class AreaTransitionDetector : public AreaEventDetector
{
public:
    
    /**
    * Constructor for AreaTransitionDetector.
    * 
    * \param    params          Parameters for controlling when the transition events fire
    */
    AreaTransitionDetector(const area_transition_detector_params_t& params);
    
    // AreaEventDetector interface
    LocalAreaEventVec detectEvents(const LocalTopoMap& map) override;
    void addPose(const LocalPose& pose) override;

private:

    bool isInitialArea_;
    utils::PoseTrace poses_;
    utils::TimedSequence<LocalPose> localPoses_;
    math::Rectangle<float> previousBoundary_;       // boundary of area robot was last reported to be in
    Gateway latestGateway_;                         // gateway cross during most recent event
    area_transition_detector_params_t params_;
    
    std::ofstream transitionLog_;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_EVENTS_AREA_TRANSITION_DETECTOR_H

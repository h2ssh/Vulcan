/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     path_direction_detector.cpp
* \author   Collin Johnson
* 
* Implementation of PathDirectionDetector.
*/

#include "hssh/local_topological/event_detection/path_direction_detector.h"
#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/areas/path_segment.h"
#include "hssh/local_topological/events/turn_around.h"
#include <iostream>
#include <cassert>

#define DEBUG_EVENTS

namespace vulcan
{
namespace hssh
{

const int MOVING_AWAY   = 1;
const int MOVING_TOWARD = -1;


PathDirectionDetector::PathDirectionDetector(const path_direction_detector_params_t& params)
: detectorState_(State::WaitingForPath)
, havePose_(false)
, shouldCreateEvent_(false)
, events_(0)
, hysteresisRadius_(params.hysteresisRadius)
{
}


PathDirectionDetector::~PathDirectionDetector(void)
{
    // For std::unique_ptr
}


LocalAreaEventVec PathDirectionDetector::detectEvents(const LocalTopoMap& map)
{
    LocalAreaEventVec events;
    
    shouldCreateEvent_ = false;
    events_ = &events;
    
    // If no new poses have arrived, then no events could have occurred
    if(!havePose_)
    {
        return events;
    }
    
    auto containingAreas = map.allAreasContaining(currentPose_.pose().toPoint());
    
    // Only check for an event if in a single area at the current time
    if(containingAreas.size() == 1)
    {
        containingAreas.front()->accept(*this);
        if(shouldCreateEvent_)
        {
            createTurnedAroundEvent(std::static_pointer_cast<LocalPathSegment>(containingAreas.front()));
        }
    }
    
    return events;
}


void PathDirectionDetector::addPose(const LocalPose& pose)
{
    currentPose_ = pose;
    havePose_ = true;
}


void PathDirectionDetector::visitDecisionPoint(const LocalDecisionPoint& decision)
{
    // If currently at a decision, jump back to WaitingForPath as the detector only works with LocalPathSegment
    detectorState_ = State::WaitingForPath;
}


void PathDirectionDetector::visitDestination(const LocalDestination& destination)
{
    // If currently at a destination, jump back to WaitingForPath as the detector only works with LocalPathSegment
    detectorState_ = State::WaitingForPath;
}


void PathDirectionDetector::visitPathSegment(const LocalPathSegment& path)
{
    assert(havePose_);
    assert(events_);
    
    switch(detectorState_)
    {
    case State::WaitingForPath:
        initializeTurnaroundCheck(currentPose_.pose(), path);
        currentPathId_ = path.id();
        detectorState_ = State::CheckForTurnaround;
        break;
        
        // If the robot has turned around, then switch over to checking if the robot has actually turned around
        // based on the moving far enough from the turn around point
    case State::CheckForTurnaround:
        if(haveTurnedAround(currentPose_.pose()))
        {
            turnaroundPose_ = lastPose_;
            detectorState_  = State::CheckHysteresis;
        }
        break;
        
        // When looking if the robot has moved out of the hysteresis zone, also check if the robot has turned back
        // to the original direction of motion. If that happens, then just go back to looking for a turnaround and
        // don't fire a new event
    case State::CheckHysteresis:
        if(haveMovedThroughHysteresis(currentPose_.pose()))
        {
            shouldCreateEvent_ = true;
            detectorState_     = State::CheckForTurnaround;
        }
        else if(haveTurnedAround(currentPose_.pose()))
        {
            detectorState_ = State::CheckForTurnaround;
        }
        break;
    }
    
    lastPose_ = currentPose_;
    havePose_ = false;
}


void PathDirectionDetector::initializeTurnaroundCheck(const pose_t& pose, const LocalPathSegment& path)
{
    entryPoint_        = pose.toPoint();
    directionOfMotion_ = MOVING_AWAY;
    
    // STUB: PathDirectionDetector::initializeTurnaroundCheck()
    std::cerr << "STUB: PathDirectionDetector::initializeTurnaroundCheck(): How are the ends selected?\n";
    
#ifdef DEBUG_EVENTS
    std::cout << "INFO:PathCreator:Entered new path:" << entryPoint_ << '\n';
#endif
}


bool PathDirectionDetector::haveTurnedAround(const pose_t& pose) const
{
    Point<float> robotPosition(pose.toPoint());
    Point<float> directionPoint(robotPosition.x + cos(pose.theta), robotPosition.y + sin(pose.theta));
    
    // When moving outbound, the turned around checks to see if angle has gone below pi/2
    // however, when doing the hysteresis check, want to see if moving back outbound again, hence flipping
    // the direction being considered
    float angle     = angle_between_points(entryPoint_, directionPoint, robotPosition);
    int   direction = (detectorState_ == State::CheckHysteresis) ? -directionOfMotion_ : directionOfMotion_;
    
#ifdef DEBUG_EVENTS
    if((angle - M_PI/2.0) * direction < 0)
    {
        std::cout << "INFO::PathDirection:Detected turnaround. Entry:" << entryPoint_ << " Robot:" << robotPosition
                  << "Direction:"<<directionPoint << " Motion:" << directionOfMotion_ << '\n';
    }
#endif
    
    return (angle - M_PI/2.0) * direction < 0;
}


bool PathDirectionDetector::haveMovedThroughHysteresis(const pose_t& pose) const
{
    float turnaroundRadius = distance_between_points(entryPoint_, turnaroundPose_.pose().toPoint());
    float poseRadius       = distance_between_points(entryPoint_, pose.toPoint());
    
#ifdef DEBUG_EVENTS
    if((turnaroundRadius - poseRadius) * directionOfMotion_ > hysteresisRadius_)
    {
        std::cout << "INFO::PathDirection:Moved through hysteresis region: Turnaround:" << turnaroundRadius
                  << " Pose:" << poseRadius << " Hyst:" << hysteresisRadius_ << " Motion:" << directionOfMotion_ << '\n';
    }
#endif
    
    // When moving toward entry, direction is negative and a turnaround means moving away, so r_turn < r_pose
    // thus, multiplying by the direction flips the sign correctly
    return (turnaroundRadius - poseRadius) * directionOfMotion_ > hysteresisRadius_;
}


void PathDirectionDetector::createTurnedAroundEvent(const std::shared_ptr<LocalPathSegment>& path)
{
    // If was moving away and then turned around, then the current direction is going back to the entry, so PLUS.
    auto currentDirection  = (directionOfMotion_ == MOVING_AWAY) ? TopoDirection::plus  : TopoDirection::minus;
    auto previousDirection = (directionOfMotion_ == MOVING_AWAY) ? TopoDirection::minus : TopoDirection::plus;
    
    events_->push_back(std::unique_ptr<LocalAreaEvent>(new TurnAroundEvent(turnaroundPose_.timestamp(),
                                                                           path,
                                                                           currentDirection,
                                                                           previousDirection,
                                                                           turnaroundPose_)));
    directionOfMotion_ *= -1;   // flip the sign of the direction so it points the other way
}

} // namespace hssh
} // namespace vulcan

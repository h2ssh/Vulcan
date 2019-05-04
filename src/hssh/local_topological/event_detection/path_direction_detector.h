/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     path_direction_detector.h
* \author   Collin Johnson
* 
* Definition of PathDirectionDetector.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_EVENTS_PATH_DIRECTION_DETECTOR_H
#define HSSH_LOCAL_TOPOLOGICAL_EVENTS_PATH_DIRECTION_DETECTOR_H

#include <hssh/local_topological/event_detection/area_event_detector.h>
#include <hssh/local_topological/area_visitor.h>
#include <hssh/local_topological/params.h>

namespace vulcan
{
namespace hssh
{
    
const std::string kPathDirectionDetectorType("path_direction");

/**
* PathDirectionDector monitors the progress of the robot as it moves along a path. The purpose is to
* detect if the robot has turned around mid-path, before reaching the next area. To detect
* turning around mid-path, the motion of the robot relative to the entry gateway is monitored. 
* If the robot begins moving toward the entry gateway, then after a short time, a 
* TurnedAroundEvent will be generated indicating the robot is moving toward the entry
* gateway rather than the opposite end of the path.
*
* The formal calculation proceeds as follows:
*
*   1) A new path is set for the detector. The center of the entry gateway is stored.
*   2) As the robot moves along the path, the angle between the vectors going from gateway center to robot center
*      and robot center to one meter along the robot's orientation are calculated.
*   3) If the angle between these vectors is >pi/2, the robot is moving to the next place.
*   4) If the angle is below pi/2, mark the last location where it was above as p.
*   5) Calculate r_entry,p and r_entry,cur. If r_entry,p - r_entry,cur > r_thresh, generate a turned around event.
*/
class PathDirectionDetector : public AreaEventDetector,
                              public LocalAreaVisitor
{
public:
    
    /**
    * Constructor for PathDirectionDetector.
    * 
    * \param    params          Parameters for controlling when the transition events fire
    */
    PathDirectionDetector(const path_direction_detector_params_t& params);
    
    /**
    * Destructor for PathDirectionDetector.
    */
    virtual ~PathDirectionDetector(void);
    
    // AreaEventDetector interface
    LocalAreaEventVec detectEvents(const LocalTopoMap& map) override;
    void addPose(const LocalPose& pose) override;
    
    // LocalAreaVisitor interface
    void visitDecisionPoint(const LocalDecisionPoint& decision) override;
    void visitDestination  (const LocalDestination& destination) override;
    void visitPathSegment  (const LocalPathSegment& path) override;
    
private:
    
    enum class State
    {
        WaitingForPath,
        CheckForTurnaround,
        CheckHysteresis
    };
    
    State              detectorState_;
    int32_t            currentPathId_;
    Point<float> entryPoint_;
    int                directionOfMotion_;
    LocalPose currentPose_;
    LocalPose lastPose_;
    LocalPose turnaroundPose_;

    bool               havePose_;
    bool               shouldCreateEvent_;
    LocalAreaEventVec* events_;
    
    const float hysteresisRadius_;
    
    
    void initializeTurnaroundCheck (const pose_t& pose, const LocalPathSegment& path);
    bool haveTurnedAround          (const pose_t& pose) const;
    bool haveMovedThroughHysteresis(const pose_t& pose) const;
    void createTurnedAroundEvent   (const std::shared_ptr<LocalPathSegment>& path);
};

} // namespace hssh 
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_EVENTS_PATH_DIRECTION_DETECTOR_H

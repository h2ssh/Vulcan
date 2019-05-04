/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     metric_path_creator.cpp
* \author   Collin Johnson
*
* Definition of MetricPathCreator.
*/

#include <ui/common/metric_path_creator.h>
#include <utils/timestamp.h>
#include <iostream>

// #define DEBUG_PATH_CREATOR

namespace vulcan
{
namespace ui
{

pose_t create_pose_from_positions(const Point<float>& position, const Point<float>& orientation);


PoseSelector::PoseSelector(void)
    : haveSelectedTarget(false)
    , amSelectingTarget(false)
{
}


void PoseSelector::reset(void)
{
    haveSelectedTarget = false;
    amSelectingTarget  = false;
}


GLEventStatus PoseSelector::handleLeftMouseDown(const GLMouseEvent& event)
{
    amSelectingTarget  = true;
    haveSelectedTarget = false; // start the selection process again, so any previous target is eliminated

    targetPosition     = event.glCoords;
    hoverTarget        = pose_t(targetPosition.x, targetPosition.y, 0); // The angle is assigned via the mouse motion
    
    return event.leftIsDown ? GLEventStatus::capture : GLEventStatus::passthrough;
}


GLEventStatus PoseSelector::handleLeftMouseUp(const GLMouseEvent& event)
{
    if(amSelectingTarget)
    {
        selectedTarget = create_pose_from_positions(targetPosition, event.glCoords);
        
        amSelectingTarget  = false;
        haveSelectedTarget = true;
    }
    
    return GLEventStatus::passthrough;
}


GLEventStatus PoseSelector::handleMouseMoved(const GLMouseEvent& event)
{
    if(amSelectingTarget)
    {
        // Keep a current waypoint up-to-date because it might be desired
        hoverTarget = create_pose_from_positions(targetPosition, event.glCoords);
    }
    else
    {
        hoverTarget = pose_t(event.glCoords.x, event.glCoords.y, 0.0f);
    }
    
    return event.leftIsDown ? GLEventStatus::capture : GLEventStatus::passthrough;
}


pose_t create_pose_from_positions(const Point<float>& position, const Point<float>& orientation)
{
    return pose_t(utils::system_time_us(), position.x, position.y, std::atan2(orientation.y-position.y, orientation.x-position.x));
}

} // namespace ui
} // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     metric_path_creator.h
* \author   Collin Johnson
*
* Definition of MetricPathCreator.
*/

#ifndef UI_COMMON_METRIC_PATH_CREATOR_H
#define UI_COMMON_METRIC_PATH_CREATOR_H

#include <core/pose.h>
#include <ui/common/gl_event.h>

namespace vulcan
{
namespace ui
{

/**
* MetricPathCreator is responsible for making new controller_waypoint_path_t based on the where the user clicks
* on the displayed map. The paths are made by combining the mouse position at two events. When the mouse
* button is pressed, that establishes the location of waypoint. The angle between the mouse position when
* the button is released establishes the desired orientation. Simple and cool.
*
* The path creator can have a bounded path length if desired. If creating a single waypoint target, a max
* length of 1 can be specified. If the max length of a path has been reached, any subsequent waypoints will
* replace the last waypoint in the path.
*/
class PoseSelector : public GLMouseHandler
{
public:

    /**
    * Default constructor for PoseSelector.
    */
    PoseSelector(void);

    /**
    * reset resets the state of the target creator, eliminating the selected target, if there was one.
    */
    void reset(void);

    /**
    * getHoverTarget retrieves the target over which the mouse is currently hovering. If a target is being
    * actively created, the hover target will be the same as the selected target.
    */
    pose_t getHoverTarget(void) const { return hoverTarget; }

    /**
    * getSelectedTarget retrieves the target that was last selected by the user. The hasSelectedTarget flag indicates
    * if a target has actually been selected. If no target was selected, then this target will be the same as the
    * hover target.
    */
    pose_t getSelectedTarget(void) const { return haveSelectedTarget ? selectedTarget : hoverTarget; }

    /**
    * hasSelectedTarget checks to see if a target has been selected by the user yet.
    */
    bool hasSelectedTarget(void) const { return haveSelectedTarget; }

    // GLMouseHandler interface
    virtual GLEventStatus handleLeftMouseDown(const GLMouseEvent& event);
    virtual GLEventStatus handleLeftMouseUp  (const GLMouseEvent& event);
    virtual GLEventStatus handleMouseMoved   (const GLMouseEvent& event);

private:

    pose_t hoverTarget;
    pose_t selectedTarget;
    bool          haveSelectedTarget;
    bool          amSelectingTarget;

    Point<float> targetPosition;
};

} // namespace ui
} // namespace vulcan

#endif // UI_COMMON_METRIC_PATH_CREATOR_H

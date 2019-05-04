/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_area_event_renderer.h
* \author   Collin Johnson
*
* Declaration of LocalAreaEventRenderer.
*/

#ifndef UI_COMPONENTS_LOCAL_AREA_EVENT_RENDERER_H
#define UI_COMPONENTS_LOCAL_AREA_EVENT_RENDERER_H

#include <hssh/local_topological/event_visitor.h>

namespace vulcan
{
namespace hssh { class LocalAreaEvent; }
namespace ui
{

/**
* LocalAreaEventRenderer draws a LocalAreaEvent. The events are drawn in the following ways:
*
*   - AreaTransition : an arrow goes from the previous area, across the gateway, and into the new area.
*   - TurnAround     : an arrow pointing from the turn around point towards the new direction of motion
*/
class LocalAreaEventRenderer : public hssh::LocalAreaEventVisitor
{
public:

    /**
    * renderEvent draws the event that has occurred.
    *
    * \param    event       Event to draw
    */
    void renderEvent(const hssh::LocalAreaEvent& event);

    // LocalAreaEventVisitor interface
    virtual void visitAreaTransition(const hssh::AreaTransitionEvent& event);
    virtual void visitTurnAround    (const hssh::TurnAroundEvent&     event);
};

}
}

#endif // UI_COMPONENTS_LOCAL_AREA_EVENT_RENDERER_H

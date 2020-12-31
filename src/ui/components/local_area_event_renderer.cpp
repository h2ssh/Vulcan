/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_event_renderer.cpp
* \author   Collin Johnson
*
* Definition of LocalTopoEventRenderer.
*/

#include "ui/components/local_area_event_renderer.h"
#include "ui/common/default_colors.h"
#include "ui/common/gl_shapes.h"
#include "ui/common/ui_color.h"
#include "hssh/local_topological/events/area_transition.h"
#include "hssh/local_topological/events/turn_around.h"

namespace vulcan
{
namespace ui
{

void LocalAreaEventRenderer::renderEvent(const hssh::LocalAreaEvent& event)
{
    event.accept(*this);
}


void LocalAreaEventRenderer::visitAreaTransition(const hssh::AreaTransitionEvent& event)
{
    // Draw an arrow with the tail in the previous area and the head in the new area
    // Draw the arrow so it is 1m long.
    // The orientation of the arrow is orthogonal to the gateway
    
    const float kArrowLength = 1.0f;
    
    auto gateway = event.transitionGateway();
    
    // If there's no gateway, then this is the initial event and it doesn't have a good visualization
    if(gateway)
    {
        float dirToNewArea = angle_to_point(gateway->center(), event.enteredArea()->center().toPoint());
        float leftDirDiff  = std::abs(angle_diff(dirToNewArea, gateway->leftDirection()));
        float rightDirDiff = std::abs(angle_diff(dirToNewArea, gateway->rightDirection()));
        
        float arrowDir = (leftDirDiff < rightDirDiff) ? gateway->leftDirection() : gateway->rightDirection();
        auto arrowTail = Point<float>(gateway->center().x - std::cos(arrowDir)*kArrowLength/2.0f,
                                            gateway->center().y - std::sin(arrowDir)*kArrowLength/2.0f);
        
        decision_point_color().set();
        gl_draw_medium_arrow(arrowTail, kArrowLength, arrowDir, 2.0f);
    }
}


void LocalAreaEventRenderer::visitTurnAround(const hssh::TurnAroundEvent& event)
{
    // The tail of the arrow begins where the robot turned around 
    // The head of the arrow points to the travel affordance for the robot's new direction on the path
    auto path = event.path();
    auto directionAffordance = (event.currentDirection() == hssh::TopoDirection::plus) ? 
                                    path->moveAlongPlus().target() : path->moveAlongMinus().target();
    float direction = angle_to_point(event.turnAroundPose().pose().toPoint(), directionAffordance.toPoint());
    
    path_color().set();
    gl_draw_small_arrow(event.turnAroundPose().pose().toPoint(), 1.0f, direction, 2.0f);
}

}
}

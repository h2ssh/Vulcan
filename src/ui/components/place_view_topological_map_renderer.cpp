/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     place_view_topological_map_renderer.cpp
* \author   Collin Johnson
*
* Definition of PlaceViewTopologicalMapRenderer.
*/

#include "ui/components/place_view_topological_map_renderer.h"
#include <GL/gl.h>
#include "ui/common/default_colors.h"
#include "ui/common/gl_shapes.h"
#include "hssh/global_topological/global_place.h"
#include "hssh/local_topological/areas/place.h"

namespace vulcan
{
namespace ui
{

void PlaceViewTopologicalMapRenderer::renderPlace(const map_place_info_t& place, place_attribute_t attributes)
{
    // TODO: Fix this once the LocalAreas are being stored.

    // Need the metric map for drawing place view
    if(place.metric)
    {
        glPushMatrix();
        glTranslatef(place.referenceFrame.x, place.referenceFrame.y, 0);
        glRotatef(place.referenceFrame.theta * 180.0f / M_PI, 0.0f, 0.0f, 1.0f);

        std::cout << "Global rotation: " << place.referenceFrame.theta << " Local rotation: "
            << place.metric->extent().center().theta << '\n';

        auto boundary = place.metric->extent().rectangleBoundary();
        boundary.rotate(place.metric->extent().center().theta); // rotate it back b/c global frame determines rotation

        if(attributes == CURRENT_PLACE)
        {
            locationColor.set(0.5f);
            gl_draw_filled_rectangle(boundary);
        }

        // Draw a border around both the extent boundary and the so it is clear where the LPM ends in the visualization
        quasi_static_color().set();
        gl_draw_line_rectangle(boundary, 1.5f);

        occupied_color().set();
        gl_draw_line_rectangle(place.metric->map().getBoundary(), 1.5f);

        gridRenderer_.setGrid(place.metric->map());
        gridRenderer_.renderGrid();

        glPopMatrix();
    }
}


void PlaceViewTopologicalMapRenderer::renderPathSegment(const hssh::GlobalPathSegment& segment,
                                                        const map_place_info_t&  plusPlace,
                                                        const map_place_info_t&  minusPlace,
                                                        path_segment_attribute_t attributes)
{
    const float BORDER_WIDTH = 3.0f;

    pose_t plusLocation  = plusPlace.referenceFrame;
    pose_t minusLocation = minusPlace.referenceFrame;

    if(plusPlace.topo->id() == hssh::kFrontierId)
    {
        plusLocation = minusLocation;
    }
    else if(minusPlace.topo->id() == hssh::kFrontierId)
    {
        minusLocation = plusLocation;
    }

    glLineWidth(BORDER_WIDTH);
    glBegin(GL_LINES);

    if(plusPlace.topo->id() == hssh::kFrontierId || minusPlace.topo->id() == hssh::kFrontierId)
    {
        frontierColor.set();
    }
    else if(attributes == CURRENT_PATH)
    {
        locationColor.set();
    }
    else
    {
        pathColor.set();
    }

    glVertex2f(plusLocation.x, plusLocation.y);
    glVertex2f(minusLocation.x, minusLocation.y);

    glEnd();
}

} // namespace ui
} // namespace vulcan

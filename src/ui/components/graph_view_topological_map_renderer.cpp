/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     graph_view_topological_map_renderer.cpp
* \author   Collin Johnson
*
* Definition of GraphViewTopologicalMapRenderer.
*/

#include "ui/components/graph_view_topological_map_renderer.h"
#include "ui/components/small_scale_star_renderer.h"
#include "ui/common/default_colors.h"
#include "ui/common/gl_shapes.h"
#include "core/point.h"
#include "core/pose.h"
#include "hssh/global_topological/global_place.h"
#include "hssh/global_topological/global_path_segment.h"
#include "hssh/local_topological/areas/place.h"
#include <cmath>
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

const int    NUM_CIRCLE_VERTICES = 36;
const float  BORDER_WIDTH        = 2.0f;
const double MIN_RADIUS          = 1.0;

GraphViewTopologicalMapRenderer::GraphViewTopologicalMapRenderer(void)
: starRenderer_(new SmallScaleStarRenderer)
{
}


GraphViewTopologicalMapRenderer::~GraphViewTopologicalMapRenderer(void) = default;   // For std::unique_ptr


void GraphViewTopologicalMapRenderer::renderPlace(const map_place_info_t& place, place_attribute_t attributes)
{
    if(place.topo)
    {
        GLColor color = getPlaceColor(place, attributes);

        color.set(0.75);
        glPushMatrix();
        glTranslatef(place.referenceFrame.x, place.referenceFrame.y, 0.0f);
        glRotatef(place.referenceFrame.theta * 180.0f / M_PI, 0.0f, 0.0f, 1.0f);

        if(place.metric)
        {
            math::Polygon<float> boundary(place.metric->extent().polygonBoundary());
            gl_draw_filled_polygon(boundary.vertices());
            color.set();
            gl_draw_line_polygon(boundary.vertices(), BORDER_WIDTH);

            starRenderer_->renderRelative(place.metric->star(), place.metric->center().toPoint());
        }
        else
        {
            gl_draw_filled_circle(Point<float>(0.0f, 0.0f), MIN_RADIUS, NUM_CIRCLE_VERTICES);
            color.set();
            gl_draw_line_circle(Point<float>(0.0f, 0.0f), MIN_RADIUS, BORDER_WIDTH, NUM_CIRCLE_VERTICES);
        }
        glPopMatrix();

        // Draw the covariance matrix as well -- always for now, but maybe sometimes in the future
        // TODO: Draw the Gaussian once a proper optimization with a correct covariance is implemented
//         gl_draw_gaussian_distribution(place.referenceFrame.uncertainty, 1, hazard_color());
    }
}

} // namespace ui
} // namespace vulcan

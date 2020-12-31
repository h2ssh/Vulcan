/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     exploration_map_renderer.cpp
* \author   Collin Johnson
*
* Definition of ExplorationMapRenderer.
*/

#include "ui/components/exploration_map_renderer.h"
#include "ui/common/default_colors.h"
#include "ui/common/gl_shapes.h"
#include "ui/common/ui_color.h"
#include "planner/exploration/local_topo/exploration_map.h"
#include <boost/range/iterator_range.hpp>

namespace vulcan
{
namespace ui
{

void render_local_area_target(const planner::LocalAreaTarget* area, int currentArea, int targetArea);
void draw_local_area_target(const planner::LocalAreaTarget* area, bool isTarget);

GLColor color_from_type(hssh::AreaType type);


void ExplorationMapRenderer::renderLocalTopoExplorationMap(const planner::LocalTopoExplorationMap& map,
                                                           int currentArea,
                                                           int targetArea)
{
    std::for_each(map.beginUnvisited(), map.endUnvisited(), [&](const planner::LocalAreaTarget* target) {
        render_local_area_target(target, currentArea, targetArea);
    });

    std::for_each(map.beginVisited(), map.endVisited(), [&](const planner::LocalAreaTarget* target) {
        render_local_area_target(target, currentArea, targetArea);
    });
}


void render_local_area_target(const planner::LocalAreaTarget* target, int currentArea, int targetArea)
{
    if(target->areaId() == targetArea)
    {
        draw_local_area_target(target, true);
    }
    else
    {
        draw_local_area_target(target, false);
    }

    // Draw the current area twice to make it darker on the screen
    if(target->areaId() == currentArea)
    {
        draw_local_area_target(target, false);
    }
}


void draw_local_area_target(const planner::LocalAreaTarget* area, bool isTarget)
{
    const float kBoundaryWidth = isTarget ? 10.0f : 2.0f;

    GLColor color = area->wasVisited() ? area_color() : color_from_type(area->areaType());

    color.set(0.5);
    gl_draw_filled_polygon(area->boundary().vertices());

    color.set();
    gl_draw_line_polygon(area->boundary().vertices(), kBoundaryWidth);
}


GLColor color_from_type(hssh::AreaType type)
{
    switch(type)
    {
    case hssh::AreaType::path_segment:
        return path_color();

    case hssh::AreaType::decision_point:
        return decision_point_color();

    case hssh::AreaType::destination:
        return destination_color();

    case hssh::AreaType::place:
    case hssh::AreaType::area:
    default:
        return area_color();
    }

    return area_color();
}

} // namespace ui
} // namespace vulcan

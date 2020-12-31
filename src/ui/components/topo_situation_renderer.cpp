/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file
* \author   Collin Johnson
*
* Definition of TopoSituationRenderer.
*/

#include "ui/components/topo_situation_renderer.h"
#include "ui/common/default_colors.h"
#include "ui/common/ui_color.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/areas/path_segment.h"
#include "mpepc/social/topo_situation.h"
#include "robot/model/params.h"
#include "utils/config_file.h"
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

using namespace mpepc;

GLColor bin_state_color(PathSituation::BinState state);
std::vector<Point<float>> points_along_boundary(Line<double> boundary,
                                                      double direction,
                                                      int numBins,
                                                      double robotRadius);


TopoSituationRenderer::TopoSituationRenderer(void)
: robotRadius_(0.3)
{
    utils::ConfigFile modelConfig(robot::kRobotModelConfigFilename);
    robot::collision_model_params_t params(modelConfig);

    // If there's a circle model radius to be loaded
    if(params.circleModelRadius > 0.0)
    {
        robotRadius_ = params.circleModelRadius;
    }
}


void TopoSituationRenderer::renderSituationPath(const PathSituation& situation,
                                                const hssh::LocalTopoMap& topoMap) const
{
    auto area = topoMap.areaWithId(situation.areaId());

    if(!area || (area->type() != hssh::AreaType::path_segment))
    {
        return;
    }

    auto path = std::static_pointer_cast<hssh::LocalPathSegment>(area);

    hssh::Gateway goal;
    hssh::Gateway start;

    // Find the ends
    if(path->plusTransition().gateway().id() == situation.gatewayId())
    {
        goal = path->plusTransition().gateway();
        start = path->minusTransition().gateway();
    }
    else if(path->minusTransition().gateway().id() == situation.gatewayId())
    {
        goal = path->minusTransition().gateway();
        start = path->plusTransition().gateway();
    }
    else
    {
        // Unknown gateway -- only dealing with the two endpoints right now in this visualization
        return;
    }

    auto bins = situation.situation();
    auto goalVertices = points_along_boundary(goal.boundary(), goal.direction(), bins.size(), robotRadius_);
    // Start line is inbound, so need to make sure direction (which is always outbound) gets flipped
    auto startVertices = points_along_boundary(start.boundary(), start.direction() + M_PI, bins.size(), robotRadius_);

    assert(goalVertices.size() == bins.size() + 1); // check for fencepost error

    const float kAlpha = 0.3f;

    glBegin(GL_QUADS);
    for(std::size_t n = 0; n < bins.size(); ++n)
    {
        auto color = bin_state_color(bins[n]);
        color.set(kAlpha);

        glVertex2f(goalVertices[n].x, goalVertices[n].y);
        glVertex2f(goalVertices[n + 1].x, goalVertices[n + 1].y);
        glVertex2f(startVertices[n + 1].x, startVertices[n + 1].y);
        glVertex2f(startVertices[n].x, startVertices[n].y);
    }
    glEnd();

    auto color = occupied_color();
    color.set(0.9);

    glLineWidth(1.0f);
    for(std::size_t n = 0; n < bins.size(); ++n)
    {
        glBegin(GL_LINE_LOOP);
        glVertex2f(goalVertices[n].x, goalVertices[n].y);
        glVertex2f(goalVertices[n + 1].x, goalVertices[n + 1].y);
        glVertex2f(startVertices[n + 1].x, startVertices[n + 1].y);
        glVertex2f(startVertices[n].x, startVertices[n].y);
        glEnd();
    }
}


void TopoSituationRenderer::renderSituationPlace(const PlaceSituation& situation,
                                                 const hssh::LocalTopoMap& topoMap) const
{
    auto place = topoMap.areaWithId(situation.areaId());

    if(!place)
    {
        return;
    }

    // TODO
}


GLColor bin_state_color(PathSituation::BinState state)
{
    switch(state)
    {
    case PathSituation::empty:
        return dynamic_color();
    case PathSituation::toward:
        return hazard_color();
    case PathSituation::away:
        return caution_color();
    case PathSituation::stationary:
        return quasi_static_color();
    }

    return GLColor();
}


std::vector<Point<float>> points_along_boundary(Line<double> boundary,
                                                      double direction,
                                                      int numBins,
                                                      double robotRadius)
{
    // If A isn't left of normal line in direction, then flip the boundary
    Line<double> normal;
    normal.a = center(boundary);
    normal.b = center(boundary);
    normal.b.x += std::cos(direction);
    normal.b.y += std::sin(direction);

    if(!left_of_line(normal, boundary.a))
    {
        std::swap(boundary.a, boundary.b);
    }

    std::vector<Point<float>> points;
    points.push_back(boundary.a);

    double orientation = angle_to_point(boundary.a, boundary.b);
    double len = length(boundary) - (2.0 * robotRadius);

    const double step = len / numBins;
    double nextPoint = robotRadius + step;

    for(int n = 0; n < numBins - 1; ++n)    // outsides of the bins accounted for, so only need interior points
    {
        Point<float> point(boundary.a.x + (nextPoint * std::cos(orientation)),
                                 boundary.a.y + (nextPoint * std::sin(orientation)));
        points.push_back(point);
        nextPoint += step;
    }

    points.push_back(boundary.b);
    return points;
}

} // namespace ui
} // namespace vulcan

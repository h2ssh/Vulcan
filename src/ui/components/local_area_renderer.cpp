/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     local_area_renderer.cpp
 * \author   Collin Johnson
 *
 * Definition of LocalAreaRenderer.
 */

#include "ui/components/local_area_renderer.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/area_detection/labeling/area_proposal.h"
#include "hssh/local_topological/areas/decision_point.h"
#include "hssh/local_topological/areas/destination.h"
#include "hssh/local_topological/areas/path_segment.h"
#include "hssh/local_topological/evaluation/heat_map.h"
#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "ui/common/gl_shapes.h"
#include "ui/common/hssh_colors.h"
#include "ui/components/area_extent_renderer.h"
#include "ui/components/gateways_renderer.h"
#include "ui/components/occupancy_grid_renderer.h"
#include "ui/components/small_scale_star_renderer.h"
#include <GL/gl.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <cassert>

namespace vulcan
{
namespace ui
{

const float kLineWidth = 2.0f;


struct LocalTopoGraphRenderer : public hssh::LocalAreaVisitor
{
    const hssh::LocalTopoMap& map;
    bool drawHSSHStyle;
    GLColor decisionColor;
    GLColor destColor;
    float decisionSize;
    float destSize;

    LocalTopoGraphRenderer(const hssh::LocalTopoMap& map, bool drawAsHSSH);

    // hssh::LocalAreaVisitor interface
    void visitDecisionPoint(const hssh::LocalDecisionPoint& decision) override;
    void visitDestination(const hssh::LocalDestination& destination) override;
    void visitPathSegment(const hssh::LocalPathSegment& path) override;
};


/////////////////// LocalAreaRender /////////////////////////////////
LocalAreaRenderer::LocalAreaRenderer(void)
: showGateways_(false)
, showStar_(false)
, extentRenderer_(std::make_unique<AreaExtentRenderer>())
, starRenderer_(std::make_unique<SmallScaleStarRenderer>())
, metersPerCell_(0.05)
{
    std::vector<GLColor> heatMapColors{GLColor{0, 0, 255, 255},
                                       //         GLColor{0, 255, 0, 255},
                                       GLColor{255, 0, 0, 255}};
    heatMapInterpolator_.setColors(heatMapColors);
}


LocalAreaRenderer::~LocalAreaRenderer(void)
{
    // For std::unique_ptr
}


void LocalAreaRenderer::renderLocalTopoMap(const hssh::LocalTopoMap& map)
{
    map.visitAreas(*this);
}


void LocalAreaRenderer::renderLocalTopoMapAsHeatMap(const hssh::LocalTopoMap& map,
                                                    const hssh::HeatMapStatistics& heatMap)
{
    // Calculate the min and max counts for the values, so the range for creating the [0,1] ranges can be found
    using namespace boost::accumulators;
    accumulator_set<int, stats<tag::min, tag::max>> visitAcc;
    for (auto& count : heatMap.areaVisitCount) {
        visitAcc(count.second);
    }

    int minVisits = min(visitAcc);
    double rangeVisits = max(visitAcc) - min(visitAcc);

    for (auto& area : map) {
        auto countIt = heatMap.areaVisitCount.find(area->id());
        if (countIt != heatMap.areaVisitCount.end()) {
            drawExtent(area->extent(),
                       heatMapInterpolator_.calculateColor((countIt->second - minVisits) / rangeVisits));
            drawGateways(area->gateways(), GLColor());
        }
    }
}


void LocalAreaRenderer::renderLocalTopoMapAsGraph(const hssh::LocalTopoMap& map)
{
    LocalTopoGraphRenderer renderer(map, false);
    map.visitAreas(renderer);
}


void LocalAreaRenderer::renderLocalTopoMapAsHSSHGraph(const hssh::LocalTopoMap& map)
{
    LocalTopoGraphRenderer renderer(map, true);
    map.visitAreas(renderer);
}


void LocalAreaRenderer::renderLocalArea(const hssh::LocalArea& area)
{
    area.accept(*this);
}


void LocalAreaRenderer::renderAreaProposal(const hssh::AreaProposal& area, const hssh::VoronoiSkeletonGrid& grid)
{
    drawExtent(area.getExtent(), color_from_local_area_type(area.getType()));
    drawGateways(area.getAllGateways(grid, math::ReferenceFrame::LOCAL), color_from_local_area_type(area.getType()));
    //     drawFrontiers(area.getFrontiers());
}


void LocalAreaRenderer::visitDecisionPoint(const hssh::LocalDecisionPoint& decision)
{
    drawExtent(decision.extent(), color_from_local_area_type(hssh::AreaType::decision_point));
    drawGateways(decision.gateways(), color_from_local_area_type(hssh::AreaType::decision_point));
    drawStar(decision.star(), decision.center().toPoint());

    //     glPushMatrix();
    //     glTranslatef(decision.center().x, decision.center().y, 0.0f);
    //     glRotatef(decision.center().theta * 180.0f / M_PI, 0.0f, 0.0f, 1.0f);
    //     OccupancyGridRenderer gridRenderer;
    //     gridRenderer.setGrid(decision.map());
    //     gridRenderer.renderGrid();
    //     glPopMatrix();
    //
    //     extentRenderer_.renderExtentRectangle(decision.extent(), GLColor(1.0f, 1.0f, 0.0f, 0.6f));
}


void LocalAreaRenderer::visitDestination(const hssh::LocalDestination& destination)
{
    drawExtent(destination.extent(), color_from_local_area_type(hssh::AreaType::destination));
    drawGateways(destination.gateways(), color_from_local_area_type(hssh::AreaType::destination));
    drawStar(destination.star(), destination.center().toPoint());
}


void LocalAreaRenderer::visitPathSegment(const hssh::LocalPathSegment& path)
{
    drawExtent(path.extent(), color_from_local_area_type(hssh::AreaType::path_segment));
    drawGateways(path.gateways(), color_from_local_area_type(hssh::AreaType::path_segment));
}


void LocalAreaRenderer::drawExtent(const hssh::AreaExtent& extent, const GLColor& color)
{
    extentRenderer_->renderExtentCells(extent, metersPerCell_, color);
}


void LocalAreaRenderer::drawGateways(const std::vector<hssh::Gateway>& gateways, const GLColor& color) const
{
    if (showGateways_) {
        GatewaysRenderer renderer;
        renderer.renderGateways(gateways, false);

        for (auto& gateway : gateways) {
            // Draw gateways separately so the gateway is drawn the correct color
            const float BOUNDARY_WIDTH = 3.0f;

            glLineWidth(BOUNDARY_WIDTH);
            glEnable(GL_LINE_STIPPLE);   // Draw boundary as a dashed line to make it pop out a bit
            glLineStipple(3, 0xAAAA);
            glBegin(GL_LINES);

            color.set();

            glVertex2f(gateway.center().x, gateway.center().y);
            glVertex2f(gateway.center().x + std::cos(gateway.direction()),
                       gateway.center().y + std::sin(gateway.direction()));

            glEnd();
            glDisable(GL_LINE_STIPPLE);
        }
    }
}


void LocalAreaRenderer::drawFrontiers(const std::vector<Point<double>>& frontiers) const
{
    for (auto frontier : frontiers) {
        frontier_color().set(0.33f);
        gl_draw_filled_circle(frontier, 0.2, 36);
        frontier_color().set();
        gl_draw_line_circle(frontier, 0.2, kLineWidth, 36);
    }
}


void LocalAreaRenderer::drawStar(const hssh::SmallScaleStar& star, const Point<double>& origin)
{
    if (showStar_) {
        starRenderer_->render(star, origin);
    }
}


/////////////////// LocalTopoGraphRenderer /////////////////////////////////
const float kGraphLineWidth = 3.0f;
const float kGraphPointSize = 10.0f;

LocalTopoGraphRenderer::LocalTopoGraphRenderer(const hssh::LocalTopoMap& map, bool drawAsHSSH)
: map(map)
, drawHSSHStyle(drawAsHSSH)
{
    if (drawHSSHStyle) {
        decisionColor = decision_point_color();
        destColor = decision_point_color();
        decisionSize = kGraphPointSize * 1.5;
        destSize = kGraphPointSize * 1.5;
    } else {
        decisionColor = decision_point_color();
        destColor = destination_color();
        decisionSize = kGraphPointSize * 2;
        destSize = kGraphPointSize;
    }
}


void LocalTopoGraphRenderer::visitDecisionPoint(const hssh::LocalDecisionPoint& decision)
{
    glLineWidth(kGraphLineWidth);
    glBegin(GL_LINES);

    for (auto adjAffordance : decision.adjacent()) {
        auto adj = map.otherSideOfGateway(adjAffordance.gateway(), decision);
        if (adj) {
            decisionColor.set(0.8);
            glVertex2f(decision.center().x, decision.center().y);

            if (adj->type() == hssh::AreaType::destination) {
                destColor.set(0.8);
            } else   // leading to a path
            {
                path_color().set(0.8);
            }

            glVertex2f(adjAffordance.target().x, adjAffordance.target().y);
        }
    }

    glEnd();

    decisionColor.set();
    Point<float> rectOffset(1.0f, 1.0f);
    gl_draw_filled_rectangle(
      math::Rectangle<float>(decision.center().toPoint() - rectOffset, decision.center().toPoint() + rectOffset));
}


void LocalTopoGraphRenderer::visitDestination(const hssh::LocalDestination& destination)
{
    // Draw line to each exit
    glLineWidth(kGraphLineWidth);
    glBegin(GL_LINES);

    for (auto adjAffordance : destination.adjacent()) {
        destColor.set(0.8);
        glVertex2f(destination.center().x, destination.center().y);
        glVertex2f(adjAffordance.target().x, adjAffordance.target().y);
    }

    glEnd();

    // Draw destinations as diamonds to make them visually distinct without color
    destColor.set();
    glPushMatrix();
    glTranslatef(destination.center().x, destination.center().y, 0.0f);
    if (!drawHSSHStyle) {
        glRotatef(45.0f, 0, 0, 1.0f);
    }
    Point<float> bl(-0.9f, -0.9f);
    Point<float> tr(0.9f, 0.9f);
    gl_draw_filled_rectangle(math::Rectangle<float>(bl, tr));
    glPopMatrix();
}


void LocalTopoGraphRenderer::visitPathSegment(const hssh::LocalPathSegment& path)
{
    Point<float> plusEnd = path.plusTransition().target().toPoint();
    Point<float> minusEnd = path.minusTransition().target().toPoint();

    Line<float> pathLine(plusEnd, minusEnd);

    glLineWidth(kGraphLineWidth);
    glBegin(GL_LINES);
    for (auto destAffordance : path.leftDestinations()) {
        auto destCenterProjection = closest_point_on_line_segment(destAffordance.target().toPoint(), pathLine);
        path_color().set(0.8);
        glVertex2f(destCenterProjection.x, destCenterProjection.y);
        destColor.set(0.8);
        glVertex2f(destAffordance.target().x, destAffordance.target().y);
    }

    for (auto destAffordance : path.rightDestinations()) {
        auto destCenterProjection = closest_point_on_line_segment(destAffordance.target().toPoint(), pathLine);
        path_color().set(0.8);
        glVertex2f(destCenterProjection.x, destCenterProjection.y);
        destColor.set(0.8);
        glVertex2f(destAffordance.target().x, destAffordance.target().y);
    }

    // Draw the line for the path itself
    path_color().set();
    glVertex2f(plusEnd.x, plusEnd.y);
    glVertex2f(minusEnd.x, minusEnd.y);
    glEnd();

    // Draw the decision points for the dest-path intersections
    if (drawHSSHStyle) {
        destColor.set(0.8);

        for (auto destAffordance : path.leftDestinations()) {
            auto dest = map.otherSideOfGateway(destAffordance.gateway(), path);
            if (dest) {
                auto destCenterProjection = closest_point_on_line_segment(destAffordance.target().toPoint(), pathLine);
                Point<float> rectOffset(1.0f, 1.0f);
                gl_draw_filled_rectangle(
                  math::Rectangle<float>(destCenterProjection - rectOffset, destCenterProjection + rectOffset));
            }
        }

        for (auto destAffordance : path.rightDestinations()) {
            auto dest = map.otherSideOfGateway(destAffordance.gateway(), path);
            if (dest) {
                auto destCenterProjection = closest_point_on_line_segment(destAffordance.target().toPoint(), pathLine);
                Point<float> rectOffset(1.0f, 1.0f);
                gl_draw_filled_rectangle(
                  math::Rectangle<float>(destCenterProjection - rectOffset, destCenterProjection + rectOffset));
            }
        }
    }
}

}   // namespace ui
}   // namespace vulcan

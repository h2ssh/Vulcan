/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     glass_map_renderer.cpp
* \author   Collin Johnson
*
* Implementation of GlassMapRenderer.
*/

#include <ui/components/glass_map_renderer.h>
#include <ui/components/cell_grid_renderer.h>
#include <ui/components/occupancy_grid_renderer.h>
#include <ui/common/color_generator.h>
#include <ui/common/default_colors.h>
#include <ui/common/gl_arrays_helpers.h>
#include <ui/common/gl_shapes.h>
#include <hssh/types.h>
#include <hssh/metrical/mapping/glass_map_utils.h>
#include <hssh/metrical/mapping/glass_walls.h>
#include <boost/range/iterator_range.hpp>
#include <GL/gl.h>
#include <cassert>
#include <cmath>

namespace vulcan
{
namespace ui
{

GlassMapRenderer::GlassMapRenderer(void)
: mapRenderer_(std::make_unique<OccupancyGridRenderer>())
, intensityRenderer_(std::make_unique<CellGridAlphaRenderer>(8000))
, anglesToDraw(GlassAnglesToDraw::normal)
{
    setRenderColors(occupied_color(), dynamic_color(), quasi_static_color());
}


GlassMapRenderer::~GlassMapRenderer(void)
{
    // For std::unique_ptr
}


void GlassMapRenderer::setRenderColors(const GLColor& occupiedColor,
                                       const GLColor& motionColor,
                                       const GLColor& angleColor)
{
    this->occupiedColor = occupiedColor;
    this->motionColor   = motionColor;
    this->angleColor    = angleColor;

    mapRenderer_->setOccupiedColor(occupiedColor);
    mapRenderer_->setDynamicColor(motionColor);
}


void GlassMapRenderer::setGlassMap(hssh::GlassMap& map)
{
    // Only load the glass angles if they are being drawn because it is a slow operation
    if(shouldDrawAngles)
    {
        createMapAngles(map);
    }

    const auto& flattened = map.flattenedMap();
    mapRenderer_->setGrid(flattened);

    intensityRenderer_->setCellGrid(map.intensityMap());

    initialized = true;
    activeBoundary_ = map.activeRegionInMeters();
}


void GlassMapRenderer::renderGrid(void)
{
    if(!initialized)
    {
        return;
    }

    if(shouldShowIntensity)
    {
        intensityRenderer_->renderGrid(true);
    }
    else
    {
        mapRenderer_->renderGrid();
    }

    angleColor.set();
    gl_draw_line_rectangle(activeBoundary_);

    if(shouldDrawAngles)
    {
        drawAngles();
    }
}


void GlassMapRenderer::renderAngleBins(int x, int y, hssh::GlassMap& map)
{
    // Ignore any cells that aren't in the map
    if(!map.isCellInGrid(x, y))
    {
        return;
    }

    // Recenter the active region so the cell is loaded
    if(!map.isCellInActiveRegion(x, y))
    {
        map.recenterActiveRegion(utils::grid_point_to_global_point(Point<float>(x, y), map));
        activeBoundary_ = map.activeRegionInMeters();
    }

    // To draw the bins along the entire bottom of the window, change the view matrices back to their defaults. The
    // defaults set the screen coordinates to be (-1, -1) to (1, 1). Thus we can draw a rectangle of width 2 to go
    // across the entire bottom of the screen

    const Point<float> origin(-1.0f, -1.0f);
    const float width = 2.0f;
    const float height = 2.0f;

    glMatrixMode (GL_MODELVIEW);
    glPushMatrix ();
    glLoadIdentity ();
    glMatrixMode (GL_PROJECTION);
    glPushMatrix ();
    glLoadIdentity ();

    float binWidth = width / map.numAngleBins();
    float binHeight = height / 20.0f;  // draw the bins as 1/20 of the screen

    assert(map.beginBin(x, y) != map.endBin(x, y));

    glBegin(GL_QUADS);
    for(int n = 0; n < map.numAngleBins(); ++n)
    {
        auto bin = *(map.beginBin(x, y) + n);
        // Set the color to gray, white, or black, depending on the odds. Don't have intermediate values because each
        // bin is usually hit only a few times
        float color = std::min(0.5 + 2.0 * std::abs(bin / 127.0), 1.0);

        // Draw hits as red
        if(bin > 0)
        {
            glColor4f(color, 0.0, 0.0, 1.0f);
        }
        // Draw misses as blue
        else if(bin < 0)
        {
            glColor4f(0.0, 0.0, color, 1.0f);
        }
        // Use white for unknown
        else
        {
            glColor4f(1.0, 1.0, 1.0, 1.0f);
        }


        glVertex2f(origin.x + n*binWidth, origin.y);
        glVertex2f(origin.x + n*binWidth, origin.y + binHeight);
        glVertex2f(origin.x + (n+1)*binWidth, origin.y + binHeight);
        glVertex2f(origin.x + (n+1)*binWidth, origin.y);
    }
    glEnd();

    // Draw a black border around the angle bins to make them visually distinct from the map
    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
    glLineWidth(3.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(origin.x, origin.y);
    glVertex2f(origin.x + width, origin.y);
    glVertex2f(origin.x + width, origin.y + binHeight);
    glVertex2f(origin.x, origin.y + binHeight);
    glEnd();

    glPopMatrix ();
    glMatrixMode (GL_MODELVIEW);
    glPopMatrix ();
}


void GlassMapRenderer::renderWalls(const std::vector<hssh::GlassWall>& walls)
{
    auto colors = generate_colors(4, 0.75f);

    glLineWidth(3.0f);
    glBegin(GL_LINES);
    for(std::size_t n = 0; n < walls.size(); ++n)
    {
        colors[n % 4].set();
        glVertex2f(walls[n].wall().a.x, walls[n].wall().a.y);
        glVertex2f(walls[n].wall().b.x, walls[n].wall().b.y);
    }
    glEnd();

    glPointSize(2.0f);
    glBegin(GL_POINTS);
    for(std::size_t n = 0; n < walls.size(); ++n)
    {
        colors[n % 4].set();
        for(auto p : walls[n])
        {
            glVertex2f(p.x, p.y);
        }
    }
    glEnd();

    for(std::size_t n = 0; n < walls.size(); ++n)
    {
        colors[n % 4].set();
        auto center = walls[n].wall().a + walls[n].wall().b;
        center.x /= 2;
        center.y /= 2;
        double lineNormal = direction(normal(walls[n].wall()));
        if(angle_diff_abs(lineNormal, walls[n].normal()) > M_PI_2)
        {
            lineNormal += M_PI;
        }
        gl_draw_small_arrow(center, 0.5f, lineNormal);
        gl_draw_medium_arrow(center, 0.5f, walls[n].normal());
    }
}


void GlassMapRenderer::drawAngles(void)
{
    switch(anglesToDraw)
    {
    case GlassAnglesToDraw::normal:
        drawNormals();
        break;

    case GlassAnglesToDraw::range:
        drawRanges();
        break;
    };
}


void GlassMapRenderer::drawNormals(void)
{
    if(!angleLineVertices_.empty())
    {
        angleColor.set();
        glLineWidth(0.5f);
        draw_gl_array(angleLineVertices_.data(), 2, angleLineVertices_.size()/2, GL_LINES);
    }
}


void GlassMapRenderer::drawRanges(void)
{
    if(!angleRangeVertices_.empty())
    {
        angleColor.set();
        draw_gl_array(angleRangeVertices_.data(), 2, angleRangeVertices_.size()/2, GL_TRIANGLES);

        // Draw the normals on top of the range to make their direction easier to see
        auto pushColor = angleColor;
        angleColor = GLColor(0, 0, 0, 255);
        drawNormals();
        angleColor = pushColor;
    }
}


void GlassMapRenderer::createMapAngles(hssh::GlassMap& glass)
{
    // NOTE: The GlassMap and flattened map aren't guaranteed to have the same bottom left corner location
    const auto& flattened = glass.flattenedMap();
    auto glassToFlatOffset = glass.glassToFlatMapOffset();

    angleLineVertices_.clear();
    angleRangeVertices_.clear();

    hssh::CellSet visited;

    // For each valid range, create two lines pointing from center of cell along start and start+range directions
    Point<double> center;

    for(std::size_t y = 0; y < glass.getHeightInCells(); ++y)
    {
        for(std::size_t x = 0; x < glass.getWidthInCells(); ++x)
        {
            // Ignore all visited cells and non-hit cells
            if((visited.find(hssh::cell_t(x, y)) != visited.end()))
            {
                continue;
            }

            center = utils::grid_point_to_global_point(Point<int>(x, y), glass);

            if(!glass.isCellInActiveRegion(x, y))
            {
                center.x += 3.0;
                center.y += 3.0;
                glass.recenterActiveRegion(center);
                activeBoundary_ = glass.activeRegionInMeters();
            }

            auto activeRegion = glass.activeRegionInCells();
            hssh::cell_t region;

            for(region.y = 0; region.y < activeRegion.height(); ++region.y)
            {
                for(region.x = 0; region.x < activeRegion.width(); ++region.x)
                {
                    hssh::cell_t cell = region + activeRegion.bottomLeft;

                    if(visited.find(cell) != visited.end())
                    {
                        continue;
                    }

                    visited.insert(cell);

                    // Ignore non-hit cells
                    if(flattened.getCost(cell + glassToFlatOffset) < 255)
                    {
                        continue;
                    }

                    center = utils::grid_point_to_global_point(cell, glass);
                    center.x += glass.metersPerCell() / 2.0;  // center the normal in the middle of the cell
                    center.y += glass.metersPerCell() / 2.0;

                    addNormal(cell, center, glass);
                    addRange(cell, center, glass);
                }
            }
        }
    }
}


void GlassMapRenderer::addNormal(Point<int> cell, Point<double> cellCenter, const hssh::GlassMap& map)
{
    double normalLength = map.metersPerCell() / 2.0;
    double normal = glass_cell_normal(cell.x, cell.y, map);

    angleLineVertices_.push_back(cellCenter.x);
    angleLineVertices_.push_back(cellCenter.y);
    angleLineVertices_.push_back(cellCenter.x + std::cos(normal)*normalLength);
    angleLineVertices_.push_back(cellCenter.y + std::sin(normal)*normalLength);
}


void GlassMapRenderer::addRange(Point<int> cell, Point<double> cellCenter, const hssh::GlassMap& map)
{
    double normalLength = map.metersPerCell() / 2.0;
    auto range = hssh::angle_bins_to_angle_range(cell.x, cell.y, map);

    angleRangeVertices_.push_back(cellCenter.x);
    angleRangeVertices_.push_back(cellCenter.y);
    // Draw the start and end of the range
    angleRangeVertices_.push_back(cellCenter.x + std::cos(range.start)*normalLength);
    angleRangeVertices_.push_back(cellCenter.y + std::sin(range.start)*normalLength);

    angleRangeVertices_.push_back(cellCenter.x + std::cos(range.start + range.extent)*normalLength);
    angleRangeVertices_.push_back(cellCenter.y + std::sin(range.start + range.extent)*normalLength);
}

} // namespace ui
} // namespace vulcan

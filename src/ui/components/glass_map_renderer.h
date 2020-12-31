/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     glass_map_renderer.h
* \author   Collin Johnson
*
* Definition of GlassMapRenderer.
*/

#ifndef UI_COMPONENTS_GLASS_MAP_RENDERER_H
#define UI_COMPONENTS_GLASS_MAP_RENDERER_H

#include "ui/common/ui_color.h"
#include "math/geometry/rectangle.h"
#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh { class GlassWall; }
namespace hssh { class GlassMap; }
namespace ui
{

class OccupancyGridRenderer;
class CellGridAlphaRenderer;

enum class GlassAnglesToDraw
{
    normal,
    range,
};

/**
* GlassMapRenderer draws the GlassMap. The glass map contains two types of data -- an occupancy grid and visible angle ranges for each
* occupied cell in the occupancy grid. The occupancy data is drawn in two colors. One color is used for cells that are definitely
* occupied. The other color is used for cells that are estimated to be from motion. The angle data is optional. The angle ranges are
* drawn using two lines to indicate the extrema of the range.
*
* The angle data is only calculated if the angles are being shown via the showAngles() method. Consequently, whenever the angles to draw
* are changed, then setGlassMap needs to be called again.
*/
class GlassMapRenderer
{
public:

    GlassMapRenderer(void);

    ~GlassMapRenderer(void);

    void setRenderColors(const GLColor& occupiedColor,
                         const GLColor& motionColor,
                         const GLColor& angleColor);

    /**
    * setGlassMap handles drawing of the glass map to the screen.
    *
    * NOTE: The glass map isn't passed by const-reference. The reason is the active region needs to be moved around
    * in order to generate the visualization of the glass map, and that operation is not const.
    *
    * \param    map         Glass map to draw
    */
    void setGlassMap(hssh::GlassMap& map);
    void showIntensity(bool show) { shouldShowIntensity = show; }
    void setAnglesToDraw(GlassAnglesToDraw angles) { anglesToDraw = angles; }
    void showAngles(bool show) { shouldDrawAngles = show; }

    void renderGrid(void);

    /**
    * renderAngleBins draws the angle bins for the provided cell along the bottom of the viewport.
    */
    void renderAngleBins(int x, int y, hssh::GlassMap& map);

    /**
    * renderWalls draws GlassWalls on the map.
    */
    void renderWalls(const std::vector<hssh::GlassWall>& walls);

private:

    std::unique_ptr<OccupancyGridRenderer> mapRenderer_;
    std::unique_ptr<CellGridAlphaRenderer> intensityRenderer_;
    math::Rectangle<float> activeBoundary_;

    bool shouldShowIntensity = false;
    bool shouldDrawAngles = false;
    GlassAnglesToDraw anglesToDraw;

    GLColor occupiedColor;
    GLColor motionColor;
    GLColor angleColor;

    std::vector<float> angleLineVertices_;
    std::vector<float> angleRangeVertices_;

    bool initialized = false;

    void drawAngles(void);
    void drawNormals(void);
    void drawRanges(void);
    void createMapAngles(hssh::GlassMap& glass);
    void addNormal(Point<int> cell, Point<double> cellCenter, const hssh::GlassMap& map);
    void addRange(Point<int> cell, Point<double> cellCenter, const hssh::GlassMap& map);
};

} // namespace ui
} // namespace vulcan

#endif // UI_COMPONENTS_GLASS_MAP_RENDERER_H

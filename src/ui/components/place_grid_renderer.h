/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UI_COMPONENTS_PLACE_GRID_RENDERER_H
#define UI_COMPONENTS_PLACE_GRID_RENDERER_H

#include "core/point.h"
#include "ui/common/ui_color.h"
#include <cstdint>
#include <vector>

namespace vulcan
{
namespace hssh
{
class VoronoiSkeletonGrid;
}
namespace ui
{

/**
 * VoronoiSkeletonGridRenderer renders a VoronoiSkeletonGrid using OpenGL. The cell classifications are
 * each drawn in a separate, configurable color.
 *
 * Occupied cells are always drawn in black. Unknown cells are always drawn in gray.
 *
 * The VoronoiSkeletonGridRenderer contains two textures, an obstacle distance texture and a special
 * cell texture. The obstacle distance texture will change the alpha values of a cell based
 * on its distance from the nearest obstacle. The special cell texture will change the color
 * of a cell based on its classification as determined by the skeleton builder.
 */
class VoronoiSkeletonGridRenderer
{
public:
    /**
     * Constructor for VoronoiSkeletonGridRenderer.
     */
    VoronoiSkeletonGridRenderer(void);

    /**
     * Destructor for VoronoiSkeletonGridRenderer.
     */
    ~VoronoiSkeletonGridRenderer(void);

    /**
     * setRenderColors sets the colors to be used for rendering the grid. The currently configurable
     * colors are for frontiers, gatetways, and skeletons.
     */
    void setRenderColors(const GLColor& frontierColor, const GLColor& skeletonColor, const GLColor& reducedColor);

    /**
     * setGrid sets the grid that is to be rendered. This method is a costly operation, as it recreates the
     * texture used to draw the grid. As such, the method should only be called when a new grid has arrived.
     * Otherwise, set the grid and then just call renderGrid() many times until a new place arrives.
     */
    void setGrid(const hssh::VoronoiSkeletonGrid& grid);

    /**
     * setCellColor sets the color for a cell. The color applies only to the current grid. As soon as the
     * grid is changed, the color will be wiped out.
     *
     * \param    cell        Cell for the color
     * \param    color       Color to set for the cell
     */
    void setCellColor(const Point<uint16_t>& cell, const GLColor& color);

    /**
     * resetCellColor resets the cell color to whatever it would be for the current grid.
     *
     * \param    cell                Cell to reset
     * \param    classification      Classification of the cell for determining the color
     */
    void resetCellColor(const Point<uint16_t>& cell, uint8_t classification);

    /**
     * renderGrid draws the grid onto the screen.
     */
    void renderGrid(void);

    // Flags for which pieces of the place grid should be shown
    void showDistances(bool show);
    void showFullSkeleton(bool show);
    void showReducedSkeleton(bool show);
    void showFrontiers(bool show);

private:
    VoronoiSkeletonGridRenderer(const VoronoiSkeletonGridRenderer& toCopy) = delete;
    void operator=(const VoronoiSkeletonGridRenderer& rhs) = delete;

    void initializeGridTextures(void);
    void updateGridTextures(const hssh::VoronoiSkeletonGrid& grid);
    void enableTextures(void);
    void disableTextures(void);

    void convertClassificationGridToTexture(const hssh::VoronoiSkeletonGrid& grid);
    void setCellClassificationColor(int x, int y, uint8_t classification);
    void convertDistanceGridToTexture(const hssh::VoronoiSkeletonGrid& grid);

    // Flags indicating which parts of the grid should be rendered
    bool renderDistances;
    bool renderSpecialCells;   // special cells are those cells labeled by a skeleton builder
    bool renderFullSkeleton;
    bool renderReducedSkeleton;
    bool renderFrontiers;

    bool classificationTextureChanged;

    // Storing in this fashion to avoid unnecessary multiplications to get unsigned byte range
    // for the different cell colors. Can't just do the memcpy of the whole grid buffer straight into
    // the texture buffer unfortunately
    uint8_t freeColor[4];
    uint8_t occupiedColor[4];
    uint8_t unknownColor[4];
    uint8_t frontierColor[4];
    uint8_t skeletonColor[4];
    uint8_t reducedColor[4];
    uint8_t defaultColor[4];

    uint16_t gridWidth;
    uint16_t gridHeight;

    float metricWidth;
    float metricHeight;

    Point<float> gridOrigin;

    uint16_t textureWidth;
    uint16_t textureHeight;

    unsigned int* textureNames;
    uint16_t* distanceTexture;
    uint8_t* classificationTexture;

    bool initialized;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_PLACE_GRID_RENDERER_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "ui/components/place_grid_renderer.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "ui/common/gl_texture_helpers.h"
#include <cassert>

namespace vulcan
{
namespace ui
{

const int NUM_TEXTURES = 2;
const int CLASSIFICATION_INDEX = 0;
const int DISTANCE_INDEX = 1;


void convert_color_to_array(const GLColor& color, uint8_t colorArray[4]);

inline void set_texture_cell_color(uint8_t* texture, const uint8_t color[4], int startIndex)
{
    // Unrolling the more obvious loop implementation
    texture[startIndex] = color[0];
    texture[startIndex + 1] = color[1];
    texture[startIndex + 2] = color[2];
    //     texture[startIndex+3] = color[3];
}

/*
 * All throughout the below code, there are flags that change between doing a single texturing pass or a multiple
 * texturing pass. The flags for the display control the information displayed. Usually both textures will be used
 * but on occasion the user might only want to see a subset of the overall information.
 */

VoronoiSkeletonGridRenderer::VoronoiSkeletonGridRenderer(void)
: gridWidth(0)
, gridHeight(0)
, textureWidth(0)
, textureHeight(0)
, textureNames(0)
, distanceTexture(0)
, classificationTexture(0)
, initialized(false)
{
    freeColor[0] = 255;
    freeColor[1] = 255;
    freeColor[2] = 255;
    freeColor[3] = 255;

    occupiedColor[0] = 0;
    occupiedColor[1] = 0;
    occupiedColor[2] = 0;
    occupiedColor[3] = 255;

    unknownColor[0] = 127;
    unknownColor[1] = 127;
    unknownColor[2] = 127;
    unknownColor[3] = 255;

    // Want to not change the color at all for lots of cell
    defaultColor[0] = 0;
    defaultColor[1] = 0;
    defaultColor[2] = 0;
    defaultColor[3] = 0;
}


VoronoiSkeletonGridRenderer::~VoronoiSkeletonGridRenderer(void)
{
    delete[] distanceTexture;
    delete[] classificationTexture;
    delete[] textureNames;
}


void VoronoiSkeletonGridRenderer::setRenderColors(const GLColor& frontierColor,
                                                  const GLColor& skeletonColor,
                                                  const GLColor& reducedColor)
{
    convert_color_to_array(frontierColor, this->frontierColor);
    convert_color_to_array(skeletonColor, this->skeletonColor);
    convert_color_to_array(reducedColor, this->reducedColor);
}


void VoronoiSkeletonGridRenderer::setGrid(const hssh::VoronoiSkeletonGrid& grid)
{
    if (textureNames == 0) {
        textureNames = new GLuint[NUM_TEXTURES];

        glGenTextures(NUM_TEXTURES, textureNames);
    }

    if (gridWidth != grid.getWidthInCells() || gridHeight != grid.getHeightInCells()) {
        if (gridWidth > 0 || gridHeight > 0) {
            delete[] distanceTexture;
            delete[] classificationTexture;
        }

        gridWidth = grid.getWidthInCells();
        gridHeight = grid.getHeightInCells();
        metricWidth = grid.getWidthInMeters();
        metricHeight = grid.getHeightInMeters();

        initializeGridTextures();

        initialized = true;
    }

    gridOrigin = grid.getBottomLeft();
    updateGridTextures(grid);

    classificationTextureChanged = false;
}


void VoronoiSkeletonGridRenderer::setCellColor(const Point<uint16_t>& cell, const GLColor& color)
{
    if (classificationTexture && (cell.x < gridWidth) && (cell.y < gridHeight)) {
        uint8_t colorArray[4];
        convert_color_to_array(color, colorArray);

        uint32_t index = (cell.y * gridWidth + cell.x) * 3;
        set_texture_cell_color(classificationTexture, colorArray, index);

        classificationTextureChanged = true;
    }
}


void VoronoiSkeletonGridRenderer::resetCellColor(const Point<uint16_t>& cell, uint8_t classification)
{
    if (classificationTexture && (cell.x < gridWidth) && (cell.y < gridHeight)) {
        setCellClassificationColor(cell.x, cell.y, classification);

        classificationTextureChanged = true;
    }
}


void VoronoiSkeletonGridRenderer::renderGrid(void)
{
    if (initialized) {
        if (classificationTextureChanged) {
            activate_texture(textureNames[CLASSIFICATION_INDEX], GL_TEXTURE1, GL_DECAL);
            set_sub_texture(classificationTexture, gridWidth, gridHeight, GL_RGB);
            disable_texture(GL_TEXTURE1);
        }

        enableTextures();

        float textureXMax = gridWidth / static_cast<float>(textureWidth);
        float textureYMax = gridHeight / static_cast<float>(textureHeight);

        glColor4f(0.0f, 0.0f, 0.0f, 0.8f);
        if (renderDistances && renderSpecialCells) {
            draw_two_textures_on_rectangle(gridOrigin.x,
                                           gridOrigin.y,
                                           metricWidth,
                                           metricHeight,
                                           textureXMax,
                                           textureYMax);
        } else {
            draw_one_texture_on_rectangle(gridOrigin.x,
                                          gridOrigin.y,
                                          metricWidth,
                                          metricHeight,
                                          textureXMax,
                                          textureYMax);
        }

        disableTextures();

        classificationTextureChanged = false;
    }
}


void VoronoiSkeletonGridRenderer::showDistances(bool show)
{
    renderDistances = show;
}


void VoronoiSkeletonGridRenderer::showFullSkeleton(bool show)
{
    renderFullSkeleton = show;

    renderSpecialCells = renderFullSkeleton | renderReducedSkeleton | renderFrontiers;
}


void VoronoiSkeletonGridRenderer::showReducedSkeleton(bool show)
{
    renderReducedSkeleton = show;

    renderSpecialCells = renderFullSkeleton | renderReducedSkeleton | renderFrontiers;
}


void VoronoiSkeletonGridRenderer::showFrontiers(bool show)
{
    renderFrontiers = show;

    renderSpecialCells = renderFullSkeleton | renderReducedSkeleton | renderFrontiers;
}


void VoronoiSkeletonGridRenderer::initializeGridTextures(void)
{
    textureWidth = round_to_power_of_two(gridWidth);
    textureHeight = round_to_power_of_two(gridHeight);

    classificationTexture = create_texture(textureWidth, textureHeight, 3);
    distanceTexture = new uint16_t[textureWidth * textureHeight];

    initialize_texture(textureNames[CLASSIFICATION_INDEX],
                       classificationTexture,
                       textureWidth,
                       textureHeight,
                       GL_RGB,
                       GL_RGB);
    initialize_texture_16(textureNames[DISTANCE_INDEX],
                          distanceTexture,
                          textureWidth,
                          textureHeight,
                          GL_ALPHA,
                          GL_ALPHA);
}


void VoronoiSkeletonGridRenderer::updateGridTextures(const hssh::VoronoiSkeletonGrid& grid)
{
    activate_texture(textureNames[CLASSIFICATION_INDEX], GL_TEXTURE1, GL_DECAL);
    convertClassificationGridToTexture(grid);
    set_sub_texture(classificationTexture, gridWidth, gridHeight, GL_RGB);
    disable_texture(GL_TEXTURE1);

    activate_texture(textureNames[DISTANCE_INDEX], GL_TEXTURE0, GL_REPLACE);
    convertDistanceGridToTexture(grid);
    set_sub_texture_16(distanceTexture, gridWidth, gridHeight, GL_ALPHA);
    disable_texture(GL_TEXTURE0);
}


void VoronoiSkeletonGridRenderer::enableTextures(void)
{
    if (renderDistances && renderSpecialCells) {
        activate_texture(textureNames[CLASSIFICATION_INDEX], GL_TEXTURE1, GL_DECAL);
        activate_texture(textureNames[DISTANCE_INDEX], GL_TEXTURE0, GL_REPLACE);
    } else if (renderDistances) {
        activate_texture(textureNames[DISTANCE_INDEX], GL_TEXTURE0, GL_REPLACE);
    } else {
        activate_texture(textureNames[CLASSIFICATION_INDEX], GL_TEXTURE0, GL_DECAL);
    }
}


void VoronoiSkeletonGridRenderer::disableTextures(void)
{
    if (renderDistances && renderSpecialCells) {
        disable_texture(GL_TEXTURE1);
        disable_texture(GL_TEXTURE0);
    } else {
        disable_texture(GL_TEXTURE0);
    }
}


void VoronoiSkeletonGridRenderer::convertClassificationGridToTexture(const hssh::VoronoiSkeletonGrid& grid)
{
    assert(gridHeight == grid.getHeightInCells());
    assert(gridWidth == grid.getWidthInCells());

    for (int y = 0; y < gridHeight; ++y) {
        for (int x = 0; x < gridWidth; ++x) {
            setCellClassificationColor(x, y, grid.getClassification(x, y));
        }
    }
}


void VoronoiSkeletonGridRenderer::setCellClassificationColor(int x, int y, uint8_t classification)
{
    int textureIndex = (y * gridWidth + x) * 3;

    if (classification & hssh::SKELETON_CELL_REDUCED_SKELETON && renderReducedSkeleton) {
        set_texture_cell_color(classificationTexture, reducedColor, textureIndex);
    } else if (classification & hssh::SKELETON_CELL_SKELETON && renderFullSkeleton) {
        set_texture_cell_color(classificationTexture, skeletonColor, textureIndex);
    } else if (classification & hssh::SKELETON_CELL_FRONTIER && renderFrontiers) {
        set_texture_cell_color(classificationTexture, frontierColor, textureIndex);
    }
    // Only draw the free/occupied/unknown space if the distance gradient is not being shown
    // because these colors will cause the gradient to be rendered incorrectly
    else if (classification & hssh::SKELETON_CELL_FREE && !renderDistances) {
        set_texture_cell_color(classificationTexture, freeColor, textureIndex);
    } else if (classification & hssh::SKELETON_CELL_OCCUPIED && !renderDistances) {
        set_texture_cell_color(classificationTexture, occupiedColor, textureIndex);
    } else if (classification & hssh::SKELETON_CELL_UNKNOWN && !renderDistances) {
        set_texture_cell_color(classificationTexture, unknownColor, textureIndex);
    } else {
        set_texture_cell_color(classificationTexture, defaultColor, textureIndex);
    }
}


void VoronoiSkeletonGridRenderer::convertDistanceGridToTexture(const hssh::VoronoiSkeletonGrid& grid)
{
    assert(gridHeight == grid.getHeightInCells());
    assert(gridWidth == grid.getWidthInCells());

    int textureIndex = 0;

    int maximumDistance = grid.getMaximumDistance();
    maximumDistance = 1000;

    float scale = 65536.0f / maximumDistance;

    for (int y = 0; y < gridHeight; ++y) {
        for (int x = 0; x < gridWidth; ++x) {
            uint16_t value = grid.getObstacleDistance(x, y);

            distanceTexture[textureIndex++] = (value < maximumDistance) ? value * scale : 65536;
        }
    }
}


void convert_color_to_array(const GLColor& color, uint8_t colorArray[4])
{
    colorArray[0] = static_cast<uint8_t>(color.red() * 255);
    colorArray[1] = static_cast<uint8_t>(color.green() * 255);
    colorArray[2] = static_cast<uint8_t>(color.blue() * 255);
    colorArray[3] = static_cast<uint8_t>(color.alpha() * 255);
}

}   // namespace ui
}   // namespace vulcan

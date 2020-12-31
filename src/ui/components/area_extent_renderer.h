/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     area_extent_renderer.h
 * \author   Collin Johnson
 *
 * Declaration of AreaExtentRenderer.
 */

#ifndef UI_COMPONENTS_AREA_EXTENT_RENDERER_H
#define UI_COMPONENTS_AREA_EXTENT_RENDERER_H

#include <vector>

namespace vulcan
{
namespace hssh
{
class AreaExtent;
}
namespace ui
{

class GLColor;

/**
 * AreaExtentRenderer
 */
class AreaExtentRenderer
{
public:
    /**
     * renderExtentCells draws the extent using the contained cells.
     */
    void renderExtentCells(const hssh::AreaExtent& extent, float metersPerCell, const GLColor& color) const;

    /**
     * renderExtentRectangle draws the extent using the rough rectangle boundary.
     */
    void renderExtentRectangle(const hssh::AreaExtent& extent, const GLColor& color) const;

    /**
     * renderExtentPolygon draws the extent using the convex hull polygon of the extent.
     */
    void renderExtentPolygon(const hssh::AreaExtent& extent, const GLColor& color) const;

private:
    mutable std::vector<float> extentVertices_;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_AREA_EXTENT_RENDERER_H

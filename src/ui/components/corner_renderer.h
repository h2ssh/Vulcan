/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     corner_renderer.h
* \author   Jong Jin Park
*
* Declaration of PoseTargetRenderer.
*/

#ifndef UI_COMPONENTS_CORNER_RENDERER_H
#define UI_COMPONENTS_CORNER_RENDERER_H

#include <vector>
#include <ui/common/ui_color.h>
#include <core/line.h>
// #include <ui/common/gl_shapes.h>

namespace vulcan
{
    
namespace ui
{

/**
* CornerRenderer renders a set of lines and circles that represents a corner.
*/
class CornerRenderer
{
public:
    
    /**
    * setRenderColor and set ShapeWidths sets the color and widths of the shapes to be rendered.
    */
    void setRenderColors(const GLColor& hoverPointColor,
                         const GLColor& cornerPointColor,
                         const GLColor& cornerLineColor,
                         const GLColor& rectifiedCornerColor);

    void setShapeWidths (float cornerLineWidth,
                         float rectifiedCornerLineWidth,
                         float circleRadius);
    
    /**
    * renderCornerSelction renders currently selected corner points and a hover point.
    * renderCornerPoints renders selected corner points.
    * renderRectifiedCorner renders a right angled corner.
    */
    void renderCornerSelection(const std::vector<Point<float>>& cornerPoints, const Point<float>& hoverPoint);
    void renderCornerPoints(const std::vector<Point<float>>& cornerPoints);
//    void renderRectifiedCorner(const mpepc::right_angled_corner_t& rightAngledCorner);

private:
    
    void renderFilledCircle(const Point<float>& center, float radius, const GLColor& color); // NOTE: There should be a Scatter function which include this.
    void renderLine(const Point<float>& pointA, const Point<float>& pointB, float lineWidth, const GLColor& color); // NOTE: This should get merged with LinesRenderer
    void renderLine(const Line<float>& line, float lineWidth, const GLColor& color);
    
    float cornerLineWidth;
    float rectifiedCornerLineWidth;
    float circleRadius;
    float hoverCircleRadius;
    
    GLColor hoverPointColor;
    GLColor cornerPointColor;
    GLColor cornerLineColor;
    GLColor rectifiedCornerColor;
};

} // ui
} // vulcan

#endif // UI_COMPONENTS_CORNER_RENDERER_H

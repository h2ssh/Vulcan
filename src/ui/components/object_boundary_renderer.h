/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     object_boundary_renderer.h
 * \author   Collin Johnson
 *
 * Declaration of ObjectBoundaryRenderer boost::static_visitor subclass for drawing tracker::ObjectBoundary.
 */

#ifndef UI_COMPONENTS_OBJECT_BOUNDARY_RENDERER_H
#define UI_COMPONENTS_OBJECT_BOUNDARY_RENDERER_H

#include "tracker/object_boundary.h"
#include <boost/variant/static_visitor.hpp>

namespace vulcan
{
namespace ui
{

class GLColor;

/**
 * ObjectBoundaryRenderer draws an ObjectBoundary with a line outline and a filled in center. The filled center has an
 * alpha that is 0.3 of the alpha for the outline.
 *
 *   ObjectBoundary b;
 *   b.visitShape(ObjectBoundaryRenderer(boundaryColor))
 */
class ObjectBoundaryRenderer : public boost::static_visitor<>
{
public:
    /**
     * Constructor for ObjectBoundaryRenderer.
     *
     * \param    color           Color to draw the boundary
     */
    ObjectBoundaryRenderer(const GLColor& color);

    // Operators to handle the drawing for the various possible shapes of the boundary
    void operator()(const tracker::Rectangle& rectangle);
    void operator()(const tracker::Circle& circle);
    void operator()(const tracker::TwoCircles& twoCircles);
    void operator()(const tracker::CircleRect& circleRect);
    void operator()(const tracker::TwoRects& rects);

private:
    const GLColor& color_;
};

/**
 * OutlineObjectBoundaryRenderer draws just an outline for the shape.
 *
 * The usage is:
 *
 *   ObjectBoundary b;
 *   b.visitShape(OutlineObjectBoundaryRenderer())
 */
class OutlineObjectBoundaryRenderer : public boost::static_visitor<>
{
public:
    /**
     * Constructor for OutlineObjectBoundaryRenderer.
     *
     * \param    color           Color to draw the boundary
     */
    OutlineObjectBoundaryRenderer(const GLColor& color);

    // Operators to handle the drawing for the various possible shapes of the boundary
    void operator()(const tracker::Rectangle& rectangle);
    void operator()(const tracker::Circle& circle);
    void operator()(const tracker::TwoCircles& twoCircles);
    void operator()(const tracker::CircleRect& circleRect);
    void operator()(const tracker::TwoRects& rects);

private:
    const GLColor& color_;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_OBJECT_BOUNDARY_RENDERER_H

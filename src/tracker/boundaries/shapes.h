/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     shapes.h
* \author   Collin Johnson
*
* Definition of all valid shapes for boundaries.
* Definition of support functions for valid BoundaryShape types.
*/

#ifndef TRACKER_BOUNDARIES_SHAPES_H
#define TRACKER_BOUNDARIES_SHAPES_H

#include "tracker/types.h"
#include "math/geometry/arc.h"
#include "math/geometry/circle.h"
#include "math/geometry/rectangle.h"
#include <array>

namespace vulcan
{
namespace tracker
{

/**
* BoundaryShape defines a simple concept for all shapes used as boundaries in the tracker.
*
* A valid shape supports the following free-functions:
*
*   - center : the center of the shape
*/

using Rectangle  = math::Rectangle<float>;
using Circle     = math::Circle<float>;
using TwoCircles = std::array<Circle, 2>;
using TwoRects   = std::array<Rectangle, 2>;
using Arc        = math::Arc<float>;
using TwoArcs    = std::array<Arc, 2>;

struct CircleRect
{
    Circle    circle;
    Rectangle rect;

    // Serialization support
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( circle,
            rect);
    }
};


// Shape center free functions for BoundaryShape concept
template <typename T>
Position center(const math::Shape<T>& shape)
{
    return shape.center();
}

template <typename Shape>
Position center(const std::array<Shape, 2>& shape)
{
    Position c = shape[0].center() + shape[1].center();
    c.x /= 2;
    c.y /= 2;
    return c;
}

inline Position center(const CircleRect& shape)
{
    Position c = shape.circle.center() + shape.rect.center();
    c.x /= 2;
    c.y /= 2;
    return c;
}

}
}

#endif // TRACKER_BOUNDARIES_SHAPES_H

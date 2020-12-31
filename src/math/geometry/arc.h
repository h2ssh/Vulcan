/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     arc.h
 * \author   Collin Johnson
 *
 * Definition of Arc.
 */

#ifndef MATH_GEOMETRY_ARC_H
#define MATH_GEOMETRY_ARC_H

#include "core/point.h"
#include "math/angle_range.h"
#include "math/geometry/circle.h"
#include <cereal/access.hpp>

namespace vulcan
{
namespace math
{

/**
 * Arc is a continuous portion of a circle. It represents some subset of a circle. The arc has a radius, center, and
 * angle range through which it sweeps.
 */
template <typename T>
class Arc
{
public:
    /**
     * Default constructor for Arc.
     */
    Arc(void) : radius_(1) { }

    /**
     * Constructor for Arc.
     */
    Arc(T radius, const Point<T>& center, const angle_range_t& range) : radius_(radius), center_(center), range_(range)
    {
    }

    /**
     * radius retrieves the radius of the arc.
     */
    T radius(void) const { return radius_; }

    /**
     * center retrieves the center of the arc.
     */
    Point<T> center(void) const { return center_; }

    /**
     * range retrieves the angle range through which the arc sweeps.
     */
    angle_range_t range(void) const { return range_; }

    /**
     * length retrieves the length of the arc along the circle.
     */
    double length(void) const { return range_.extent * radius_; }

    /**
     * toCircle retrieves the circle of which this arc is a segment.
     */
    Circle<T> toCircle(void) const { return Circle<T>(radius_, center_); }

private:
    T radius_;
    Point<T> center_;
    angle_range_t range_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(radius_, center_, range_);
    }
};

}   // namespace math
}   // namespace vulcan

#endif   // MATH_GEOMETRY_ARC_H

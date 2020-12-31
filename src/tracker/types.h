/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     types.h
 * \author   Collin Johnson
 *
 * Definition of utility types used in the object_tracker:
 *
 *   - velocity_t
 *   - Position
 *   - TwoCircles
 *   - ConstPointIter
 */

#ifndef TRACKER_TYPES_H
#define TRACKER_TYPES_H

#include "core/matrix.h"
#include "core/point.h"
#include "math/geometry/arc.h"
#include "math/geometry/circle.h"
#include "math/geometry/rectangle.h"
#include <deque>

namespace vulcan
{
namespace tracker
{

struct object_state_t;

/**
 * ObjectId is a unique id assigned to each object.
 */
using ObjectId = int32_t;

/**
 * Position describes the position of an object as a point with x and y coordinates.
 */
using Position = Point<float>;

/**
 * Endpoints describes the endpoints of a shape.
 */
using Endpoints = std::array<Position, 2>;

/**
 * velocity_t describes the motion of object along the x- and y-axis. The
 */
struct velocity_t
{
    float x;
    float y;

    explicit velocity_t(float x = 0.0f, float y = 0.0f) : x(x), y(y) { }
};

const std::size_t kRecentTrajectoryLength = 250;

using ObjectTrajectory = std::deque<object_state_t>;
using TrajConstIter = ObjectTrajectory::const_iterator;

using PositionHistory = std::deque<Point<float>>;
using HistoryConstIter = PositionHistory::const_iterator;
using ConstPointIter = std::vector<Point<float>>::const_iterator;

/**
 * EstimatedShape stores the shape estimated within a LaserObject.
 */
template <class Shape>
struct EstimatedShape
{
    Shape shape;
    double error;
    Matrix uncertainty;

    // Serialization support
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(shape, error, uncertainty);
    }
};

/**
 * Possible boundary types for objects.
 */
enum class BoundaryType
{
    rectangle,
    one_circle,
    two_circles,
    two_rects,
    circle_rect,
    unknown,
    best,
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_TYPES_H

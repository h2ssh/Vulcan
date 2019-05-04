/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file
* \author   Collin Johnson
*
* Declaration of various utility type aliasing for working with points. Included are a hash function
* and support for using with Boost.Geometry.
*/

#ifndef CORE_POINT_UTIL_H
#define CORE_POINT_UTIL_H

#include <core/point.h>

#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Traits support for use of Point with Boost.Geometry -- see bottom of file
#include <boost/mpl/int.hpp>
#include <boost/static_assert.hpp>
#include <boost/geometry/core/access.hpp>
#include <boost/geometry/core/coordinate_type.hpp>
#include <boost/geometry/core/coordinate_system.hpp>
#include <boost/geometry/core/coordinate_dimension.hpp>
#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/register/point.hpp>

namespace vulcan
{

/**
* PointHash is a simple hash function for a Point that uses the following hash function:
*
*   h(point) = point.x*xSpan + point.y
*
* The hash will be perfect when dealing with a grid if xSpan > grid.width
*/
template <typename T>
struct PointHash
{
    int xSpan = 1000000;

    std::size_t operator()(const Point<T>& point) const { return point.x*xSpan + point.y; }
};


template <typename T, typename Type>
using PointToTypeMap = std::unordered_map<Point<T>, Type, PointHash<T>>;

template <typename T>
using PointSet = std::unordered_set<Point<T>, PointHash<T>>;

template <typename T>
using PointVec = std::vector<Point<T>>;

} // namespace vulcan

BOOST_GEOMETRY_REGISTER_POINT_2D(vulcan::Point<double>, double, cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_POINT_2D(vulcan::Point<float>, float, cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_POINT_2D(vulcan::Point<int>, int, cs::cartesian, x, y)

#endif // CORE_POINT_UTIL_H

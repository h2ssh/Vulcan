/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef MATH_GEOMETRY_CONVEX_HULL_H
#define MATH_GEOMETRY_CONVEX_HULL_H

#include "math/geometry/polygon.h"
#include "core/line.h"
#include <algorithm>
#include <vector>
#include <cassert>
#include <cstdint>

namespace vulcan
{
namespace math
{

/**
* convex_hull finds the convex hull of a set of points using the Melkman algorithm.
*
* The convex_hull is a the polygon that of the shapes that contains all points in the provided
* set.
*
* There must be at least three points for the convex hull to be valid, otherwise it will immediately
* fail with no result because there is no polygon that can be made.
*
* \param    points          Points from which to find the convex hull
* \return   Polygon with vertices representing the convex hull in counter-clockwise order.
*/
template <typename T>
Polygon<T> convex_hull(typename std::vector<Point<T>>::const_iterator begin,
                       typename std::vector<Point<T>>::const_iterator end)
{
    /*
    * Using the Monotone chain algorithm.
    */

    std::vector<Point<T>> uniquePoints(begin, end);

    std::sort(uniquePoints.begin(), uniquePoints.end());
    auto endUniqueIt = std::unique(uniquePoints.begin(), uniquePoints.end());
    uniquePoints.resize(std::distance(uniquePoints.begin(), endUniqueIt));

    if(uniquePoints.size() < 3)
    {
        return Polygon<T>();
    }

    std::vector<Point<T>> upperHull     = {uniquePoints[0], uniquePoints[1]};
    upperHull.reserve(uniquePoints.size());
    std::size_t           upperHullSize = 2;

    // Do some swapping here to push the "erased" elements to the end of the vector -- takes more memory, but saves the copying from erasing
    for(std::size_t n = 2; n < uniquePoints.size(); ++n)
    {
        if(upperHull.size() == upperHullSize)
        {
            upperHull.push_back(uniquePoints[n]);
            ++upperHullSize;
        }
        else
        {
            upperHull[upperHullSize++] = uniquePoints[n];
        }

        while(upperHullSize > 2u && left_of_line(upperHull[upperHullSize-3], upperHull[upperHullSize-2], upperHull[upperHullSize-1]))
        {
            std::swap(upperHull[upperHullSize-2], upperHull[upperHullSize-1]);
            --upperHullSize;
        }
    }

    std::vector<Point<T>> lowerHull     = {uniquePoints.back(), uniquePoints[uniquePoints.size()-2]};
    lowerHull.reserve(uniquePoints.size());
    std::size_t           lowerHullSize = 2;

    for(int n = uniquePoints.size() - 2; --n >= 0;)
    {
        if(lowerHull.size() == lowerHullSize)
        {
            lowerHull.push_back(uniquePoints[n]);
            ++lowerHullSize;
        }
        else
        {
            lowerHull[lowerHullSize++] = uniquePoints[n];
        }

        while(lowerHullSize > 2u && left_of_line(lowerHull[lowerHullSize-3],
            lowerHull[lowerHullSize-2],
            lowerHull[lowerHullSize-1]))
        {
            std::swap(lowerHull[lowerHullSize-2], lowerHull[lowerHullSize-1]);
            --lowerHullSize;
        }
    }

    upperHull.resize(upperHullSize);
    // Copy everything but the first and last points -- where the two hulls meet
    std::copy(lowerHull.begin() + 1, lowerHull.begin() + lowerHullSize - 1, std::back_inserter(upperHull));

    // Ensure the first and last vertices are the same to enforce the polygon invariant
    if(upperHull.front() != upperHull.back())
    {
        upperHull.push_back(upperHull.front());
    }

    std::reverse(upperHull.begin(), upperHull.end());
    return Polygon<T>(upperHull);
}

} // namespace math
} // namespace vulcan

#endif // MATH_CONVEX_HULL_H

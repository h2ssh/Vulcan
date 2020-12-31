/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef MATH_GEOMETRY_LINE_INTERSECTIONS_H
#define MATH_GEOMETRY_LINE_INTERSECTIONS_H

#include <vector>
#include "core/line.h"
#include "core/point.h"

namespace vulcan
{
namespace math
{

/**
* intersection_point_t contains information about an intersection found between
* at least two lines.
*/
struct intersection_point_t
{
    Point<float>              intersection;      ///< Point where the lines intersected
    std::vector<unsigned int> indices;           ///< Indices in the input vector of all the intersecting lines (size >= 2)
};

/**
* find_line_intersections finds all intersections between lines in a set. The intersections are found using the
* Bentley-Ottmann algorithm as described in Section 2.1 of Computational Geometry by de Berg et. al. The algorithm
* runs in O(n*lg(n) + k*lg(n)) time, where n is the number of lines and k is the number of intersections.
*
* This runtime is not quite the theoretical lower bound of O(n*lg(n) + k). The naive approach is O(n^2), so the
* Bentley-Ottmann algorithm should be an improvement.
*/
std::vector<intersection_point_t> find_line_intersections(const std::vector<Line<double>>& lines);

}
}

#endif // MATH_GEOMETRY_LINE_INTERSECTIONS_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_POINT_ARRAY_T_H
#define LCMTYPES_POINT_ARRAY_T_H

#include "lcmtypes/vulcan_lcm_point_array_t.h"
#include "core/point.h"
#include <type_traits>
#include <vector>

namespace vulcan
{
namespace lcm
{

// Helper
template <typename message_t>
void allocate_point_array_message(message_t& pointsMessage)
{
    using point_t = typename std::remove_reference<decltype(*pointsMessage.points)>::type;
    pointsMessage.points = new point_t[pointsMessage.num_points];
}

// Interface
template <typename message_t, typename T>
void convert_lcm_to_vulcan(const message_t& pointsMessage, std::vector<Point<T>>& points)
{
    points.resize(pointsMessage.num_points);
    
    for(int i = points.size(); --i >= 0;)
    {
        points[i].x = pointsMessage.points[i].x;
        points[i].y = pointsMessage.points[i].y;
    }
}

template <typename message_t>
auto convert_lcm_to_vulcan(const message_t& pointsMessage) -> std::vector<Point<decltype(pointsMessage.points[0].x)>>
{
    using T = decltype(pointsMessage.points[0].x);
    std::vector<Point<T>> points;
    convert_lcm_to_vulcan(pointsMessage, points);
    return points;
}


template <typename T, typename message_t>
void convert_vulcan_to_lcm(const std::vector<Point<T>>& points, message_t& pointsMessage)
{
    pointsMessage.num_points = points.size();
    
    allocate_point_array_message(pointsMessage);
    
    for(int i = 0; i < pointsMessage.num_points; ++i)
    {
        pointsMessage.points[i].x = points[i].x;
        pointsMessage.points[i].y = points[i].y;
    }
}


template <typename point_t, typename message_t, class PointIter>
void convert_vulcan_to_lcm(PointIter begin, PointIter end, message_t& pointsMessage)
{
    pointsMessage.num_points = std::distance(begin, end);
    
    allocate_point_array_message(pointsMessage);
    
    for(int i = 0; i < pointsMessage.num_points; ++i, ++begin)
    {
        pointsMessage.points[i].x = begin->x;
        pointsMessage.points[i].y = begin->y;
    }
}


template <typename message_t>
void free_point_array_message(message_t& pointsMessage)
{
    delete [] pointsMessage.points;
}

}
}

#endif // LCMTYPES_POINT_ARRAY_T_H

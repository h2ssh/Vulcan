/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_COMMON_POINT_T_H
#define LCMTYPES_COMMON_POINT_T_H

#include <core/point.h>

namespace vulcan
{
namespace lcm
{

template <typename PointMessage, typename T>
PointMessage convert_point_to_lcm(const Point<T>& point)
{
    PointMessage pointMessage;
    
    pointMessage.x = point.x;
    pointMessage.y = point.y;
    
    return pointMessage;
}

template <typename PointMessage>
auto convert_lcm_to_point(const PointMessage& pointMessage) -> Point<decltype(pointMessage.x)>
{
    using T = decltype(pointMessage.x);

    return Point<T>(pointMessage.x, pointMessage.y);
}

}
}

#endif // LCMTYPES_COMMON_POINT_T_H

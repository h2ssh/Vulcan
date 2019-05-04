/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_COMMON_CIRCLE_T_H
#define LCMTYPES_COMMON_CIRCLE_T_H

#include <lcmtypes/vulcan_lcm_circle_t.h>
#include <lcmtypes/common/point_t.h>
#include <math/geometry/circle.h>

namespace vulcan
{
namespace lcm
{

template <typename CircleMessage, typename T>
void convert_circle_to_lcm(const math::Circle<T>& circle, CircleMessage& circleMessage)
{
    circleMessage.center = convert_point_to_lcm<decltype(circleMessage.center)>(circle.center());
    circleMessage.radius = circle.radius();
}


template <typename CircleMessage>
auto convert_lcm_to_circle(const CircleMessage& circleMessage) -> math::Circle<decltype(circleMessage.center.x)>
{
    using T = decltype(circleMessage.center.x);
    return math::Circle<T>(circleMessage.radius, convert_lcm_to_point(circleMessage.center));
}

}
}

#endif // LCMTYPES_COMMON_CIRCLE_T_H

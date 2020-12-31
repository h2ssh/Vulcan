/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_COMMON_RECTANGLE_T_H
#define LCMTYPES_COMMON_RECTANGLE_T_H

#include "lcmtypes/vulcan_lcm_rectangle_t.h"
#include "lcmtypes/common/point_t.h"
#include "math/geometry/rectangle.h"

namespace vulcan
{
namespace lcm
{

template <typename RectMessage, typename T>
void convert_rectangle_to_lcm(const math::Rectangle<T>& rect, RectMessage& rectMessage)
{
    using LCMPoint = decltype(rectMessage.bottom_left);
    
    rectMessage.bottom_left  = convert_point_to_lcm<LCMPoint>(rect.bottomLeft);
    rectMessage.bottom_right = convert_point_to_lcm<LCMPoint>(rect.bottomRight);
    rectMessage.top_left     = convert_point_to_lcm<LCMPoint>(rect.topLeft);
    rectMessage.top_right    = convert_point_to_lcm<LCMPoint>(rect.topRight);
}


template <typename RectMessage>
auto convert_lcm_to_rectangle(const RectMessage& rectMessage) -> math::Rectangle<decltype(rectMessage.bottom_left.x)>
{
    using T = decltype(rectMessage.bottom_left.x);
    
    return math::Rectangle<T>(convert_lcm_to_point(rectMessage.top_left),
                              convert_lcm_to_point(rectMessage.top_right),
                              convert_lcm_to_point(rectMessage.bottom_right),
                              convert_lcm_to_point(rectMessage.bottom_left));
}

}
}

#endif // LCMTYPES_COMMON_RECTANGLE_T_H

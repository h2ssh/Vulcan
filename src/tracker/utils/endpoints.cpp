/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     endpoints.cpp
* \author   Collin Johnson
* 
* Definition of utility functions:
* 
*   - major_axis_endpoints
*   - two_circles_endpoints
*/

#include "tracker/utils/endpoints.h"
#include <cmath>

namespace vulcan
{
namespace tracker
{

Position circle_endpoint(const Circle&    circle, float direction);
Position rect_endpoint  (const Rectangle& rect,   float direction);


Endpoints major_axis_endpoints(const Rectangle& rect)
{
    // The endpoints of the object that are tracked are those where the line formed by the major axis intersects
    // the boundary of the rectangle
    auto adjustedRect = math::make_major_axis_along_bottom(rect);
    
    auto leftEndpoint = adjustedRect.bottomLeft + adjustedRect.topLeft;
    leftEndpoint.x /= 2;
    leftEndpoint.y /= 2;
    
    auto rightEndpoint = adjustedRect.bottomRight + adjustedRect.topRight;
    rightEndpoint.x /= 2;
    rightEndpoint.y /= 2;

    return {leftEndpoint, rightEndpoint};
}


Endpoints circle_rect_endpoints(const CircleRect& circleRect)
{
//     float axisDirection = angle_to_point(circleRect.circle.center(), circleRect.rect.center());
//     return {circle_endpoint(circleRect.circle, axisDirection + M_PI), rect_endpoint(circleRect.rect, axisDirection)};
    return {circleRect.circle.center(), circleRect.rect.center()};
}


Endpoints two_rects_endpoints(const TwoRects& rects)
{
//     float axisDirection = angle_to_point(rects[0].center(), rects[1].center());
//     return {rect_endpoint(rects[0], axisDirection + M_PI), rect_endpoint(rects[1], axisDirection)};
    return {rects[0].center(), rects[1].center()};
}


Endpoints two_circles_endpoints(const TwoCircles& circles)
{
    // The endpoints are the intersection of the line connecting the circle centers with the outside boundary of
    // each circle
//     float axisDirection = angle_to_point(circles[0].center(), circles[1].center());
//     return {circle_endpoint(circles[0], axisDirection + M_PI), circle_endpoint(circles[1], axisDirection)};
    return {circles[0].center(), circles[1].center()};
}


Position circle_endpoint(const Circle& circle, float direction)
{
    Position zeroEndpoint = circle.center();
    zeroEndpoint.x += std::cos(direction) * circle.radius();
    zeroEndpoint.y += std::sin(direction) * circle.radius();
    return zeroEndpoint;
}


Position rect_endpoint(const Rectangle& rect, float direction)
{
    std::vector<Position> intersections;
    Position outsideRectPosition = rect.center();
    outsideRectPosition.x += std::cos(direction) * rect.height() * 1000.0f;
    outsideRectPosition.y += std::sin(direction) * rect.height() * 1000.0f;
    rect.intersections(Line<float>(rect.center(), outsideRectPosition), intersections);

    assert(intersections.size() == 1);

    return intersections.front();
}

} // namespace tracker
} // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     shape_intersection_area.cpp
* \author   Collin Johnson
* 
* Definition of various functions to find the area of the intersection between shapes. Currently supported shapes are:
* 
*   - circle_rectangle_intersection_area
*   - circle_circle_intersection_area
*/


#include "math/geometry/shape_intersection_area.h"
#include <cassert>

namespace vulcan 
{
namespace math
{
    
///////////////////////////////// circle_rectangle_intersection_area //////////////////////////////////
float three_corner_intersection(const Circle<float>& circle, const Rectangle<float>& rectangle);
float two_corner_intersection  (const Circle<float>& circle, const Rectangle<float>& rectangle);
float one_corner_intersection  (const Circle<float>& circle, const Rectangle<float>& rectangle);
float zero_corner_intersection (const Circle<float>& circle, const Rectangle<float>& rectangle);

float circle_rectangle_intersection_area(const Circle<float>& circle, const Rectangle<float>& rectangle)
{
    assert(circle.radius() > 0.0f);
    
    // First check if the circle and rectangle are disjoint
    // If the center of the circle is further from the rectangle than the radius, they can't overlap
    if(rectangle.distanceToPoint(circle.center()) > circle.radius())
    {
        return 0.0f;
    }
    
    // The algorithm splits into five contains based on the number of corners of the rectangle that fall inside
    // the circle
    int blInCircle = circle.contains(rectangle.bottomLeft)  ? 1 : 0;
    int brInCircle = circle.contains(rectangle.bottomRight) ? 1 : 0;
    int tlInCircle = circle.contains(rectangle.topLeft)     ? 1 : 0;
    int trInCircle = circle.contains(rectangle.topRight)    ? 1 : 0;
    
    int totalInCircle = blInCircle + brInCircle + tlInCircle + trInCircle;
    
    // If all four are in the circle, the intersection is the entire rectangle
    if(totalInCircle == 4)
    {
        return rectangle.area();
    }
    else if(totalInCircle == 3)
    {
        return three_corner_intersection(circle, rectangle);
    }
    else if(totalInCircle == 2)
    {
        return two_corner_intersection(circle, rectangle);
    }
    else if(totalInCircle == 1)
    {
        return one_corner_intersection(circle, rectangle);
    }
    else  // if(totalInCircle == 0)
    {
        return zero_corner_intersection(circle, rectangle);
    }
}


float three_corner_intersection(const Circle<float>& circle, const Rectangle<float>& rectangle)
{
    // TODO
    auto squareApprox = Rectangle<float>(Point<float>(circle.center().x - circle.radius(), 
                                                      circle.center().y - circle.radius()),
                                         Point<float>(circle.center().x + circle.radius(), 
                                                      circle.center().y + circle.radius()));
    return squareApprox.intersection(rectangle).area();
}


float two_corner_intersection(const Circle<float>& circle, const Rectangle<float>& rectangle)
{
    // TODO
    auto squareApprox = Rectangle<float>(Point<float>(circle.center().x - circle.radius(), 
                                                      circle.center().y - circle.radius()),
                                         Point<float>(circle.center().x + circle.radius(), 
                                                      circle.center().y + circle.radius()));
    return squareApprox.intersection(rectangle).area();
}


float one_corner_intersection(const Circle<float>& circle, const Rectangle<float>& rectangle)
{
    // TODO
    auto squareApprox = Rectangle<float>(Point<float>(circle.center().x - circle.radius(), 
                                                      circle.center().y - circle.radius()),
                                         Point<float>(circle.center().x + circle.radius(), 
                                                      circle.center().y + circle.radius()));
    return squareApprox.intersection(rectangle).area();
}


float zero_corner_intersection(const Circle<float>& circle, const Rectangle<float>& rectangle)
{
    // TODO
    auto squareApprox = Rectangle<float>(Point<float>(circle.center().x - circle.radius(), 
                                                      circle.center().y - circle.radius()),
                                         Point<float>(circle.center().x + circle.radius(), 
                                                      circle.center().y + circle.radius()));
    return squareApprox.intersection(rectangle).area();
}


///////////////////////////////// circle_circle_intersection_area //////////////////////////////////

float circle_segment_area(float radius, float chordDist);


float circle_circle_intersection_area(const Circle<float>& lhs, const Circle<float>& rhs)
{
    assert(lhs.radius() > 0.0f);
    assert(rhs.radius() > 0.0f);
    
    // Calculate the distance between the circle centers.
    float centerDistance = distance_between_points(lhs.center(), rhs.center());
    
    // If the distance is >= lhs.radius + rhs.radius then there's no overlap because the circles are too far apart
    if(centerDistance >= lhs.radius() + rhs.radius())
    {
        return 0.0f;
    }
    
    // If the distance is zero, then one circle is inside the other circle, so the intersection area is the smaller
    // of the two areas
    if(centerDistance == 0.0f)
    {
        return std::min(lhs.area(), rhs.area());
    }
    
    // Otherwise, the circles intersect at two points.
    // The key is to think of the intersection nibbling off a circle segment for each circle. The trick is to find the
    // chord for the circle, which then can be used to find the area of the circle segment
    
    // The math is too hard to write without pictures and ASCII art would be a pain, so here's a link that describes
    // essentially what I'm doing, though it only deals with circles with the same radius:
    //
    //      http://jwilson.coe.uga.edu/EMAT6680Su12/Carreras/EMAT6690/Essay2/essay2.html
    //
    // To handle circles of different radius, distribute the centerDistance by taking the appropriate proportion, which
    // is proportional to the ratio of the radii,  d_1 = d*r_1 / (r_1 + r_2)   d_2 = d*r_2 / (r_1 + r_2)
    
    float lhsChordDist = centerDistance * lhs.radius() / (lhs.radius() + rhs.radius());
    float rhsChordDist = centerDistance * rhs.radius() / (lhs.radius() + rhs.radius());
    
    return circle_segment_area(lhs.radius(), lhsChordDist) + circle_segment_area(rhs.radius(), rhsChordDist);
}


float circle_segment_area(float radius, float chordDist)
{
    assert(radius >= chordDist);   // acos undefined outside of [-1,1]
    
    // The circle segment is the area of the sector of the circle, minus the area the of the triangle formed by the
    // endpoints of the chord and the center of the circle
    // Using trig, this boils down to some equations with an isoceles triangle
    float sectorArea   = radius * radius * std::acos(chordDist / radius);
    float triangleArea = chordDist * std::sqrt(radius*radius - chordDist*chordDist);
    
    return sectorArea - triangleArea;
}

} // namespace math
} // namespace vulcan

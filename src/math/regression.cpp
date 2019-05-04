/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <math/regression.h>

// #define DEBUG_TLS

namespace vulcan
{
namespace math
{


Line<float> simple_least_squares(std::vector<Point<float>>::const_iterator begin, std::vector<Point<float>>::const_iterator end)
{
    /**
    * For this least squares calculation, use the formula described at: http://mathworld.wolfram.com/LeastSquaresFitting.html
    *
    *   a = sum(x)
    *   b = sum(x^2)
    *   c = sum(y)
    *   d = sum(y * x)
    *   e = size(end - begin)
    *
    *   D = d / b
    *   C = c / e - b * a / e
    */

    #ifdef DEBUG_SLS
    std::cout<<"SLS: Start: "<<*begin<<" End: "<<*(end - 1)<<std::endl;
    #endif

    // Define the variables above, then do a straight-forward calculation of the values
    double a = 0;
    double b = 0;
    double c = 0;
    double d = 0;
    int    e = 0;

    std::pair<double, double> result;

    for(std::vector<Point<float>>::const_iterator pointIt = begin; pointIt != end; ++pointIt)
    {
        a += pointIt->x;
        b += pointIt->x * pointIt->x;
        c += pointIt->y;
        d += pointIt->x * pointIt->y;
        ++e;
    }

    // Only calculate the line if there are at least two points. Otherwise just return end empty line.
    if(e > 1)
    {
        result.first  = d / b;
        result.second = c / e - b * a / e;

        #ifdef DEBUG_SLS
        std::cout<<"SLS: y = "<<result.second<<" + "<<result.first<<"x"<<std::endl;
        #endif

        Line<float> line;

        line.a.x = begin->x;
        line.a.y = result.first + result.second*begin->x;
        line.b.x = (end-1)->x;
        line.b.y = result.first + result.second*(end-1)->x;

        return line;
    }
    else
    {
        return Line<float>();
    }
}

// Helper functions
Line<double> line_from_least_squares(Point<double> p1,
                                     Point<double> p2,
                                     double xAverage,
                                     double yIntercept,
                                     double slope)
{
    /*
    * The line can be determined using the in the following way:
    *
    *   x = x
    *   y = slope*(x - p.x) + p.y
    */

    Line<float> l;

    // For all lines, have the second x or y coordinate be a shifted version of the first to
    // ensure that the two endpoints are actually different when the calculation occurs
    if(slope == 0)
    {
        // Slope is zero, so line should just run from p1.x to p2.x at height point.y
        l.a = Point<double>(p1.x,       yIntercept);
        l.b = Point<double>(p1.x + 2.0, yIntercept);
    }
    else if(slope == INFINITY)
    {
        // Slope is infinite, so run from p1.y to p2.y at width of point.x
        l.a = Point<double>(xAverage, p1.y);
        l.b = Point<double>(xAverage, p1.y + 2.0);
    }
    else
    {
        // The other line segment should be
        l.a = Point<double>(p1.x,     slope*p1.x       + yIntercept);
        l.b = Point<double>(p1.x+2.0, slope*(p1.x+2.0) + yIntercept);
    }

    // Project the two points onto the completed line because lines with big slopes will
    // usually overshoot the point on each end, which then casts the line into areas of
    // the world that clearly have no line. The projection here creates nice, crisp corners
    return Line<double>(closest_point_on_line(p1, l),
                        closest_point_on_line(p2, l));
}


double dist_to_line(Point<double> point, double a, double b)
{
    /*
    * Distance from point to line is given by the following equation:
    *
    * dist = abs(point.y - (a + b*point.x)) / sqrt(1 + b^2)
    *
    */

    return fabs(point.y - (a + b*point.x)) / sqrt(1 + b*b);
}

} // namespace math
} // namespace vulcan

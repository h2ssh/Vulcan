/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     circle_fitting.cpp
* \author   Collin Johnson
* 
* Implementation of minimum_geometric_error_circle.
*/

#include <math/geometry/shape_fitting.h>
#include <math/geometry/algebraic_circle_fitting.h>
#include <math/geometry/geometric_circle_fitting.h>
#include <core/float_comparison.h>
#include <cassert>

namespace vulcan
{
namespace math
{

using namespace detail;
    

Circle<float> minimum_geometric_error_circle(DataIterator pointsBegin, 
                                             DataIterator pointsEnd, 
                                             const double minRadius, 
                                             const double maxRadius,
                                             double*      fitError)
{
    assert(std::distance(pointsBegin, pointsEnd) > 2);
    
    bool haveDifferentX = false;
    bool haveDifferentY = false;
    // Before fitting, validate that the data aren't a vertical or horizontal line
    for(auto pIt = pointsBegin, nIt = pointsBegin+1; nIt != pointsEnd; ++pIt, ++nIt)
    {
        if(!absolute_fuzzy_equal(pIt->x, nIt->x))
        {
            haveDifferentX = true;
        }
        
        if(!absolute_fuzzy_equal(pIt->y, nIt->y))
        {
            haveDifferentY = true;
        }
        
        if(haveDifferentX && haveDifferentY)
        {
            break;
        }
    }
    
    if(!haveDifferentX || !haveDifferentY)
    {
        std::cerr << "ERROR: minimum_geometric_error_circle: Trying to fit a circle to a straight line. Returning a"
                  << " mean-fit circle.\n";
        return mean_position_and_radius_circle(pointsBegin, pointsEnd);
    }
    
    Circle<float> fitCircle;

    // If the algebraic initial guess yields too large a circle, the optimization isn't going to bring it to a smaller
    // radius very much, thus just return the mean_position_and_radius_circle
    auto initialGuess = circle_fit_by_taubin(pointsBegin, pointsEnd);
    if((initialGuess.radius() >= minRadius) && (initialGuess.radius() <= maxRadius))
    {
        int fitResult = circle_fit_by_lma(pointsBegin, pointsEnd, initialGuess, 0.01, fitCircle);

        if(fitResult != 0)
        {
//             std::cerr << "WARNING: minimum_geometric_error_circle: Failed to find proper fit! Using algebraic fit.\n";
            fitCircle = initialGuess;
        }
        
        // If the final circle is absurdly large, then use the algebraic circle, which was already validated
        if((fitCircle.radius() < minRadius) || (fitCircle.radius() > maxRadius))
        {
//             std::cerr << "WARNING: minimum_geometric_error_circle: Geometric fit circle too big or small."
//                 << "Using algebraic instead. Geom:" << fitCircle << " Alg:" << initialGuess << '\n';
            fitCircle = initialGuess;
        }
    }
    else
    {
        fitCircle = mean_position_and_radius_circle(pointsBegin, pointsEnd);
    }
    
    return fitCircle;
}


Circle<float> mean_position_and_radius_circle(std::vector<Point<float>>::const_iterator pointsBegin, std::vector<Point<float>>::const_iterator pointsEnd)
{
    auto numPoints = std::distance(pointsBegin, pointsEnd);

    assert(numPoints > 1);

    Point<float> center;
    for(auto pointIt = pointsBegin; pointIt != pointsEnd; ++pointIt)
    {
        center += *pointIt;
    }

    center.x /= numPoints;
    center.y /= numPoints;

    double radius = 0.0;
    for(auto pointIt = pointsBegin; pointIt != pointsEnd; ++pointIt)
    {
        radius += distance_between_points(*pointIt, center);
    }
    
    radius /= numPoints;

    if(radius == 0.0)
    {
        std::cerr << "ERROR: mean_position_and_radius_circle: Bad data! Mean distance is 0! Mean:" << center << '\n';
        radius = std::numeric_limits<float>::max();
    }

    return Circle<float>(radius, center);
}

} // namespace math
} // namespace vulcan

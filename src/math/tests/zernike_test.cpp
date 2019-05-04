/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <core/point.h>
#include <math/zernike_moments.h>
#include <math/geometry/rectangle.h>
#include <math/geometry/shape_fitting.h>
#include <iostream>
#include <iterator>
#include <vector>

int main(int argc, char** argv)
{
    using namespace vulcan;
    
    std::vector<Point<double>> testPoints;
    for(int x = -10; x <= 0; ++x)
    {
        for(int y = -10; y <= 0; ++y)
        {
            testPoints.emplace_back(x, y);
        }
    }
    
    auto center   = Point<double>(0, 0);
    double length = 25.0;
    
    math::ZernikeMoments<10> zernike;
    auto moments = zernike.moments<double>(testPoints.begin(), testPoints.end(), center, length);
    
    testPoints.clear();
    for(int x = 0; x <= 10; ++x)
    {
        for(int y = 0; y <= 10; ++y)
        {
            testPoints.emplace_back(x, y);
        }
    }
    
    auto rotatedMoments = zernike.moments<double>(testPoints.begin(), testPoints.end(), center, length);
    
    std::cout << "Moments: Orig  Rotated  Diff %Diff\n";
    for(std::size_t n = 0; n < moments.size(); ++n)
    {
        std::cout << moments[n] << ' ' << rotatedMoments[n] << ' ' << (moments[n] - rotatedMoments[n])
            << ((moments[n] - rotatedMoments[n]) / moments[n]) << '\n';
    }
    
    testPoints.clear();
    for(int x = -10; x <= 0; ++x)
    {
        for(int y = 0; y <= 10; ++y)
        {
            testPoints.emplace_back(x, y);
        }
    }
    auto nextRotationMoments = zernike.moments<double>(testPoints.begin(), testPoints.end(), center, length);
    
    std::copy(nextRotationMoments.begin(), nextRotationMoments.end(), std::ostream_iterator<double>(std::cout, "\n"));
    
    math::Rectangle<double> failureBoundary(Point<double>(-45.1, -30.8), Point<double>(-44.75, -30.45));
    Point<double> failureCenter(-44.925, -30.625);

    testPoints.clear();
    testPoints.emplace_back(-45.0, -30.8);
    testPoints.emplace_back(failureBoundary.bottomLeft);
    testPoints.emplace_back(failureBoundary.bottomRight);
    testPoints.emplace_back(failureBoundary.topLeft);
    testPoints.emplace_back(failureBoundary.topRight);
    testPoints.emplace_back(failureCenter);

    auto failureMoments = zernike.moments<double>(testPoints.begin(), 
                                                  testPoints.end(), 
                                                  failureCenter, 
                                                  std::max(failureBoundary.width(), failureBoundary.height()));
    std::cout << "Moments for points bounded by " << failureBoundary << " with center " << failureCenter << ":\n";
    std::copy(failureMoments.begin(), failureMoments.end(), std::ostream_iterator<double>(std::cout, "\n"));

    return 0;
}

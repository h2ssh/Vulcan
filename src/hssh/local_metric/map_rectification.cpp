/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


// /**
// * \file     map_rectification.cpp
// * \author   Collin Johnson
// *
// * Definition of map rectification functionality:
// *
// *   - calculate_map_orientation
// *   - rotate_lpm
// */
// 
// #include <hssh/local_metric/map_rectification.h>
// #include <cmath>
// #include <hssh/local_metric/lpm.h>
// 
// namespace vulcan
// {
// namespace hssh
// {
// 
// float calculate_map_orientation(const LocalPerceptualMap& lpm)
// {
//     /*
//     * From the Chapter 3 of "Robot Vision" by Horn, we get the following formulas for determining the orientation:
//     *
//     * x' = x - avgX
//     * y' = y - avgY
//     *
//     * a = sum(x'^2) for all pixels in the region
//     * b = sum(x' * y') for all pixels in the reigon
//     * c = sum(y'^2) for all pixels in the region
//     *
//     * Alternately, we can define these so they can be calculated on the fly via the following functions:
//     *
//     * a = sumXSquare - sumX * sumX / numPixels
//     * b = sumXY - sumX * sumY / numPixels
//     * c = sumYSquare - sumY * sumY / numPixels
//     *
//     * Once we have determined a, b, and c, we can perform these calculations:
//     *
//     * tan(2 * theta) = b / (a - c)
//     */
// 
//     int64_t sumX  = 0;
//     int64_t sumY  = 0;
//     int64_t sumX2 = 0;
//     int64_t sumY2 = 0;
//     int64_t sumXY = 0;
//     int64_t area  = 0;
// 
//     for(int64_t y = 0; y < lpm.getHeightInCells(); ++y)
//     {
//         for(int64_t x = 0; x < lpm.getWidthInCells(); ++x)
//         {
//             if(lpm.getCellTypeNoCheck(x, y) & kFreeOccGridCell)
//             {
//                 sumX  += x;
//                 sumY  += y;
//                 sumX2 += x*x;
//                 sumY2 += y*y;
//                 sumXY += x*y;
// 
//                 ++area;
//             }
//         }
//     }
// 
//     double a = sumX2 - sumX*sumX / area;
//     double b = sumXY - sumX*sumY / area;
//     double c = sumY2 - sumY*sumY / area;
// 
//     return std::atan(b / (a-c)) / 2.0;
// }
// 
// 
// LocalPerceptualMap rotate_lpm(const LocalPerceptualMap& lpm, float angle)
// {
//     LocalPerceptualMap rotated(lpm);
//     rotated.reset();
// 
//     LPMSubGrid& costs = rotated.getCostGrid();
//     LPMSubGrid& types = rotated.getTypeGrid();
// 
//     float cosAngle = std::cos(angle);
//     float sinAngle = std::sin(angle);
// 
//     Point<uint16_t> rotatedPoint;
// 
//     for(size_t y = 0; y < lpm.getHeightInCells(); ++y)
//     {
//         for(size_t x = 0; x < lpm.getWidthInCells(); ++x)
//         {
//             rotatedPoint.x = x*cosAngle - y*sinAngle;
//             rotatedPoint.y = x*sinAngle + y*cosAngle;
// 
//             costs.setValue(rotatedPoint.x, rotatedPoint.y, lpm.getCostNoCheck(x, y));
//             types.setValue(rotatedPoint.x, rotatedPoint.y, lpm.getCellTypeNoCheck(x, y));
//         }
//     }
// 
//     return rotated;
// }
// 
// } // namespace hssh
// } // namespace vulcan

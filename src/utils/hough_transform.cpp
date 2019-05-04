/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hough_transform.cpp
* \author   Collin Johnson
* 
* Definition of HoughTransform.
*/

#include <utils/hough_transform.h>
#include <core/angle_functions.h>
#include <algorithm>
#include <cassert>

namespace vulcan
{
namespace utils
{

radial_line_t HoughTransform::bestLine(void)
{
    auto gridRange = make_grid_iterators<int32_t>(accumulator_);
    auto bestIt = std::max_element(gridRange.first, gridRange.second);
    assert(bestIt != gridRange.second);
    
    radial_line_t line;
    line.radius = bestIt.y() * kMetersPerCell_;
    line.theta = wrap_to_pi((bestIt.x() * kRadiansPerCell_) - M_PI_2);
    
    return line;
}
    

void HoughTransform::markCellLines(int x, int y)
{
    const int kMaxRCell = accumulator_.getHeightInCells();
    
    for(std::size_t theta = 0; theta < accumulator_.getWidthInCells(); ++theta)
    {
        float radians = (theta * kRadiansPerCell_) - M_PI_2;
        float radius = (x * std::cos(radians)) + (y * std::sin(radians)) + kMaxRadius_;
        int rCell = radius / kMetersPerCell_;
        
        if((rCell >= 0) && (rCell < kMaxRCell))
        {
            ++accumulator_(theta, rCell);
        }
    }
}

} // namespace utils
} // namespace vulcan

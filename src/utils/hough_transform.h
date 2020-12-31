/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hough_transform.h
* \author   Collin Johnson
*
* Declaration of HoughTransform.
*/

#ifndef UTILS_HOUGH_TRANSFORM_H
#define UTILS_HOUGH_TRANSFORM_H

#include "utils/cell_grid.h"
#include "utils/grid_iterators.h"
#include <cmath>

namespace vulcan
{
namespace utils
{
  
/**
* radial_line_t defines the coordinates (r, theta) of a line found in the Hough transform.
*/
struct radial_line_t
{
    double radius;  // distance from origin to line
    double theta;   // normal of the line pointing away from origin
};

/**
* wrap_hough_angle changes the angle from the Hough transform to the orthogonal angle closest to the x-axis. The
* assumption here is that either axis will be detected by the transform, so select the axis closest to the x-axis
* for consistency.
* 
* This function puts an angle in the range [-pi/2, pi/2] into the range [-pi/4, pi/4].
*/
inline double wrap_hough_angle(double angle)
{
    if(angle > M_PI_4)
    {
        return angle - M_PI_2;
    }
    else if(angle < -M_PI_4)
    {
        return angle + M_PI_2;
    }
    else
    {
        return angle;
    }
}

/**
* HoughTransform is a class to create and run a HoughTransform. Reusing the class will save memory allocation costs.
*/
class HoughTransform
{
public:
    
    using Accumulator = CellGrid<int32_t>;
    
    /**
    * Constructor for HoughTransform.
    * 
    * \param    angleResolution         360/angle_resolution = Number of angle bins to use (optional, default = 1)
    * \param    radiusResolution        max_radius_in_cells/radius_resolution = Number of radius bins to use, 
    *                                       (optional, default = 1)
    */
    explicit HoughTransform(float angleResolution = 1.0f, float radiusResolution = 1.0f)
    : kRadiansPerCell_(std::max(angleResolution * M_PI / 180.0, 0.001))
    , kMetersPerCell_(std::max(radiusResolution, 0.01f))
    {
    }
    
    /**
    * compute computes the Hough transform for the provided Grid instance.
    * 
    * The Grid is the type of Grid for which the transform is computed. It must satisfy the Grid concept.
    * The CellFunc is a function that takes a cell coordinate and grid (x, y, grid) and returns true if the cell should
    *   be considered for the calculation of a Hough line.
    */
    template <class Grid, class CellFunc>
    void compute(const Grid& grid, CellFunc op);
    
    /**
    * bestLine retrieves the coordinates of the best line in the accumulator grid. Remember that the theta is the 
    * normal to the line and radius is distance to the line from the origin.
    */
    radial_line_t bestLine(void);
    
    /**
    * accumulator retrieves the full accumulator grid to allow further processing.
    */
    const Accumulator& accumulator(void) const { return accumulator_; }
    
private:
    
    const float kRadiansPerCell_;
    const float kMetersPerCell_;
    float kMaxRadius_;
    
    Accumulator accumulator_;         // x = angle, y = radius
    
    void markCellLines(int x, int y);
};


template <class Grid, class CellFunc>
void HoughTransform::compute(const Grid& grid, CellFunc op)
{
    std::size_t width = std::ceil(M_PI / kRadiansPerCell_) + 1;
    
    kMaxRadius_ = std::sqrt(std::pow(grid.getHeightInCells(), 2.0) + std::pow(grid.getWidthInCells(), 2.0));
    std::size_t height = (2 * std::ceil(kMaxRadius_ / kMetersPerCell_)) + 1;
    
    if((accumulator_.getWidthInCells() < width)
        || (accumulator_.getHeightInCells() < height))
    {
        accumulator_.setGridSizeInCells(width, height);
    }
    accumulator_.reset(0);
    
    for(std::size_t y = 0; y < grid.getHeightInCells(); ++y)
    {
        for(std::size_t x = 0; x < grid.getWidthInCells(); ++x)
        {
            if(op(x, y, grid))
            {
                markCellLines(x, y);
            }
        }
    }
}

} // namespace utils
} // namespace vulcan

#endif // UTILS_HOUGH_TRANSFORM_H

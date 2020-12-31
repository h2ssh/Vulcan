/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     filter_initializer_impl.cpp
 * \author   Collin Johnson
 *
 * Definition of implementations of the FilterInitializer interface:
 *
 *   - RegionFilterInitializer
 *   - FreeSpaceFilterInitializer
 */

#include "hssh/metrical/relocalization/filter_initializer_impl.h"
#include "hssh/metrical/localization/particle.h"
#include "hssh/metrical/occupancy_grid.h"
#include "hssh/metrical/relocalization/particle_sampling.h"
#include <cstdlib>

namespace vulcan
{
namespace hssh
{

//////////////////// RegionFilterInitializer implementation /////////////////////////
RegionFilterInitializer::RegionFilterInitializer(const math::Rectangle<float>& region,
                                                 int numPositions,
                                                 int posesPerPosition)
: region_(region)
, numPositions_(numPositions)
, posesPerPosition_(posesPerPosition)
{
}

// FilterInitializer interface
std::vector<particle_t> RegionFilterInitializer::generateInitialSamples(const OccupancyGrid& grid,
                                                                        const metric_slam_data_t& data) const
{
    std::vector<particle_t> samples;

    Line<float> regionBottom(region_.bottomLeft, region_.bottomRight);
    double cellWidth = std::sqrt((region_.area() / numPositions_) * posesPerPosition_);
    double bottomAngle = angle_to_point(regionBottom.a, regionBottom.b);
    double sideAngle = angle_to_point(region_.bottomLeft, region_.topLeft);

    double sampleWeight = 1.0 / numPositions_;

    int samplesAlongX = length(regionBottom) / cellWidth + 1;

    for (int y = 0; samples.size() < numPositions_; ++y) {
        Point<float> start(region_.bottomLeft.x + y * cellWidth * std::cos(sideAngle),
                           region_.bottomLeft.y + y * cellWidth * std::sin(sideAngle));

        for (int x = 0; x < samplesAlongX; ++x) {
            Point<float> position(start.x + x * cellWidth * std::cos(bottomAngle),
                                  start.y + x * cellWidth * std::sin(bottomAngle));

            generate_uniform_samples_at_position(position, posesPerPosition_, sampleWeight, samples);
        }
    }

    return samples;
}


//////////////////// FreeSpaceFilterInitializer implementation /////////////////////////
FreeSpaceFilterInitializer::FreeSpaceFilterInitializer(int stride, int posesPerCell)
: stride_(stride)
, posesPerCell_(posesPerCell)
{
}


std::vector<particle_t> FreeSpaceFilterInitializer::generateInitialSamples(const OccupancyGrid& grid,
                                                                           const metric_slam_data_t& data) const
{
    // Create the empty set of particles
    // Iterate through cells in the map, according to the stride
    // If a cell is free space, generate random samples
    // Go through and normalize the cell weights because the number of free cells isn't know a priori

    std::vector<particle_t> samples;

    for (std::size_t y = stride_; y < grid.getHeightInCells(); ++y) {
        for (std::size_t x = stride_; x < grid.getWidthInCells(); ++x) {
            if (grid.getCellTypeNoCheck(x, y) & kFreeOccGridCell && (drand48() > 0.75)) {
                generate_random_samples_at_position(utils::grid_point_to_global_point(Point<int>(x, y), grid),
                                                    posesPerCell_,
                                                    0.1,
                                                    samples);
            }
        }
    }

    Point<int> zero = utils::global_point_to_grid_cell(Point<double>(0.0, 0.0), grid);

    if (grid.getCellType(zero.x, zero.y) & kFreeOccGridCell) {
        generate_uniform_samples_at_position(Point<double>(0.0, 0.0), posesPerCell_ * 5, 0.1, samples);
    }

    for (auto& sample : samples) {
        sample.weight = 1.0 / samples.size();
    }

    return samples;
}

}   // namespace hssh
}   // namespace vulcan

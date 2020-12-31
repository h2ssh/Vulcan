/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     scan_matching_initializer.cpp
* \author   Collin Johnson
* 
* Definition of ScanMatchingInitializer.
*/

#include "hssh/metrical/relocalization/scan_matching_initializer.h"
#include "hssh/metrical/relocalization/filter_initializer_impl.h"
#include "hssh/metrical/localization/particle.h"
#include "hssh/metrical/data.h"
#include "hssh/metrical/occupancy_grid.h"
#include "utils/ray_tracing.h"
#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/binary.hpp>
#include <cassert>
#include <future>
#include <iostream>

#define DEBUG_FREE_SPACE_SAMPLES
#define DEBUG_REGION_SAMPLES

namespace vulcan
{
namespace hssh
{
    
using ParticleIter = std::vector<particle_t>::iterator;
    

inline bool is_ray_endpoint(const OccupancyGrid& grid, Point<int> cell)
{
    return grid.getCellType(cell.x, cell.y) & (kOccupiedOccGridCell | kUnknownOccGridCell);
}

void                   assign_particle_errors_to_weights(ParticleIter begin, ParticleIter end, const metric_slam_data_t& data, const OccupancyGrid& grid);
double                 calculate_particle_error         (const particle_t& particle, const metric_slam_data_t& data, const OccupancyGrid& grid);
math::Rectangle<float> create_region_for_sample         (const particle_t& sample);
bool                   is_sample_in_existing_region     (const particle_t& sample, const std::vector<math::Rectangle<float>>& regions);

struct ParticleErrorFunc
{
    void operator()(ParticleIter begin, ParticleIter end, const metric_slam_data_t& data, const OccupancyGrid& grid)
    {
        for(auto& particleIt = begin; particleIt != end; ++particleIt)
        {
            particleIt->weight = calculate_particle_error(*particleIt, data, grid);
        }
    }
};


ScanMatchingInitializer::ScanMatchingInitializer(void)
{
}


std::vector<particle_t> ScanMatchingInitializer::generateInitialSamples(const OccupancyGrid& grid, const metric_slam_data_t& data) const
{
    /*
    * An initial set of samples needs to be sprinkled across the free space in the map -- the robot could be anywhere.
    * Each samples is scored.
    * The best N samples are found.  (N=500)
    * A 1x1 region is defined around each of the best samples.
    * Samples are generated within each region.
    * The region-samples are returned.
    */
    
    const std::size_t kNumRegionSamples    = 500;
    const int         kNumSamplesPerRegion = 500;
    const int         kNumPosesPerPosition = 20;
    
    auto freeSpaceSamples = FreeSpaceFilterInitializer(10, 10).generateInitialSamples(grid, data);
    
    assign_particle_errors_to_weights(freeSpaceSamples.begin(), freeSpaceSamples.end(), data, grid);
   
    // If there are more free space samples than the number of region samples, sort in descending order of weight so the best samples are easily found
    if(kNumRegionSamples < freeSpaceSamples.size())
    {
        std::sort(freeSpaceSamples.begin(), freeSpaceSamples.end());
    }
    
    std::vector<math::Rectangle<float>> regionsForBestSamples;
    std::vector<particle_t>             finalSamples;
    
    for(std::size_t n = 0, end = std::min(kNumRegionSamples, freeSpaceSamples.size()); n < end; ++n)
    {
        if(!is_sample_in_existing_region(freeSpaceSamples[n], regionsForBestSamples))
        {
            auto region        = create_region_for_sample(freeSpaceSamples[n]);
            auto regionSamples = RegionFilterInitializer(region, kNumSamplesPerRegion, kNumPosesPerPosition).generateInitialSamples(grid, data);
            
            finalSamples.insert(finalSamples.end(), regionSamples.begin(), regionSamples.end());
            regionsForBestSamples.push_back(region);
        }
    }
    
    return finalSamples;
}


void assign_particle_errors_to_weights(ParticleIter begin, ParticleIter end, const metric_slam_data_t& data, const OccupancyGrid& grid)
{
    // Create up to four threads for finding the isovists. Assign 1/4th of the work to each thread.
    const std::size_t numParticles = std::distance(begin, end);
    std::size_t kMaxThreads = 4;
    const int kNumThreads = std::min(kMaxThreads, numParticles/kMaxThreads+1);
    
    std::size_t positionsPerThread = numParticles / kNumThreads + 1; // Add one to the positions/thread to account for truncating due to integer division
    // The last thread will take up to 3 fewer isovist calculations
    
#ifdef DEBUG_FREE_SPACE_SAMPLES
    std::cout << "DEBUG: ScanMatchingInitializer: Calculating weights for " << numParticles << " particles using " << kNumThreads << " threads.\n";
#endif
    
    // Launch the threads
    std::vector<std::future<void>> particleFutures;
    for(int n = 0; n < kNumThreads; ++n)
    {
        std::size_t start = positionsPerThread*n;
        std::size_t end   = std::min(positionsPerThread*(n+1), numParticles);
        
        particleFutures.push_back(std::async(std::launch::async, ParticleErrorFunc(), begin+start, begin+end, data, grid));
    }
    
    for(auto& f : particleFutures)
    {
        f.get();
    }
}


double calculate_particle_error(const particle_t& particle, const metric_slam_data_t& data, const OccupancyGrid& grid)
{
//     const std::size_t kRayStride = 20;
//     
//     utils::ray_trace_range_t traceRange(wrap_to_pi(data.laser.startAngle + data.laser.offset.theta + particle.pose.theta),
//                                         data.laser.ranges.size() / kRayStride, 
//                                         std::abs(data.laser.angularResolution) * kRayStride, 
//                                         data.laser.angularResolution > 0.0f);
//     
//     auto startingGridPosition = utils::global_point_to_grid_point(particle.pose.toPoint(), grid);
//     
//     auto rays = utils::trace_range_until_condition(startingGridPosition,
//                                                    traceRange, 
//                                                    data.laser.maxRange, 
//                                                    grid, 
//                                                    is_ray_endpoint);
// 
//     double error = 0.0;
//     
//     for(std::size_t n = 0; n < traceRange.numIncrements; n += kRayStride)
//     {
//         float expectedDist = distance_between_points(startingGridPosition, rays[n]) * grid.cellsPerMeter();
//         float measuredDist = data.laser.ranges[n*kRayStride];
//         
//         // Distances less than 0 indicate an error
//         if((measuredDist > 0.0f) && (expectedDist > 0.0f))
//         {
//             error += std::abs(expectedDist - measuredDist) / expectedDist;
//         }
//     }
//     
//     return error;
    
    cartesian_laser_scan_t scan;
    polar_scan_to_cartesian_scan_in_global_frame(data.laser, particle.pose, scan);
    
    int numValidRanges  = 0;
    int numOccupiedHits = 0;
    
    for(std::size_t n = 0; n < data.laser.ranges.size(); ++n)
    {
        if(data.laser.ranges[n] < 20.0f && data.laser.ranges[n] > 0.25f)
        {
            auto gridPoint = utils::global_point_to_grid_cell(scan.scanPoints[n], grid);
            
            if(grid.getCellType(gridPoint) & (kOccupiedOccGridCell | kUnknownOccGridCell))
            {
                numOccupiedHits += grid.getCost(gridPoint);
            }
            
            numValidRanges += grid.getMaxCellCost();
        }
    }
    
    
    return (numValidRanges > 0) ? static_cast<float>(numOccupiedHits) / numValidRanges : 0.0f;
}


math::Rectangle<float> create_region_for_sample(const particle_t& sample)
{
    const float kRegionRadius = 0.5f;
    
    return math::Rectangle<float>(Point<float>(sample.pose.x-kRegionRadius, sample.pose.y-kRegionRadius),
                                  Point<float>(sample.pose.x+kRegionRadius, sample.pose.y+kRegionRadius));
}


bool is_sample_in_existing_region(const particle_t& sample, const std::vector<math::Rectangle<float>>& regions)
{
    for(auto& region : regions)
    {
        if(region.contains(sample.pose.x, sample.pose.y))
        {
            return true;
        }
    }
    
    return false;
}

} // namespace hssh
} // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     discrete_beam_model.cpp
* \author   Collin Johnson
* 
* Definition of DiscreteBeamModel.
*/

#include "hssh/metrical/localization/discrete_beam_model.h"
#include "hssh/metrical/localization/params.h"
#include "hssh/metrical/localization/particle.h"
#include "hssh/metrical/localization/scan_weighting.h"
#include "hssh/metrical/occupancy_grid.h"
#include "laser/laser_scan_lines.h"
#include "laser/moving_laser_scan.h"
#include "utils/cell_grid_utils.h"
#include "utils/ray_tracing.h"
#include <cassert>

namespace vulcan
{
namespace hssh
{

namespace
{
bool is_ray_endpoint(const OccupancyGrid& grid, Point<int> cell)
{
//     return grid.getCellType(cell.x, cell.y) & kOccupiedOccGridCell;
    return grid.getCost(cell) > 128;
}
}
    

DiscreteBeamModel::DiscreteBeamModel(const discrete_beam_model_params_t& params)
: maxLaserDistance_      (params.maxRayDistance)
, stride_                (params.rayStride)
, hitLoglihood_          (params.hitLoglihood)
, shortLoglihood_        (params.shortLoglihood)
, longLoglihood_         (params.longLoglihood)
, useCovarianceWeighting_(params.useCovarianceWeighting)
{
}


void DiscreteBeamModel::initializeModel(const laser::laser_scan_lines_t&  scan, 
                                        const MultivariateGaussian& proposalDistribution)
{
    if(useCovarianceWeighting_)
    {
        calculate_covariance_weights(scan, proposalDistribution, stride_, scanWeights_);
    }
    else
    {
        scanWeights_.resize(scan.scan.numRanges);
        std::fill(scanWeights_.begin(), scanWeights_.end(), 1.0);
    }
}
    
    
double DiscreteBeamModel::sampleLikelihood(const particle_t&                 sample,
                                           const laser::laser_scan_lines_t&  scan,
                                           const OccupancyGrid&              map,
                                           particle_filter_debug_info_t*     debug)
{
    laser::MovingLaserScan movingScan(scan.scan, sample.parent, sample.pose, stride_);
    assert(movingScan.size() <= scanWeights_.size());
    
    double scanLogLikelihood = 0.0;
    
    for(std::size_t n = 0; n < movingScan.size(); ++n)
    {
        scanLogLikelihood += scanWeights_[n] * beamLogLikelihood(movingScan[n], map);
    }
    
    return scanLogLikelihood;
}


double DiscreteBeamModel::beamLogLikelihood(const laser::adjusted_ray_t& ray, const OccupancyGrid& grid)
{
    auto startingGridPoint = utils::global_point_to_grid_point(ray.position, grid);
    auto expectedEndpoint  = utils::trace_ray_until_condition(startingGridPoint,
                                                              angle_to_point(ray.position, ray.endpoint), 
                                                              maxLaserDistance_, 
                                                              grid, 
                                                              is_ray_endpoint);
    
    Point<int> expectedCell = expectedEndpoint;
    Point<int> measuredCell = utils::global_point_to_grid_cell(ray.endpoint, grid);
    
    if(expectedCell == measuredCell)
    {
        return hitLoglihood_;
    }
    
    int expectedDistanceInCells = distance_between_points(startingGridPoint, expectedEndpoint);
    int measuredDistanceInCells = ray.range * grid.cellsPerMeter();
    
    if(expectedDistanceInCells < measuredDistanceInCells)
    {
        return longLoglihood_;
    }
    else // if(measuredDistanceInCells < expectedDistanceInCells)
    {
        return shortLoglihood_;
    }
}

} // namespace hssh
} // namespace vulcan

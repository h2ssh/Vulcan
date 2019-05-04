/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gaussian_observation_model.cpp
* \author   Collin Johnson
*
* Definition of GaussianObservationModel.
*/

#include <hssh/metrical/localization/gaussian_observation_model.h>
#include <hssh/metrical/localization/params.h>
#include <hssh/metrical/localization/particle.h>
#include <hssh/metrical/occupancy_grid.h>
#include <laser/laser_scan_lines.h>
#include <laser/moving_laser_scan.h>

namespace vulcan
{
namespace hssh
{

GaussianObservationModel::GaussianObservationModel(const gaussian_observation_model_params_t& params)
: kMaxDistance_(params.maxEndpointDistance)
, kStride_(params.rayStride)
{
    assert(kMaxDistance_ > 0.0);
    assert(kStride_ > 0);

    double sigma = params.gaussianSigma;
    double normalizer = 1.0 / (std::sqrt(2*M_PI)*sigma);

    double total = 0.0;
    int index = 0;
    for(int y = -1; y <= 1; ++y)
    {
        for(int x = -1; x <= 1; ++x)
        {
            double rad = 0.05 * std::sqrt(x*x + y*y);
            gaussian_[index] = normalizer * std::exp(-0.5*rad*rad / (sigma*sigma));
            total += gaussian_[index];
            ++index;
        }
    }

    for(auto& val : gaussian_)
    {
        val /= total;
    }
}


void GaussianObservationModel::initializeModel(const laser::laser_scan_lines_t&  scan,
                                               const MultivariateGaussian& proposalDistribution)
{
    // Nothing to initialize
}


double GaussianObservationModel::sampleLikelihood(const particle_t&                 sample,
                                                  const laser::laser_scan_lines_t&  scan,
                                                  const OccupancyGrid&              map,
                                                  particle_filter_debug_info_t*     debug)
{
    const double kMaxCost = map.getMaxCellCost();

    double totalLikelihood = 0.0;
    double maxTotalLikelihood = 0.0;
    laser::MovingLaserScan movingScan(scan.scan, sample.parent, sample.pose, kStride_);

    for(std::size_t n = 0; n < movingScan.size(); ++n)
    {
        const auto& ray = movingScan[n];

        if((ray.range < kMaxDistance_) && (ray.range > 0.0))
        {
            double rayCost = 0.0;
            int index = 0;

            auto gridPoint = utils::global_point_to_grid_cell(ray.endpoint, map);

            // Convolve the 3x3 square centered on the ray's endpoint in the grid
            for(int y = gridPoint.y-1; y <= gridPoint.y+1; ++y)
            {
                for(int x = gridPoint.x-1; x <= gridPoint.x+1; ++x)
                {
                    if(map.isCellInGrid(Point<int>(x, y)))
                    {
                        cell_type_t pointType = map.getCellTypeNoCheck(x, y);
                        int cost = map.getCostNoCheck(x, y);

                        // Unobserved cells always have 0 cost
                        if(pointType & ~kUnobservedOccGridCell)
                        {
                            // Otherwise the accumulated cost is scaled by the Gaussian to determine how much it
                            // changes the particular ray's additive cost
                            rayCost += gaussian_[index] * cost;
                        }

                        ++index;
                    }
                }
            }

            // The likelihood of the ray is how much cost it accrued over the maximum possible cost
            totalLikelihood += rayCost / kMaxCost;
            maxTotalLikelihood += kMaxCost;
        }
    }

    assert(maxTotalLikelihood > 0.0);
    assert(totalLikelihood <= maxTotalLikelihood);
    assert(totalLikelihood >= 0.0);

    return (totalLikelihood > 0.0) ? (totalLikelihood / maxTotalLikelihood) : (1.0 / (kMaxCost * movingScan.size()));
}

} // namespace hssh
} // namespace vulcan

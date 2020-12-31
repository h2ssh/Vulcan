/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     discrete_beam_model.h
* \author   Collin Johnson
* 
* Declaration of DiscreteBeamModel.
*/

#ifndef HSSH_UTILS_METRICAL_LOCALIZATION_DISCRETE_BEAM_MODEL_H
#define HSSH_UTILS_METRICAL_LOCALIZATION_DISCRETE_BEAM_MODEL_H

#include "hssh/metrical/localization/observation_model.h"
#include <vector>

namespace vulcan
{
namespace laser { struct adjusted_ray_t; }
namespace hssh
{
    
struct discrete_beam_model_params_t;

const std::string kDiscreteBeamModelType("discrete_beam");

/**
* DiscreteBeamModel is a grid-based observation model that uses ray-casting and occupancy-based weighting to determine 
* how well a given scan matches the map.
*/
class DiscreteBeamModel : public ObservationModel
{
public:
    
    /**
    * Constructor for DiscreteBeamModel.
    * 
    * \param    params          Parameters controlling the behavior of the observation model
    */
    DiscreteBeamModel(const discrete_beam_model_params_t& params);
    
    ////////////// ObservationModel interface //////////////////
    virtual void initializeModel(const laser::laser_scan_lines_t&  scan, 
                                 const MultivariateGaussian& proposalDistribution) override;
                                 
    virtual double sampleLikelihood(const particle_t&                 sample,
                                    const laser::laser_scan_lines_t&  scan,
                                    const OccupancyGrid&              map,
                                    particle_filter_debug_info_t*     debug) override;
                                                                 
private:

    const float  maxLaserDistance_;
    const int    stride_;
    const double hitLoglihood_;
    const double shortLoglihood_;
    const double longLoglihood_;
    
    const bool         useCovarianceWeighting_;
    std::vector<float> scanWeights_;
    
    
    double beamLogLikelihood(const laser::adjusted_ray_t& ray, const OccupancyGrid& grid);
};

}
}

#endif // HSSH_UTILS_METRICAL_LOCALIZATION_DISCRETE_BEAM_MODEL_H

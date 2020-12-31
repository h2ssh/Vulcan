/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     observation_model.cpp
* \author   Collin Johnson
*
* Definition of the create_observation_model factory.
*/

#include "hssh/metrical/localization/observation_model.h"
#include "hssh/metrical/localization/grid_based_observation_models.h"
#include "hssh/metrical/localization/discrete_beam_model.h"
#include "hssh/metrical/localization/gaussian_observation_model.h"
#include "hssh/metrical/localization/params.h"
#include <cassert>
#include <iostream>

namespace vulcan
{
namespace hssh
{

std::unique_ptr<ObservationModel> create_observation_model(const std::string& modelName, const observation_model_params_t& params)
{
    if(modelName == ENDPOINT_OBSERVATION_MODEL_TYPE)
    {
        return std::unique_ptr<ObservationModel>(new EndpointObservationModel(params.endpointParams));
    }
    else if(modelName == BEAM_OBSERVATION_MODEL_TYPE)
    {
        return std::unique_ptr<ObservationModel>(new BeamObservationModel(params.beamParams));
    }
    else if(modelName == kDiscreteBeamModelType)
    {
        return std::unique_ptr<ObservationModel>(new DiscreteBeamModel(params.discreteParams));
    }
    else if(modelName == kGaussianModelType)
    {
        return std::unique_ptr<ObservationModel>(new GaussianObservationModel(params.gaussianParams));
    }
    else
    {
        std::cerr<<"ERROR: createObservationModel: Unknown model type: "<<modelName<<std::endl;
        assert(false);
    }

    return std::unique_ptr<ObservationModel>();
}

}
}

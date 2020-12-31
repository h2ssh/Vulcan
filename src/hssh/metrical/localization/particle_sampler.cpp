/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     particle_sampler.cpp
 * \author   Collin Johnson
 *
 * Definition of create_particle_sampler factory function.
 */

#include "hssh/metrical/localization/particle_sampler.h"
#include "hssh/metrical/localization/adaptive_particle_filter.h"
#include "hssh/metrical/localization/vanilla_particle_filter.h"
#include <cassert>
#include <iostream>

namespace vulcan
{
namespace hssh
{

std::unique_ptr<ParticleSampler> create_particle_sampler(const std::string& type)
{
    if (type == ADAPTIVE_PARTICLE_FILTER_TYPE) {
        return std::unique_ptr<ParticleSampler>(new GaussianParticleSampler);
    } else if (type == VANILLA_PARTICLE_FILTER_TYPE) {
        return std::unique_ptr<ParticleSampler>(new LowDispersionSampler);
    }

    std::cerr << "ERROR: create_particle_sampler: Unknown type: " << type << '\n';
    assert(false);
    return std::unique_ptr<ParticleSampler>();
}

}   // namespace hssh
}   // namespace vulcan

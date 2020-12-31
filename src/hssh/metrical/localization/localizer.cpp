/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     localizer.cpp
 * \author   Collin Johnson
 *
 * Definition of make_localizer.
 */

#include "hssh/metrical/localization/localizer.h"
#include "hssh/metrical/localization/known_pose_localizer.h"
#include "hssh/metrical/localization/monte_carlo.h"
#include "hssh/metrical/localization/params.h"
#include <cassert>
#include <iostream>

namespace vulcan
{
namespace hssh
{

std::unique_ptr<Localizer> make_localizer(const localizer_params_t& params)
{
    if (params.type == kKnownPoseType) {
        return std::unique_ptr<Localizer>(new KnownPoseLocalizer);
    } else if (params.type == kMonteCarloType) {
        return std::unique_ptr<Localizer>(new MonteCarloLocalization(params.monteCarloParams));
    }

    std::cerr << "ERROR: make_localizer: Unknown localizer type:" << params.type << '\n';
    assert(false);
    return std::unique_ptr<Localizer>();
}

}   // namespace hssh
}   // namespace vulcan

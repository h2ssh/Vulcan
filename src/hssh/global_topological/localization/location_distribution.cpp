/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     location_distribution.cpp
* \author   Collin Johnson
* 
* Definition of GlobalLocationDistribution.
*/

#include "hssh/global_topological/localization/location_distribution.h"
#include "core/float_comparison.h"
#include <cassert>

namespace vulcan
{
namespace hssh
{
    
GlobalLocationDistribution::GlobalLocationDistribution(const std::vector<WeightedGlobalLocation>& locations)
: locations_(locations)
{
    // Confirm the invariant holds
    double sumProb = 0.0;
    for(auto& l : locations_)
    {
        sumProb += l.probability;
    }
    
    assert(absolute_fuzzy_equal(sumProb, 1.0));
}


GlobalLocationDistribution::GlobalLocationDistribution(const GlobalLocation& location)
{
    locations_.emplace_back(1.0, location);
}

} // namespace hssh
} // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     place.cpp
 * \author   Collin Johnson
 *
 * Definition of LocalPlace.
 */

#include "hssh/local_topological/areas/place.h"

namespace vulcan
{
namespace hssh
{

boost::optional<local_path_fragment_t> match_gateway_to_star(const Gateway& gateway, const SmallScaleStar& star);


LocalPlace::LocalPlace(const SmallScaleStar& star,
                       const LocalPerceptualMap& map,
                       int id,
                       const AreaExtent& extent,
                       const std::vector<Gateway>& gateways)
: LocalArea(id, extent, gateways)
, star_(star)
, map_(map)
{
    // For each navigable path fragment in the small-scale star, create an associated transition affordance
    for (auto& f : star_) {
        if (f.navigable) {
            adjacent_.emplace_back(f.gateway, f.type);
        }
    }
}


boost::optional<TransitionAffordance> LocalPlace::affordanceForFragment(const local_path_fragment_t& fragment) const
{
    // There is a 1-1 correspondence between path segment affordances and path fragments.
    // Iterate through the fragments and count how many navigable fragments occur before this fragment is found.
    // That will be the index into path segments

    // If not navigable, then certainly no associated affordance
    if (!fragment.navigable) {
        return boost::none;
    }

    int numNavigableSoFar = 0;

    for (auto& f : star_) {
        // If the fragment is found, the index will be the count of navigable fragments so far
        if (f == fragment) {
            return adjacent_[numNavigableSoFar];
        } else if (f.navigable) {
            ++numNavigableSoFar;
        }
    }

    // If the fragment isn't found, then there is no associated affordance
    return boost::none;
}


boost::optional<local_path_fragment_t> LocalPlace::findGatewayFragment(const Gateway& gateway) const
{
    return match_gateway_to_star(gateway, star_);
}


void LocalPlace::visitAffordances(NavigationAffordanceVisitor& visitor) const
{
    for (auto& adj : adjacent_) {
        adj.accept(visitor);
    }
}


boost::optional<local_path_fragment_t> match_gateway_to_star(const Gateway& gateway, const SmallScaleStar& star)
{
    auto matchingFrag = std::find_if(star.begin(), star.end(), [&gateway](const local_path_fragment_t& frag) {
        return frag.gateway.isSimilarTo(gateway) && frag.navigable;
    });

    if (matchingFrag != star.end()) {
        return *matchingFrag;
    } else {
        std::cerr << "WARNING::SmallScaleStar: Failed to find fragment associated with gateway " << gateway
                  << " Star: " << star << '\n';
        return boost::none;
    }
}

}   // namespace hssh
}   // namespace vulcan

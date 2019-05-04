/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     location_distribution.h
* \author   Collin Johnson
* 
* Declaration of GlobalLocationDistribution.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_LOCALIZATION_LOCATION_DISTRIBUTION_H
#define HSSH_GLOBAL_TOPOLOGICAL_LOCALIZATION_LOCATION_DISTRIBUTION_H

#include <hssh/global_topological/global_location.h>
#include <vector>

namespace vulcan
{
namespace hssh
{
    
/**
* WeightedGlobalLocation defines a global location with an associated probability weight.
*/
struct WeightedGlobalLocation
{
    double probability;
    GlobalLocation location;
    
    WeightedGlobalLocation(double probability, const GlobalLocation& location)
    : probability(probability)
    , location(location)
    {
    }
    
    WeightedGlobalLocation(void) = default;
};
    

/**
* GlobalLocationDistribution represents a distribution across possible global locations for the robot. The default is
* that there's a single possible location, given a single action. However, if a probabilistic approach ever ends up
* being taken, then multiple possible locations weighted by probability could be encountered.
*/
class GlobalLocationDistribution
{
public:
    
    using Iter = std::vector<WeightedGlobalLocation>::const_iterator;
    
    /**
    * Constructor for GlobalLocationDistribution.
    * 
    * \pre  sum(l.probability in locations) == 1.0
    * \param    locations           Locations in the distribution
    */
    explicit GlobalLocationDistribution(const std::vector<WeightedGlobalLocation>& locations);
    
    /**
    * Constructor for GlobalLocationDistribution.
    * 
    * Create a distribution with a single location of probability=1. Useful for deterministic motion scenarios.
    * 
    * \param    location            Single location of the robot, which is probability of 1
    */
    explicit GlobalLocationDistribution(const GlobalLocation& location);
    
    /**
    * Default constructor for GlobalLocationDistribution.
    */
    GlobalLocationDistribution(void) = default;
    
    /**
    * isValid checks if the distribution is valid. An invalid distribution means that no location could exist for the
    * robot, given the current topological information. When isValid() == false, size() == 0, empty() == true.
    */
    bool isValid(void) const { return !empty(); }
    
    // Basic RandomAccess interface is provided
    std::size_t size(void) const { return locations_.size(); }
    bool empty(void) const { return locations_.empty(); }
    Iter begin(void) const { return locations_.begin(); }
    Iter end(void) const { return locations_.end(); }
    WeightedGlobalLocation operator[](int index) const { return locations_[index]; }
    WeightedGlobalLocation at(int index) const { return locations_.at(index); }

private:

    // INVARIANT: Sum of probability for locations is 1.0.
    std::vector<WeightedGlobalLocation> locations_;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_LOCALIZATION_LOCATION_DISTRIBUTION_H

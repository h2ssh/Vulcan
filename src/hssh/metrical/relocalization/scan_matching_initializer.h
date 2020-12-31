/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     scan_matching_initializer.h
 * \author   Collin Johnson
 *
 * Declaration of ScanMatchingInitializer.
 */

#ifndef HSSH_UTILS_METRICAL_RELOCALIZATION_SCAN_MATCHING_INITIALIZER_H
#define HSSH_UTILS_METRICAL_RELOCALIZATION_SCAN_MATCHING_INITIALIZER_H

#include "hssh/metrical/relocalization/filter_initializer.h"
#include <cereal/access.hpp>
#include <vector>

namespace vulcan
{
namespace hssh
{

/**
 * ScanMatchingInitializer initializes the relocalization process by combining the FreeSpaceFilterInitializer,
 * RegionFilterInitializer, and a simple error metric for determining how closely a laser scan matches the expected scan
 * taken from a given pose.
 *
 * The steps for scan-matching initializer are:
 *
 *   1) Generate a big chunk of samples using the free-space initializer.
 *   2) For each sample, use an error metric to find out how good the fit is.
 *   3) For the best N samples, draw samples using the region initializer.
 *
 * The error metric used works as follows:
 *
 *   1) Trace along the direction of each ray until hitting a wall. (r' = expected distance)
 *   2) Comapre the expected distance with the measured distance    (r  = measured distance)
 *   3) Error:
 *
 *       1.0 - (|r-r'| / r')
 */
class ScanMatchingInitializer : public FilterInitializer
{
public:
    ScanMatchingInitializer(void);

    // FilterInitializer interface
    virtual std::vector<particle_t> generateInitialSamples(const OccupancyGrid& grid,
                                                           const metric_slam_data_t& data) const override;

private:
    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        // No parameters here -- just need the correct class to be instantiated on the other side.
    }
};

}   // namespace hssh
}   // namespace vulcan

// Serialization support via smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::ScanMatchingInitializer)

#endif   // HSSH_UTILS_METRICAL_RELOCALIZATION_SCAN_MATCHING_INITIALIZER_H

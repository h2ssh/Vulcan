/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     filter_initializer.h
 * \author   Collin Johnson
 *
 * Definition of FilterInitializer interface.
 */

#ifndef HSSH_UTILS_METRICAL_RELOCALIZATION_FILTER_INITIALIZER_H
#define HSSH_UTILS_METRICAL_RELOCALIZATION_FILTER_INITIALIZER_H

#include <vector>

namespace vulcan
{
namespace hssh
{

struct particle_t;
struct metric_slam_data_t;
class OccupancyGrid;

/**
 * FilterInitializer is an interface for a class that generates the initial sample set of particles
 * for the particle filter used for relocalization. The class has a single method generateInitialSamples
 * that performs the desired task.
 */
class FilterInitializer
{
public:
    virtual ~FilterInitializer(void) { }

    /**
     * generateInitialSamples generates a sample set to initialize the relocalization particle filter.
     *
     * \param    grid            Map in which the samples will exist
     * \param    data            Data to use for the generation
     * \return   A set of particles to use to initialize the particle filter.
     */
    virtual std::vector<particle_t> generateInitialSamples(const OccupancyGrid& grid,
                                                           const metric_slam_data_t& data) const = 0;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_UTILS_METRICAL_RELOCALIZATION_FILTER_INITIALIZER_H

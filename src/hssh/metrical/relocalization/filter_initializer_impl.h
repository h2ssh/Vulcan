/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     filter_initializer_impl.h
 * \author   Collin Johnson
 *
 * Declaration of implementations of the FilterInitializer interface:
 *
 *   - RegionFilterInitializer
 *   - FreeSpaceFilterInitializer
 */

#ifndef HSSH_UTILS_METRICAL_RELOCALIZATION_FILTER_INITIALIZER_IMPL_H
#define HSSH_UTILS_METRICAL_RELOCALIZATION_FILTER_INITIALIZER_IMPL_H

#include "hssh/metrical/relocalization/filter_initializer.h"
#include "math/geometry/rectangle.h"
#include <cereal/access.hpp>

namespace vulcan
{
namespace hssh
{

struct particle_t;
class OccupancyGrid;

/**
 * RegionFilterInitializer spreads a uniform distribution of particles within a rectangular region. The samples
 * are evenly spaced throughout the region. The orientation of the particles is randomly generated.
 */
class RegionFilterInitializer : public FilterInitializer
{
public:
    /**
     * Constructor for RegionFilterInitializer.
     *
     * \param    region              Region in the map in which the robot is theorized to be
     * \param    numPositions        Number of positions at which to place samples
     * \param    posesPerPosition    Number of poses (pos, theta) to generate at each position
     */
    RegionFilterInitializer(const math::Rectangle<float>& region, int numPositions, int posesPerPosition);

    // FilterInitializer interface
    virtual std::vector<particle_t> generateInitialSamples(const OccupancyGrid& grid,
                                                           const metric_slam_data_t& data) const override;

private:
    math::Rectangle<float> region_;
    std::size_t numPositions_;
    int posesPerPosition_;

    // Serialization support
    friend class cereal::access;

    RegionFilterInitializer(void) { }

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar& region_;
        ar& numPositions_;
        ar& posesPerPosition_;
    }
};

/**
 * FreeSpaceFilterInitializer spreads samples throughout the free space in the OccupancyGrid. The orientation of the
 * particles is randomly generated.
 */
class FreeSpaceFilterInitializer : public FilterInitializer
{
public:
    /**
     * Constructor for FreeSpaceFilterInitializer
     *
     * \param    stride              Number of cells to jump when creating the particles
     * \param    posesPerCell        Number of poses to generate for each cell
     */
    FreeSpaceFilterInitializer(int stride, int posesPerCell);

    // FilterInitializer interface
    virtual std::vector<particle_t> generateInitialSamples(const OccupancyGrid& grid,
                                                           const metric_slam_data_t& data) const override;

private:
    int stride_;
    int posesPerCell_;

    // Serialization support
    friend class cereal::access;

    FreeSpaceFilterInitializer(void) { }

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar& stride_;
        ar& posesPerCell_;
    }
};

}   // namespace hssh
}   // namespace vulcan

// Serialization support for smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::FreeSpaceFilterInitializer)
CEREAL_REGISTER_TYPE(vulcan::hssh::RegionFilterInitializer)

#endif   // HSSH_UTILS_METRICAL_RELOCALIZATION_FILTER_INITIALIZER_IMPL_H

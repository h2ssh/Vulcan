/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     destination.h
 * \author   Collin Johnson
 *
 * Declaration of LocalDestination.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREAS_DESTINATION_H
#define HSSH_LOCAL_TOPOLOGICAL_AREAS_DESTINATION_H

#include "hssh/local_topological/affordances/transition.h"
#include "hssh/local_topological/areas/place.h"
#include <cereal/access.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace hssh
{

/**
 * LocalDestination describes a destination in small-scale space. A destination is a non-decision point
 * place. Destinations occur at places like offices that are accessible from paths, but aren't at the endpoints of
 * any path segments.
 *
 * In addition to the metric map of a LocalPlace, a destination contains a set of travel affordances that carry the
 * robot from the destination to any other type of area. The important bit here is that any LocalArea can be connected
 * to a destination.
 */
class LocalDestination : public LocalPlace
{
public:
    /**
     * Constructor for LocalDestination.
     *
     * \param    star            Small-scale star describing the topology of the place
     * \param    map             Metric map of the place
     * \param    id              Unique id for the area
     * \param    extent          Physical extent of the area in the world
     * \param    gateways        All gateways in the destination
     */
    LocalDestination(const SmallScaleStar& star,
                     const LocalPerceptualMap& map,
                     int id,
                     const AreaExtent& extent,
                     const std::vector<Gateway>& gateways);

    ////////////////////////////// LocalArea interface //////////////////////////

    AreaType type(void) const override { return AreaType::destination; }
    std::string description(void) const override;
    void accept(LocalAreaVisitor& visitor) const override;

private:
    // Serialization support
    LocalDestination(void) { }

    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::base_class<LocalPlace>(this));
    }
};

}   // namespace hssh
}   // namespace vulcan

// Serialization support for smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::hssh::LocalDestination)

#endif   // HSSH_LOCAL_TOPOLOGICAL_AREAS_DESTINATION_H

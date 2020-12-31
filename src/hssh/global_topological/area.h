/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     area.h
 * \author   Collin Johnson
 *
 * Declaration of GlobalArea along with operator==,!=, and <<.
 */

#ifndef HSSH_GLOBAL_TOPOLOGICAL_AREA_H
#define HSSH_GLOBAL_TOPOLOGICAL_AREA_H

#include "hssh/types.h"
#include "hssh/utils/id.h"
#include <cassert>
#include <cereal/access.hpp>
#include <iosfwd>

namespace vulcan
{
namespace hssh
{

/**
 * GlobalArea define the basic properties of an area in the global topological map. Each GlobalArea has the following
 * properties:
 *
 *   - a unique id : id is globally unique amongst all possible topological maps
 *   - a type      : a descriptor indicating the type of area associated with this id, which can be used to
 *                   access the correct instantation for within a topological map
 */
class GlobalArea
{
public:
    /**
     * Constructor for GlobalArea.
     *
     * \param    id          Unique id to assign to the area
     * \param    type        Specific type of the area
     * \pre  type == path_segment || type == path || type == destination || type == decision_point
     */
    GlobalArea(Id id, AreaType type);

    /**
     * Constructor for GlobalArea.
     *
     * Creates a frontier GlobalArea. The specific area is not bound because it hasn't been explored yet.
     *
     * \param    type            Specific type of the area
     * \pre  type == path_segment || type == path || type == destination || type == decision_point
     */
    explicit GlobalArea(AreaType type);

    // The default constructor creates an area with an invalid id
    GlobalArea(void) = default;

    /**
     * id retrieves the id assigned to the area.
     */
    Id id(void) const { return id_; }

    /**
     * type retreives the type of the area.
     */
    AreaType type(void) const { return type_; }

private:
    Id id_ = kInvalidId;
    AreaType type_ = AreaType::area;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(id_, type_);
    }
};

// Operators for GlobalArea
/**
 * Equality operator: Two areas are the same if they have the same id and the same type.
 */
bool operator==(const GlobalArea& lhs, const GlobalArea& rhs);
bool operator!=(const GlobalArea& lhs, const GlobalArea& rhs);

/**
 * Output operator: Output is id::type.
 */
std::ostream& operator<<(std::ostream& out, const GlobalArea& area);

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_GLOBAL_TOPOLOGICAL_AREA_H

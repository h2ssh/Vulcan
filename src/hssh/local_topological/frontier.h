/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     frontier.h
 * \author   Collin Johnson
 *
 * Declaration of Frontier.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_FRONTIER_H
#define HSSH_LOCAL_TOPOLOGICAL_FRONTIER_H

#include "core/line.h"
#include "hssh/types.h"
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <cstdint>

namespace vulcan
{
namespace hssh
{

/**
 * Frontier defines a frontier, which is an exit from the LPM. A frontier is indicated
 * by a line segment that contains the associated skeleton anchors. The actual point of exit
 * along the Voronoi graph is also provided and can be used for determining the LocalPath.
 *
 * The direction of the frontier is the direction of the Voronoi graph as it enters the
 * exit point.
 *
 * The frontier is in metric coordinates, not grid coordinates.
 */
class Frontier
{
public:
    int64_t timestamp = -1;   ///< Timestamp of the map with which this frontier is associated
    int32_t id = -1;          ///< Unique identifier for the fronter in the current map

    std::vector<cell_t>
      cells;   ///< Ordered sequence of cells in the frontier. The first and last cell are adjacent to occupied cells.

    Point<float> exitPoint;   ///< Exit point that generated this frontier
    Line<int> boundary;       ///< Approximate boundary of the frontier
    float direction = 0.0f;   ///< Direction of space emanating from the frontier

    Frontier(void) = default;

    Frontier(int32_t id) : id(id) { }

private:
    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(timestamp, id, cells, exitPoint, boundary, direction);
    }
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_FRONTIER_H

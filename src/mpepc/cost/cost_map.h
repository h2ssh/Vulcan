/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     cost_map.h
* \author   Collin Johnson
*
* Definition of CostMap for use in constructing the NavigationGrid.
*/

#ifndef MPEPC_COSTS_COST_MAP_H
#define MPEPC_COSTS_COST_MAP_H

#include <utils/cell_grid.h>
#include <system/message_traits.h>
#include <cereal/types/base_class.hpp>

namespace vulcan
{
namespace mpepc
{

const int32_t kMinCollisionCost = 1000000;  // all collisions have a cost at least this high
const int32_t kNonRouteCost = 1000;

/**
* CostMap is just a simple CellGrid with an Id and timestamp added. No special processing occurs within the
* CostMap itself.
*
* The cost map maintains a maximum non-obstacle cost for easier rendering or other processing.
*/
class CostMap : public utils::CellGrid<int32_t>
{
public:

    // Inherit all base class constructors
    using CellGrid<int32_t>::CellGrid;

    void setTimestamp(int64_t timestamp) { timestamp_ = timestamp; }
    int32_t getTimestamp(void) const { return timestamp_ ; }

    void setId(int32_t id) { id_ = id; }
    int32_t getId(void) const { return id_ ; }

    int32_t getMaxCost(void) const { return maxCost_; }
    void resetMaxCost(void) { maxCost_ = 0; }

    void setCost(int x, int y, int32_t cost)
    {
        setValue(x, y, cost);

        if((cost > maxCost_) && (cost < kMinCollisionCost))
        {
            maxCost_ = cost;
        }
    }

    void addCost(int x, int y, int32_t cost)
    {
        int32_t& cellCost = (*this)(x, y);
        cellCost += cost;

        if((cellCost > maxCost_) && (cellCost < kMinCollisionCost))
        {
            maxCost_ = cellCost;
        }
    }

    void setMinCost(int x, int y, int32_t cost)
    {
        int32_t cellCost = std::min(cost, getValueNoCheck(x, y));
        setValueNoCheck(x, y, cellCost);

        if((cellCost > maxCost_) && (cellCost < kMinCollisionCost))
        {
            maxCost_ = cellCost;
        }
    }

    void setMaxCost(int x, int y, int32_t cost)
    {
        int32_t cellCost = std::max(cost, getValueNoCheck(x, y));
        setValueNoCheck(x, y, cellCost);

        if((cellCost > maxCost_) && (cellCost < kMinCollisionCost))
        {
            maxCost_ = cellCost;
        }
    }

private:

    int64_t timestamp_;
    int32_t id_;
    int32_t maxCost_ = 0;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void save(Archive& ar) const
    {
        ar( cereal::base_class<utils::CellGrid<int32_t>>(this),
            timestamp_,
            id_,
            maxCost_);
    }

    template <class Archive>
    void load(Archive& ar)
    {
        ar( cereal::base_class<utils::CellGrid<int32_t>>(this),
            timestamp_,
            id_,
            maxCost_);
    }
};

} // namespace mpepc
} // namespace vulcan

DEFINE_DEBUG_MESSAGE(mpepc::CostMap, ("DEBUG_MPEPC_COST_MAP"))

#endif // MPEPC_COSTS_COST_MAP_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_map.cpp
* \author   Collin Johnson
*
* Definition of LocalTopoMap.
*/

#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/areas/path_segment.h"
#include "hssh/local_topological/areas/place.h"
#include "utils/algorithm_ext.h"
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>
#include <algorithm>

namespace vulcan
{
namespace hssh
{

void set_lambda(LocalPathSegment& segment, const LocalTopoMap& map);


LocalTopoMap::LocalTopoMap(void)
: id_(-1)
, timestamp_(0)
{
    // Can safely do nothing because all it means is the various structures have no members
}


LocalTopoMap::LocalTopoMap(int32_t id,
                           int64_t timestamp,
                           const LocalAreaVec& areas,
                           const VoronoiSkeletonGrid& skeleton,
                           double logProb)
: id_(id)
, timestamp_(timestamp)
, logProb_(logProb)
, skeleton_(skeleton)
{
    // Only add an area if it isn't nullptr
    areas_.reserve(areas.size());
    std::copy_if(areas.begin(),
                 areas.end(),
                 std::back_inserter(areas_),
                 [](const LocalAreaPtr& p)
                 {
                     return p != nullptr;
                 });

    // Establish an edge between all adjacent areas
    std::unordered_map<int32_t, int> gatewayToArea;
    for(std::size_t n = 0; n < areas_.size(); ++n)
    {
        // For each gateway in an area, map from the gateway id to the area index
        for(const auto& g : areas_[n]->gateways())
        {
            auto areaIt = gatewayToArea.find(g.id());

            // If the gateway already exists, then the other side of the gateway is found so the
            // index of each area should be mapped to the other
            if(areaIt != gatewayToArea.end())
            {
                int adjacentAreaId = areaIt->second;
                areaToNeighbors_.insert(std::make_pair(areas_[n]->id(), adjacentAreaId));
                areaToNeighbors_.insert(std::make_pair(adjacentAreaId, areas_[n]->id()));
            }
            // Otherwise add the gateway id -> area index map
            else
            {
                gatewayToArea.insert(std::make_pair(g.id(), areas_[n]->id()));
            }
        }
    }

    for(auto& a : areas_)
    {
        if(a->type() == AreaType::path_segment)
        {
            // FIX: Make sure auto& so that set_lambda isn't called on a copy!
            auto& segment = static_cast<LocalPathSegment&>(*a);
            set_lambda(segment, *this);
        }
    }
}


LocalAreaPtr LocalTopoMap::areaWithId(int32_t id) const
{
    auto areaIt = std::find_if(areas_.begin(), areas_.end(), [id](const LocalAreaPtr& a)
        { return a->id() == id; });

    return (areaIt != areas_.end()) ? *areaIt : nullptr;
}


std::shared_ptr<LocalPathSegment> LocalTopoMap::pathSegmentWithId(int32_t id) const
{
    auto area = areaWithId(id);

    if(!area || (area->type() != AreaType::path_segment))
    {
        return nullptr;
    }

    return std::static_pointer_cast<LocalPathSegment>(area);
}


std::shared_ptr<LocalDecisionPoint> LocalTopoMap::decisionPointWithId(int32_t id) const
{
    auto area = areaWithId(id);

    if(!area || (area->type() != AreaType::decision_point))
    {
        return nullptr;
    }

    return std::static_pointer_cast<LocalDecisionPoint>(area);
}


std::shared_ptr<LocalDestination> LocalTopoMap::destinationWithId(int32_t id) const
{
    auto area = areaWithId(id);

    if(!area || (area->type() != AreaType::destination))
    {
        return nullptr;
    }

    return std::static_pointer_cast<LocalDestination>(area);
}


LocalAreaVec LocalTopoMap::allAreasContaining(const Point<float>& position) const
{
    // The containing area is the one with this position inside it
    // Use the contains method to defer exactly what contains means to the implementation of LocalArea
    LocalAreaVec containing;

    for(const auto& a : areas_)
    {
        if(a->contains(position) && a->containsCell(position))
        {
            containing.push_back(a);
        }
    }

    return containing;
}


LocalAreaPtr LocalTopoMap::areaContaining(const Point<float>& position) const
{
    // The containing area is the one with this position inside it
    // Use the contains method to defer exactly what contains means to the implementation of LocalArea
    for(const auto& a : areas_)
    {
        if(a->contains(position) && a->containsCell(position))
        {
            return a;
        }
    }

    return nullptr;
}


LocalAreaVec LocalTopoMap::neighborsOf(const LocalArea& area) const
{
    auto adjacentIndices = areaToNeighbors_.equal_range(area.id());

    // Neighbors are those on the other side of gateways from the provided area
    // The areas are mapped to their neighbors, acquire the relevant neighbor area indices
    // Convert the indices into the actual pointers-to-area
    LocalAreaVec adjacentAreas;
    std::transform(adjacentIndices.first,
                   adjacentIndices.second,
                   std::back_inserter(adjacentAreas),
                   [this](const std::pair<const int, std::size_t>& index) {
        return areaWithId(index.second);
    });

    return adjacentAreas;
}


LocalAreaPtr LocalTopoMap::otherSideOfGateway(const Gateway& gateway, const LocalArea& area) const
{
    // Find the neighbors of the area
    // If any of the neighbors contains this gateway, then its on the other side
    auto neighborAreas = neighborsOf(area);
    auto otherSide = std::find_if(neighborAreas.begin(), neighborAreas.end(), [&gateway](const LocalAreaPtr& p) {
        return p->hasGateway(gateway);
    });

    return (otherSide != neighborAreas.end()) ? *otherSide : nullptr;
}


std::vector<Gateway> LocalTopoMap::gateways(void) const
{
    std::vector<Gateway> gateways;

    for(auto& a : areas_)
    {
        boost::push_back(gateways, boost::as_array(a->gateways()));
    }

    std::sort(gateways.begin(), gateways.end());
    utils::erase_unique(gateways);
    return gateways;
}


LocalAreaVec LocalTopoMap::areasAdjacentToGateway(const Gateway& gateway) const
{
    LocalAreaVec adjacent;

    for(auto& a : areas_)
    {
        if(a->hasGateway(gateway))
        {
            adjacent.push_back(a);
        }
    }

    return adjacent;
}


void LocalTopoMap::visitAreas(LocalAreaVisitor& visitor) const
{
    for(auto& a : areas_)
    {
        a->accept(visitor);
    }
}


void LocalTopoMap::clearSkeleton(void)
{
    skeleton_ = VoronoiSkeletonGrid();
}


void set_lambda(LocalPathSegment& segment, const LocalTopoMap& map)
{
    auto minusArea = map.otherSideOfGateway(segment.minusTransition().gateway(), segment);
    auto plusArea = map.otherSideOfGateway(segment.plusTransition().gateway(), segment);

    // Only compute the lambda if both plus and minus are available, otherwise the stored value
    // is as good as can be done for now.
    if(minusArea && plusArea)
    {
        std::cout << "Setting lambda for " << segment.boundary(math::ReferenceFrame::GLOBAL) << '\n';
        Lambda lambda(minusArea->center(), plusArea->center());
        segment.setLambda(lambda);
    }
}

} // namespace hssh
} // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     local_topo_map.h
 * \author   Collin Johnson
 *
 * Declaration of LocalTopoMap.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_LOCAL_AREA_MAP_H
#define HSSH_LOCAL_TOPOLOGICAL_LOCAL_AREA_MAP_H

#include "core/point.h"
#include "hssh/local_topological/areas/serialization.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "system/message_traits.h"
#include <cereal/access.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace hssh
{

class Gateway;
class LocalAreaVisitor;

using LocalAreaPtr = std::shared_ptr<LocalArea>;
using LocalAreaVec = std::vector<LocalAreaPtr>;

/**
 * LocalTopoMap provides a symbolic representation of small-scale space. The map contains a set of distinct,
 * non-overlapping areas. The LocalTopoMap provides the bridge between the LPM and the global topological map by
 * grounding the symbols used in the topological map in the LPM. This grounding is achieved using gateways. Each gateway
 * represents an affordance for transitioning between adjacent areas, thereby converting the route planned in the
 * global topological map into a well-defined pose targets for a motion planner.
 *
 * The LocalTopoMap supports querying for an area given a position in the LPM. The neighbors of this area can be found
 * along with the area on the other side of a gateway. A simple visitation API allows for visiting each area in the map.
 */
class LocalTopoMap
{
public:
    /**
     * Default constructor for LocalTopoMap.
     *
     * Creates an empty map.
     */
    LocalTopoMap(void);

    /**
     * Constructor for LocalTopoMap.
     *
     * Build a map from a collection of areas within an LPM.
     *
     * \param    id              Id to assign to the map
     * \param    timestamp       Timestamp for which data is incorporated into this map
     * \param    areas           Areas in the LPM
     * \param    skeleton        Voronoi skeleton of the environment for this LocalTopoMap
     * \param    logProb         Log-probability of this map (optional, default = 0.0)
     */
    LocalTopoMap(int32_t id,
                 int64_t timestamp,
                 const LocalAreaVec& areas,
                 const VoronoiSkeletonGrid& skeleton,
                 double logProb = -100000.0);

    /**
     * mapId retrieves the id of this map.
     */
    int32_t mapId(void) const { return id_; }

    /**
     * timestamp retrieves the latest timestamp for the data included in this map.
     */
    int64_t timestamp(void) const { return timestamp_; }

    /**
     * logProb retrieves the log probability of the computed map.
     */
    double logProb(void) const { return logProb_; }

    /**
     * areaWithId retrieves the area with the provided id.
     *
     * \return   The associated area, or nullptr if no such area exists.
     */
    LocalAreaPtr areaWithId(int32_t id) const;

    /**
     * Retrieves the path segment with the provided id.
     *
     * \return   The associated path segment, or nullptr if no such area exists or the area has a type other than path
     *  segment.
     */
    std::shared_ptr<LocalPathSegment> pathSegmentWithId(int32_t id) const;

    /**
     * Retrieves the decision point with the provided id.
     *
     * \return   The associated decision point, or nullptr if no such area exists or the area has a type other than
     *  decision point.
     */
    std::shared_ptr<LocalDecisionPoint> decisionPointWithId(int32_t id) const;

    /**
     * Retrieves the destination with the provided id.
     *
     * \return   The associated destination, or nullptr if no such area exists or the area has a type other than
     *  destination.
     */
    std::shared_ptr<LocalDestination> destinationWithId(int32_t id) const;

    /**
     * allAreasContaining searches through the map to find any areas whose approximate boundary contains the specified
     * position in the current LPM. The position is in the LPM's reference frame.
     *
     * The result will usually be a single area. However, near the edge of an area, the boundary can subtly shift
     * between updates or the convex hull approximation can result in the true boundary being slightly smaller. As such,
     * any method will need to handle the case where multiple areas are returned.
     *
     * \param    position            Position to check for an area
     * \return   All areas whose approximate boundary contains the specified position.
     */
    LocalAreaVec allAreasContaining(const Point<float>& position) const;

    /**
     * Search through the map to find the area whose boundary contains the specified position.
     * The position is in the map's reference frame.
     *
     * Two areas may share cells along their boundary. In these cases, the first area stored in the map will be
     * returned. The answer will be consistent, but this means they may be slightly flickering exactly on the gateway
     * boundary of two areas.
     *
     * \param    position            Position to check for an area
     * \return   The area containing this position.
     */
    LocalAreaPtr areaContaining(const Point<float>& position) const;

    /**
     * neighborsOf finds all neighbors of the specified area. If the area has no neighbors because it is the only area
     * in the map, then the vector will be empty.
     *
     * \param    area            Area for which to find the neighbors
     * \return   Neighbors of the area, i.e. those on the other side of the gateways.
     */
    LocalAreaVec neighborsOf(const LocalArea& area) const;

    /**
     * otherSideOfGateway finds the area on the other side of the gateway from the supplied area. If the gateway doesn't
     * belong to the area, then a nullptr is returned.
     */
    LocalAreaPtr otherSideOfGateway(const Gateway& gateway, const LocalArea& area) const;

    /**
     * gateways retrieves all gateways contained in the map.
     */
    std::vector<Gateway> gateways(void) const;

    /**
     * areasAdjacentToGateway retrieves the area(s) adjacent to a particular gateway. There are one or two areas,
     * depending on whether or not the gateway is a false gateway at a frontier/dead end or a true boundary between
     * two areas.
     *
     * \param    gateway         Gateway to find adjacent areas to
     * \return   Area(s) adjacent to the gateway.
     */
    LocalAreaVec areasAdjacentToGateway(const Gateway& gateway) const;

    /**
     * voronoiSkeleton retrieves the Voronoi skeleton of the environment for the LocalTopoMap.
     */
    const VoronoiSkeletonGrid& voronoiSkeleton(void) const { return skeleton_; }

    /**
     * visitAreas passes the visitor to each area in the map.
     *
     * \param    visitor         Implementation of the LocalAreaVisitor interface
     */
    void visitAreas(LocalAreaVisitor& visitor) const;

    // Iterate over the areas in the map
    LocalAreaVec::const_iterator begin(void) const { return areas_.cbegin(); }
    LocalAreaVec::const_iterator end(void) const { return areas_.cend(); }
    std::size_t size(void) const { return areas_.size(); }

    // Mutators for reducing memory consumption

    /**
     * clearSkeleton clears out memory associated with the VoronoiSkeletonGrid. If it isn't used, clearing the grid
     * can potentially save a lot of memory.
     */
    void clearSkeleton(void);

private:
    // INVARIANT: No nullptr exist in areas_
    // INVARIANT: Every key and value in areaToNeighbors_ is a valid index in areas_
    int32_t id_;
    int64_t timestamp_;
    double logProb_ = -100000.0;
    std::vector<std::shared_ptr<LocalArea>> areas_;
    std::multimap<int, std::size_t> areaToNeighbors_;
    VoronoiSkeletonGrid skeleton_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar(id_, timestamp_, areas_, areaToNeighbors_, skeleton_);

        if (version == 1) {
            ar(logProb_);
        }
    }
};

}   // namespace hssh
}   // namespace vulcan

CEREAL_CLASS_VERSION(vulcan::hssh::LocalTopoMap, 1)

DEFINE_SYSTEM_MESSAGE(hssh::LocalTopoMap, ("HSSH_LOCAL_TOPO_MAP"))

#endif   // HSSH_LOCAL_TOPOLOGICAL_LOCAL_AREA_MAP_H

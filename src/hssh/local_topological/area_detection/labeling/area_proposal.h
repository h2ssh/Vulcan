/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     area_proposal.h
 * \author   Collin Johnson
 *
 * Declaration of AreaProposal.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREAS_AREA_PROPOSAL_H
#define HSSH_LOCAL_TOPOLOGICAL_AREAS_AREA_PROPOSAL_H

#include "hssh/local_topological/area.h"
#include "hssh/local_topological/area_extent.h"
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace hssh
{

class VoronoiSkeletonGrid;

/**
 * AreaProposal
 *
 * The coordinates for all members of the AreaProposal are assumed to be in the reference frame of the
 * current LPM. The coordinates are NOT relative to the center of the AreaProposal. The relative coordinates
 * are only established for the final LocalAreas.
 */
class AreaProposal
{
public:
    /**
     * path_endpoint_t describes an endpoint for a proposal that is a path. It's either a gateway or some frontier.
     * Only proposal's whose type == PATH_SEGMENT have valid endpoints.
     */
    struct path_endpoint_t
    {
        AreaType type;   // area, dead_end, frontier
        bool isGatewayEndpoint;
        Gateway gateway;
        Point<double> point;
    };


    /**
     * Default constructor for AreaProposal.
     */
    AreaProposal(void) : type(AreaType::area), onFrontier(true), haveEndpoints(false) { }

    /**
     * Constructor for AreaProposal.
     *
     * Creates an AreaProposal representing a path segment.
     *
     * \param    id                      Unique identifier for the proposal
     * \param    plus                    Endpoint at the plus end
     * \param    minus                   Endpoint at the minus end
     * \param    isFrontier              Flag indicating if the proposal is of a frontier area
     * \param    isBoundary              Flag indicating if the proposal exists on the boundary of the map
     * \param    extent                  Extent of the proposed area
     * \param    paths                   Path gateways
     * \param    destinations            Destination gateways
     * \param    decisions               Decision point gateways
     * \param    frontiers               Location of the frontiers in the area
     * \param    deadEnds                Location of the dead ends in the area
     * \throws InvalidAreaException
     */
    AreaProposal(int id,
                 path_endpoint_t plus,
                 path_endpoint_t minus,
                 bool isFrontier,
                 bool isBoundary,
                 const AreaExtent& extent,
                 const std::vector<Gateway>& paths,
                 const std::vector<Gateway>& destinations,
                 const std::vector<Gateway>& decisions,
                 const std::vector<Point<double>>& frontiers,
                 const std::vector<Point<double>>& deadEnds);

    /**
     * Constructor for AreaProposal.
     *
     * Creates an AreaProposal representing a place.
     *
     * \param    id                      Unique identifier for the proposal
     * \param    type                    Type of area represented by the proposal
     * \param    frontier                Flag indicating if the proposal is of a frontier area
     * \param    boundary                Flag indicating if the proposal exists on the boundary of the map
     * \param    extent                  Extent of the proposed area
     * \param    paths                   Path gateways
     * \param    destinations            Destination gateways
     * \param    decisions               Decision point gateways
     * \param    frontiers               Location of the frontiers in the area
     * \param    deadEnds                Location of the dead ends in the area
     * \throws InvalidAreaException
     */
    AreaProposal(int id,
                 AreaType type,
                 bool frontier,
                 bool boundary,
                 const AreaExtent& extent,
                 const std::vector<Gateway>& paths,
                 const std::vector<Gateway>& destinations,
                 const std::vector<Gateway>& decisions,
                 const std::vector<Point<double>>& frontiers,
                 const std::vector<Point<double>>& deadEnds);

    // Operator overloads
    /**
     * equality operator overload. Two proposals are considered to be equal if:
     *
     *   - their gateways match and the center of the proposal is in the same position relative
     *     to each gateway, i.e. if two gateways match, they have similar boundaries and the
     *     main portion of the area is on the same side of the gateway
     *
     *   -
     */
    bool operator==(const AreaProposal& rhs) const;

    /**
     * inequality operator overload.
     *
     * \param    rhs         AreaProposal on right-hand of '!='
     * \return   True if the proposals are not equal as discussed in the operator== description.
     */
    bool operator!=(const AreaProposal& rhs) const { return !operator==(rhs); }

    /**
     * contains checks to see if a particular point is contained in the area.
     *
     * \param    point       Point to check if inside
     * \return   True if point falls within the extent of the area.
     */
    bool contains(const Point<double>& point) const;

    /**
     * getId retrieves the id of the area proposal.
     */
    int getId(void) const { return id_; }

    /**
     * getType retrieves the classification assigned to this proposal.
     */
    AreaType getType(void) const { return type; }

    /**
     * isFrontier asks if this area is a frontier.
     */
    bool isFrontier(void) const { return onFrontier; }

    /**
     * isOnBoundary checks if the area exists on the boundary of the map, and is therefore incomplete.
     */
    bool isOnBoundary(void) const { return onBoundary; }

    /**
     * getGatewayType determines which type of area a particular gateway leads to.
     *
     * \param    g           Gateway to find the type for
     * \return   The type of area on the other side of the gateway. AreaType::area if no such gateway exists.
     */
    AreaType getGatewayType(const Gateway& g) const;

    /**
    * getExtent retrieves the calculated extent of the place. If no extent has been calculated
A    * yet, then the extent will have a zero area boundary.
    */
    AreaExtent getExtent(void) const { return extent; }

    /**
     * getGlobalBoundary retrieves the rectangle boundary of the area in the global coordinates, not
     * relative to the proposal center.
     */
    math::Rectangle<float> getGlobalBoundary(void) const;

    /**
     * getCenter retrieves the center pose of the area.
     */
    pose_t getCenter(void) const { return extent.center(); }

    /**
     * getGlobalGateways retrieves all gateways associated with the proposal transformed back into the global reference
     * frame of the provided grid.
     *
     * \param    grid            Grid whose reference frame the gateways will be in
     * \param    frame           Reference frame for the gateways being retrieved  (default = GLOBAL)
     * \return   Gateways in the grid reference frame.
     */
    std::vector<Gateway> getAllGateways(const VoronoiSkeletonGrid& grid,
                                        math::ReferenceFrame frame = math::ReferenceFrame::GLOBAL) const;

    /**
     * getPathGateways retrieves all gateways the lead TO a path segment.
     *
     * \param    frame           Reference frame for the gateways being retrieved  (default = GLOBAL)
     */
    std::vector<Gateway> getPathGateways(math::ReferenceFrame frame = math::ReferenceFrame::GLOBAL) const
    {
        return getGatewaysOfType(AreaType::path_segment, frame);
    }

    /**
     * getDestinationGateways retrieves all gateways leading TO a destination.
     *
     * \param    frame           Reference frame for the gateways being retrieved  (default = GLOBAL)
     */
    std::vector<Gateway> getDestinationGateways(math::ReferenceFrame frame = math::ReferenceFrame::GLOBAL) const
    {
        return getGatewaysOfType(AreaType::destination, frame);
    }

    /**
     * getDecisionGateways retrieves all gateways leading TO a decision point.
     *
     * \param    frame           Reference frame for the gateways being retrieved  (default = GLOBAL)
     */
    std::vector<Gateway> getDecisionGateways(math::ReferenceFrame frame = math::ReferenceFrame::GLOBAL) const
    {
        return getGatewaysOfType(AreaType::decision_point, frame);
    }

    /**
     * getFrontiers retrieves the frontiers associated with the area.
     */
    std::vector<Point<double>> getFrontiers(void) const { return frontiers; }

    /**
     * getDeadEnds retrieves the dead ends associated with the area.
     */
    std::vector<Point<double>> getDeadEnds(void) const { return deadEnds; }

    /**
     * getPathEndpoints retrieves the endpoints of the proposal if it is a PATH_SEGMENT. Otherwise, there won't
     * be any path segments.
     */
    std::vector<path_endpoint_t> getPathEndpoints(void) const { return pathEndpoints; }

private:
    struct proposal_gateway_t
    {
        Gateway gateway;
        AreaType type;   // Type of area this gateway leads to

        bool operator==(const Gateway& g) const { return gateway == g; }

        template <class Archive>
        void serialize(Archive& ar)
        {
            ar(type, gateway);
        }
    };

    int id_;
    AreaType type;
    bool onFrontier;
    bool onBoundary;
    math::ReferenceFrame frame;
    std::vector<proposal_gateway_t> gateways;
    std::vector<Point<double>> frontiers;
    std::vector<Point<double>> deadEnds;
    AreaExtent extent;

    std::vector<path_endpoint_t> pathEndpoints;
    bool haveEndpoints;

    void addGateways(const std::vector<Gateway>& toAdd, AreaType type);
    AreaType gatewayType(const Gateway& g) const;

    void changeGatewayReferenceFrames(const pose_t& transform, const VoronoiSkeletonGrid& grid);
    void changePointReferenceFrames(std::vector<Point<double>>& points, const pose_t& trans);

    std::vector<Gateway> getGatewaysOfType(AreaType type, math::ReferenceFrame frame) const;
    bool hasInvalidGatewayAssociation(const Gateway& gateway,
                                      const std::vector<proposal_gateway_t>& rhsGateways,
                                      const AreaExtent& rhsExtent) const;

    void enforceInvariant(void);

    // Serialization support
    friend class ::cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(id_,
           type,
           onFrontier,
           onBoundary,
           frame,
           gateways,
           frontiers,
           deadEnds,
           extent,
           pathEndpoints,
           haveEndpoints);
    }
};


template <class Archive>
void serialize(Archive& ar, AreaProposal::path_endpoint_t& endpoint, const unsigned int version)
{
    ar(endpoint.gateway, endpoint.isGatewayEndpoint, endpoint.point);
}

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_AREAS_AREA_PROPOSAL_H

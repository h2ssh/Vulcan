/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_proposal.cpp
* \author   Collin Johnson
*
* Definition of AreaProposal.
*/

#include <hssh/local_topological/area_detection/labeling/area_proposal.h>
#include <hssh/local_topological/area_detection/labeling/invalid_area.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <iostream>


namespace vulcan
{
namespace hssh
{

void print_proposal_info(const std::vector<Gateway>& gateways, const std::vector<Point<double>>& frontiers);


AreaProposal::AreaProposal(int id,
                           path_endpoint_t plus,
                           path_endpoint_t minus,
                           bool isFrontier,
                           bool isBoundary,
                           const AreaExtent& extent,
                           const std::vector<Gateway>& paths,
                           const std::vector<Gateway>& destinations,
                           const std::vector<Gateway>& decisions,
                           const std::vector<Point<double>>& frontiers,
                           const std::vector<Point<double>>& deadEnds)
: id_(id)
, type(AreaType::path_segment)
, onFrontier(isFrontier)
, onBoundary(isBoundary)
, frame(math::ReferenceFrame::GLOBAL)
, frontiers(std::move(frontiers))
, deadEnds(std::move(deadEnds))
, extent(std::move(extent))
, pathEndpoints{plus, minus}
, haveEndpoints(true)
{
    addGateways(paths,        AreaType::path_segment);
    addGateways(destinations, AreaType::destination);
    addGateways(decisions,    AreaType::decision_point);
    enforceInvariant();
}


AreaProposal::AreaProposal(int id,
                           AreaType                                type,
                           bool                                    frontier,
                           bool                                    boundary,
                           const AreaExtent&                       extent,
                           const std::vector<Gateway>&             paths,
                           const std::vector<Gateway>&             destinations,
                           const std::vector<Gateway>&             decisions,
                           const std::vector<Point<double>>& frontiers,
                           const std::vector<Point<double>>& deadEnds)
: id_(id)
, type(type)
, onFrontier(frontier)
, onBoundary(boundary)
, frame(math::ReferenceFrame::GLOBAL)
, frontiers(frontiers.begin(), frontiers.end())
, deadEnds(deadEnds.begin(), deadEnds.end())
, extent(extent)
, haveEndpoints(false)
{
    assert(type != AreaType::path_segment);

    addGateways(paths,        AreaType::path_segment);
    addGateways(destinations, AreaType::destination);
    addGateways(decisions,    AreaType::decision_point);
    enforceInvariant();
}


bool AreaProposal::operator==(const AreaProposal& rhs) const
{
    // Areas match if either overlaps the other by over 50%. If one area overlaps by this much, then
    // it cannot possibly overlap another area by any greater amount.
    const float OVERLAP_MATCHING_RATIO = 0.5f;

    for(auto& gateway : gateways)
    {
        if(hasInvalidGatewayAssociation(gateway.gateway, rhs.gateways, rhs.extent))
        {
#ifdef DEBUG_EQUALITY
            std::cout<<"DEBUG:AreaProposal: Invalid association between gateways for "<<getGlobalBoundary()<<" and "<<rhs.getGlobalBoundary()<<'\n';
#endif
            return false;
        }
    }

    auto modelBoundary = rhs.getGlobalBoundary();
    auto areaBoundary  = getGlobalBoundary();

    assert(modelBoundary.area() > 0.0 && areaBoundary.area() > 0.0);

    return modelBoundary.overlap(areaBoundary) > OVERLAP_MATCHING_RATIO;
}


bool AreaProposal::contains(const Point<double>& point) const
{
    return extent.contains(point, math::ReferenceFrame::GLOBAL);
}


AreaType AreaProposal::getGatewayType(const Gateway& g) const
{
    auto gIt = std::find(gateways.begin(), gateways.end(), g);
    return (gIt != gateways.end()) ? gIt->type : AreaType::area;
}


math::Rectangle<float> AreaProposal::getGlobalBoundary(void) const
{
    return extent.rectangleBoundary(math::ReferenceFrame::GLOBAL);
}


std::vector<Gateway> AreaProposal::getAllGateways(const VoronoiSkeletonGrid& grid, math::ReferenceFrame frame) const
{
    std::vector<Gateway> global(gateways.size());
    std::transform(gateways.begin(), gateways.end(), global.begin(), [](const auto& g) {
        return g.gateway;
    });

    if(frame != this->frame)
    {
        auto transform = math::convert_reference_frame(pose_t(0.0f, 0.0f, 0.0f),
                                                       this->frame,
                                                       frame,
                                                       extent.center());

        for(auto& gateway : global)
        {
            gateway = gateway.changeReferenceFrame(transform, grid);
        }
    }

    return global;
}


void AreaProposal::changeGatewayReferenceFrames(const pose_t& transform, const VoronoiSkeletonGrid& grid)
{
    for(auto& gateway : gateways)
    {
        gateway.gateway = gateway.gateway.changeReferenceFrame(transform, grid);
    }
}


void AreaProposal::changePointReferenceFrames(std::vector<Point<double>>& points, const pose_t& trans)
{
    for(auto& point : points)
    {
        point = transform(point, -trans.x, -trans.y, -trans.theta);
    }
}


void AreaProposal::addGateways(const std::vector<Gateway>& toAdd, AreaType type)
{
    for(auto& gateway : toAdd)
    {
        gateways.push_back({gateway, type});
    }
}


std::vector<Gateway> AreaProposal::getGatewaysOfType(AreaType type, math::ReferenceFrame frame) const
{
    std::vector<Gateway> requested;

    for(auto& gateway : gateways)
    {
        if((gateway.type == type) && (frame == this->frame))
        {
            requested.push_back(gateway.gateway);
        }
        else if(gateway.type == type)
        {
            requested.push_back(gateway.gateway);
        }
    }

    return requested;
}


bool AreaProposal::hasInvalidGatewayAssociation(const Gateway&                         gateway,
                                                const std::vector<proposal_gateway_t>& rhsGateways,
                                                const AreaExtent&                      rhsExtent) const
{
    // An invalid gateway association occurs when two proposals have an identical gateway, but the center of the areas are
    // on different sides of the matching gateway, indicating they belong to separate areas sharing the same boundary
    for(auto& rhs : rhsGateways)
    {
        if(gateway.isSimilarTo(rhs.gateway) &&
            (gateway.isPointToLeft(extent.center().toPoint()) != gateway.isPointToLeft(rhsExtent.center().toPoint())))
        {
            return true;
        }
    }

    return false;
}


void AreaProposal::enforceInvariant(void)
{
    if(gateways.size() + frontiers.size() + deadEnds.size() < 2)
    {
        std::ostringstream errorOut;
        errorOut<<"ERROR::AreaProposal: Invariant failed:\n"
                 <<"Extent:"<<extent.rectangleBoundary()<<'\n'
                 <<"Center:"<<extent.center()<<'\n'
                 <<"Gateways:\n";
        for(auto& gateway : gateways)
        {
            errorOut<<gateway.gateway.boundary()<<'\n';
        }
        errorOut<<"Frontiers:\n";
        for(auto frontier : frontiers)
        {
            errorOut<<frontier<<'\n';
        }
        errorOut<<"Dead Ends:\n";
        for(auto dead : deadEnds)
        {
            errorOut<<dead<<'\n';
        }

        throw InvalidAreaException(errorOut.str());
//         assert(gateways.size() + frontiers.size() + deadEnds.size() > 1);
    }
}


void print_proposal_info(const std::vector<Gateway>& gateways, const std::vector<Point<double>>& frontiers)
{
    std::cout<<"Gateways:\n";
    for(auto& gateway : gateways)
    {
        std::cout<<gateway.boundary()<<','<<gateway.center()<<'\n';
    }
    std::cout<<"Frontiers:\n";
    for(auto point : frontiers)
    {
        std::cout<<point<<'\n';
    }
}

} // namespace hssh
} // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_builder.cpp
* \author   Collin Johnson
*
* Definition of AreaBuilder.
*/

#include <hssh/local_topological/area_detection/labeling/area_creator.h>
#include <hssh/local_topological/affordances/transition.h>
#include <hssh/local_topological/affordances/exploration.h>
#include <hssh/local_topological/areas/decision_point.h>
#include <hssh/local_topological/areas/destination.h>
#include <hssh/local_topological/areas/path_segment.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <hssh/local_topological/area_detection/local_topo_isovist_field.h>
#include <hssh/local_topological/area_detection/gateways/gateway_utils.h>
#include <hssh/local_topological/area_detection/labeling/area_proposal.h>
#include <hssh/local_topological/area_detection/labeling/small_scale_star_builder.h>
#include <hssh/local_topological/area_detection/voronoi/voronoi_utils.h>
#include <utils/algorithm_ext.h>
#include <algorithm>
#include <array>
#include <iostream>
#include <cassert>

namespace vulcan
{
namespace hssh
{


Gateway create_outbound_gateway(const Gateway& g, Point<float> interiorPoint);
Gateway create_gateway_for_skeleton_cell(const Point<float>& skeletonCenter,
                                         const VoronoiSkeletonGrid& grid,
                                         const VoronoiIsovistField& isovists);


AreaBuilder::AreaBuilder(std::shared_ptr<SmallScaleStarBuilder> starBuilder)
: nextAreaId(0)
, placeBoundaryRadius(2.0f)
, starBuilder(starBuilder)
, hough_(1.0f, 1.0f)
{
}


std::unique_ptr<LocalArea> AreaBuilder::buildArea(const AreaProposal&        proposal,
                                                  const LocalPerceptualMap&  lpm,
                                                  const VoronoiSkeletonGrid& skeleton,
                                                  const VoronoiIsovistField& isovists)
{
    return buildArea(nextAreaId++, proposal, lpm, skeleton, isovists);
}


std::unique_ptr<LocalArea> AreaBuilder::buildArea(int                        id,
                                                  const AreaProposal&        proposal,
                                                  const LocalPerceptualMap&  lpm,
                                                  const VoronoiSkeletonGrid& skeleton,
                                                  const VoronoiIsovistField& isovists)
{
    proposal_info_t info = {id, proposal, lpm, skeleton, isovists};

    return buildAreaFromProposal(info);
}


std::unique_ptr<LocalArea> AreaBuilder::buildAreaFromProposal(const proposal_info_t& info)
{
    // A proposal should be one of: path segment, decision point, destination. Any other area type can't be converted
    // to a local area because they aren't concrete types
    switch(info.proposal.getType())
    {
    case AreaType::decision_point:
        return createDecisionPoint(info);

    case AreaType::destination:
        return createDestination(info);

    case AreaType::path_segment:
        return createPathSegment(info);

    case AreaType::area:
    case AreaType::place:
    default:
        std::cerr << "WARNING: AreaBuilder: Unable to handle proposal classification.\n";
        break;
    }

    return std::unique_ptr<LocalArea>{};
}


std::unique_ptr<LocalArea> AreaBuilder::createDecisionPoint(const proposal_info_t& info)
{
    // The all gateways for a decision point are used for creating the small-scale star
    // Every decision point has a metric map because it is a place
    auto gateways = info.proposal.getAllGateways(info.grid, math::ReferenceFrame::GLOBAL);

    // Small-scale star gateways already point outward
    auto star = starBuilder->buildStar(gateways,
                                       info.proposal.getCenter().toPoint(),
                                       info.proposal.getGlobalBoundary());

    // Assign the correct type to each of the fragments in the star
    // Reassign the internal gateways to match the direction of the small-scale star
    gateways.clear();
    std::vector<local_path_fragment_t> fragments(star.getAllFragments());
    for(auto& frag : fragments)
    {
        frag.type = info.proposal.getGatewayType(frag.gateway);

        if(frag.navigable)
        {
            gateways.push_back(frag.gateway);
        }
    }

    auto map = createPlaceMetricMap(info.proposal.getExtent(), info.lpm);
    hough_.compute(map, [](int x, int y, const LocalPerceptualMap& map) {
        return map.getCellType(x, y) & kOccupiedOccGridCell;
    });
    auto bestOrientation = hough_.bestLine();
    bestOrientation.theta = utils::wrap_hough_angle(bestOrientation.theta);

    auto extent = info.proposal.getExtent();
    extent.setOrientation(bestOrientation.theta);

    auto globalBoundary = extent.rectangleBoundary(math::ReferenceFrame::GLOBAL);

    std::cout << "orientation of " << extent.rectangleBoundary(math::ReferenceFrame::GLOBAL) << ": "
        << (bestOrientation.theta * 180.0 / M_PI) << " degrees. extent center:" << extent.center()
        << " extent boundary:" << extent.rectangleBoundary() << " map boundary:" << map.getBoundary()
        << " global adj:" << (globalBoundary.bottomLeft - extent.center().toPoint()) << "->"
        << (globalBoundary.topRight - extent.center().toPoint()) << '\n';

    return std::unique_ptr<LocalArea>(new LocalDecisionPoint(SmallScaleStar(fragments),
                                                             std::move(map),
                                                             info.id,
                                                             std::move(extent),
                                                             gateways));
}


std::unique_ptr<LocalArea> AreaBuilder::createDestination(const proposal_info_t& info)
{
    // The all gateways for a destination are used for creating the small-scale star
    // Every destination has a metric map because it is a place
    auto gateways = info.proposal.getAllGateways(info.grid, math::ReferenceFrame::GLOBAL);

    // Small-scale star gateways already point outward
    auto star = starBuilder->buildStar(gateways,
                                       info.proposal.getCenter().toPoint(),
                                       info.proposal.getGlobalBoundary());

    // Assign the correct type to each of the fragments in the star
    // Reassign the internal gateways to match the direction of the small-scale star
    gateways.clear();
    std::vector<local_path_fragment_t> fragments(star.getAllFragments());
    for(auto& frag : fragments)
    {
        frag.type = info.proposal.getGatewayType(frag.gateway);

        if(frag.navigable)
        {
            gateways.push_back(frag.gateway);
        }
    }

    auto map = createPlaceMetricMap(info.proposal.getExtent(), info.lpm);
    hough_.compute(map, [](int x, int y, const LocalPerceptualMap& map) {
        return map.getCellType(x, y) & kOccupiedOccGridCell;
    });
    auto bestOrientation = hough_.bestLine();
    bestOrientation.theta = utils::wrap_hough_angle(bestOrientation.theta);

    auto extent = info.proposal.getExtent();
    extent.setOrientation(bestOrientation.theta);

    std::cout << "orientation of " << extent.rectangleBoundary(math::ReferenceFrame::GLOBAL) << ": "
        << (bestOrientation.theta * 180.0 / M_PI) << " degrees.\n";

    return std::unique_ptr<LocalArea>(new LocalDestination(SmallScaleStar(fragments),
                                                           std::move(map),
                                                           info.id,
                                                           std::move(extent),
                                                           gateways));
}


std::unique_ptr<LocalArea> AreaBuilder::createPathSegment(const proposal_info_t& info)
{
    // The endpoints of the path segment are the gateways towards which the robot drives. They provide the travel
    // affordance for the area
    auto endpoints = info.proposal.getPathEndpoints();

    assert(endpoints.size() == 2);

    // Sort the endpoints so they always have the same ordering across updates. Use the endpoint closest to the origin
    // the plus end
    if(distance_between_points(Point<double>(0.0, 0.0), endpoints[1].point) <
        distance_between_points(Point<double>(0.0, 0.0), endpoints[0].point))
    {
        std::swap(endpoints[0], endpoints[1]);
    }

    auto gateways = info.proposal.getAllGateways(info.grid, math::ReferenceFrame::GLOBAL);
    std::vector<Gateway> finalGateways;
    std::size_t numNonGwyEndpoints = 0;

    for(int n = 0; n < 2; ++n)
    {
        auto& endpoint = endpoints[n];
        const auto otherEndpoint = endpoints[(n + 1) % 2];

        if(!endpoint.isGatewayEndpoint)
        {
            endpoint.gateway = create_gateway_for_skeleton_cell(endpoint.point, info.grid, info.isovists);

            // If the gateways point the same direction, then flip them
            if(angle_diff_abs(endpoint.gateway.direction(),
                angle_to_point(endpoint.point, otherEndpoint.point)) < M_PI_2)
            {
                endpoint.gateway.reverseDirections();
            }
            // Make sure this fake gateway is included in the complete list of gateways for the area
            finalGateways.push_back(endpoint.gateway);
            ++numNonGwyEndpoints;
        }
        else
        {
            if(!utils::contains(gateways, endpoint.gateway))
            {
                std::cerr << "ERROR: AreaCreator: Cannot create path segment because gateway endpoint was not found."
                    << "\nDesired:" << endpoint.gateway.boundary() << "\nGateways:\n";
                for(auto& g : gateways)
                {
                    std::cout << g.boundary() << '\n';
                }
                assert(utils::contains(gateways, endpoint.gateway));
            }

            // If the gateways point the same direction, then flip them
            if(angle_diff_abs(endpoint.gateway.direction(),
                angle_to_point(endpoint.point, otherEndpoint.point)) < M_PI_2)
            {
                endpoint.gateway.reverseDirections();
            }

            finalGateways.push_back(endpoint.gateway);
        }
    }

    // Destinations go along the left and right side of the path segment
    auto destinations = info.proposal.getDestinationGateways(math::ReferenceFrame::GLOBAL);
    std::vector<TransitionAffordance> left;
    std::vector<TransitionAffordance> right;

    // The path runs from plus to minus, which creates the distinction for separating the left and right destinations
    Line<float> centerLine(endpoints[0].point, endpoints[1].point);

    for(auto& dest : destinations)
    {
        bool isEndpoint = utils::contains_if(endpoints, [&dest](const AreaProposal::path_endpoint_t& end) {
            return end.isGatewayEndpoint && (end.gateway == dest);
        });

        // Skip all endpoint destinations
        if(isEndpoint)
        {
            std::cout << "INFO: AreaBuilder: Path segment has destination endpoint at " << dest << '\n';
            continue;
        }

        auto gateway = create_outbound_gateway(dest, closest_point_on_line(dest.center(), centerLine));
        finalGateways.push_back(gateway);

        if(left_of_line(centerLine, dest.center()))
        {
            left.emplace_back(gateway, AreaType::destination);
        }
        else
        {
            right.emplace_back(gateway, AreaType::destination);
        }
    }

    // There can be multiple edges leading to a decision point, so need to also add those if they aren't added in yet
    auto decisions = info.proposal.getDecisionGateways(math::ReferenceFrame::GLOBAL);
    for(auto& gwy : decisions)
    {
        if(!utils::contains(finalGateways, gwy))
        {
            std::cout << "INFO: AreaBuilder: Added secondary path-to-decision gateway at " << gwy << '\n';
            finalGateways.push_back(gwy);
        }
    }

    // For fixed areas, you can get adjacent path segments because they aren't actually meaningful. In this case, just
    // let them exist because they don't hurt anything
    auto paths = info.proposal.getPathGateways(math::ReferenceFrame::GLOBAL);
    if(!paths.empty())
    {
        std::cout << "WARNING: AreaBuilder: Found " << paths.size() << " path gateways adjacent to a path.\n";
        for(auto& gwy : paths)
        {
            if(!utils::contains(finalGateways, gwy))
            {
                finalGateways.push_back(gwy);
            }
        }
    }

    if(finalGateways.size() < gateways.size() + numNonGwyEndpoints)
    {
        std::cout << "Path segment gateways for "
            << info.proposal.getExtent().rectangleBoundary(math::ReferenceFrame::GLOBAL) << ":\n";
        for(auto& gwy : finalGateways)
        {
            std::cout << gwy.id() << " -> " << gwy << '\n';
        }
        std::cout << '\n';

        std::cout << "Original gateways:\n";
        for(auto& gwy : gateways)
        {
            std::cout << gwy.id() << " -> " << gwy << '\n';
        }
        std::cout << '\n';

        std::cout << "Initial gateways: " << gateways.size() << " Final: " << finalGateways.size()
            << " Non-end: " << numNonGwyEndpoints << '\n'
            << "Num dest gwys: " << destinations.size()
            << " Num decision gwys: " << info.proposal.getDecisionGateways(math::ReferenceFrame::GLOBAL).size()
            << "\n\n";

        assert(finalGateways.size() == gateways.size() + numNonGwyEndpoints);
    }

    // Can't actually determine lambda until the place at the other end has been calculated...just
    // use the length for now.
    Lambda lambda(centerLine.b.x - centerLine.a.x, centerLine.b.y - centerLine.a.y);

    // TODO: The ends are always treated as gateways right now. The frontier at the end of a path should probably be
    //       used in lieu of a fake gateway.

    return std::unique_ptr<LocalArea>(new LocalPathSegment(
        // If an endpoint doesn't lead to an area, then it is either a frontier or a dead end, both of which are
        // allowed. If it does lead to an area, then get the type of area that it leads to.
        TransitionAffordance(endpoints[0].gateway,
                             (endpoints[0].type == AreaType::area) ?
                                info.proposal.getGatewayType(endpoints[0].gateway) : endpoints[0].type),
        TransitionAffordance(endpoints[1].gateway,
                            (endpoints[1].type == AreaType::area) ?
                            info.proposal.getGatewayType(endpoints[1].gateway) : endpoints[1].type),
        left,
        right,
        lambda,
        info.id,
        info.proposal.getExtent(),
        finalGateways
    ));
}


LocalPerceptualMap AreaBuilder::createPlaceMetricMap(const AreaExtent& extent, const LocalPerceptualMap& map)
{
    // Each place exists within the current LPM (or at least part of it does)
    // Intersect the extent of the place with the LPM
    math::Rectangle<int> placeBoundaryInMap = calculateMapBoundary(extent, map);

    // Create a new LPM to hold the metric map for the place
    LocalPerceptualMap placeMap(placeBoundaryInMap.width(),
                                placeBoundaryInMap.height(),
                                map.metersPerCell(),
                                Point<float>(0.0f, 0.0f),
                                200,
                                0);

    // Copy the portion of the LPM into the metric map for the place
    Point<int> cell(0, 0);
    for(int yEnd = placeMap.getHeightInCells(); cell.y < yEnd; ++cell.y)
    {
        cell.x = 0;
        for(int xEnd = placeMap.getWidthInCells(); cell.x < xEnd; ++cell.x)
        {
            placeMap.setCostNoCheck(cell, map.getCost(placeBoundaryInMap.bottomLeft + cell));
            placeMap.setTypeNoCheck(cell, map.getCellType(placeBoundaryInMap.bottomLeft + cell));
        }
    }

    // The center is relative to the center of the area, which is defined by the extent, so adjust it accordingly.
    auto globalMapOrigin = utils::grid_point_to_global_point(placeBoundaryInMap.bottomLeft, map);
    globalMapOrigin -= extent.center().toPoint();
    placeMap.setBottomLeft(globalMapOrigin);

    return placeMap;
}


math::Rectangle<int> AreaBuilder::calculateMapBoundary(const AreaExtent& extent, const LocalPerceptualMap& map)
{
    auto globalBoundary = extent.rectangleBoundary(math::ReferenceFrame::GLOBAL);

    math::Rectangle<int> boundary{utils::global_point_to_grid_cell(globalBoundary.bottomLeft, map),
                                  utils::global_point_to_grid_cell(globalBoundary.topRight,   map)};

    int boundaryExpansion = placeBoundaryRadius * map.cellsPerMeter();

    boundary.bottomLeft.x = std::max(boundary.bottomLeft.x - boundaryExpansion, 0);
    boundary.bottomLeft.y = std::max(boundary.bottomLeft.y - boundaryExpansion, 0);
    boundary.topRight.x   = std::min(boundary.topRight.x + boundaryExpansion, static_cast<int>(map.getWidthInCells()));
    boundary.topRight.y   = std::min(boundary.topRight.y + boundaryExpansion, static_cast<int>(map.getHeightInCells()));

    assert(boundary.bottomLeft.x < boundary.topRight.x);
    assert(boundary.bottomLeft.y < boundary.topRight.y);

    math::Rectangle<float> newBoundary{utils::grid_point_to_global_point(boundary.bottomLeft, map),
                                       utils::grid_point_to_global_point(boundary.topRight,   map)};

    // FIX: Need to recreate the rectangle in order for bottomRight and topLeft to get set correctly.
    return math::Rectangle<int>(boundary.bottomLeft, boundary.topRight);
}


Gateway create_outbound_gateway(const Gateway& g, Point<float> interiorPoint)
{
    Gateway outGateway = g;

    // An outbound gateway minimizes the difference between the gateway direction and the angle from the interior point
    // to the gateway. This ensures the direction of the gateway points away from the area because a direction pointing
    // into the area will be closer to pi.

    auto interiorAngle = angle_to_point(interiorPoint, g.center());
    auto leftAngleDiff = angle_diff_abs(interiorAngle, g.leftDirection());
    auto rightAngleDiff = angle_diff_abs(interiorAngle, g.rightDirection());

    if(rightAngleDiff < leftAngleDiff)
    {
        outGateway.reverseDirections();
    }

    return outGateway;
}


Gateway create_gateway_for_skeleton_cell(const Point<float>& skeletonCenter,
                                         const VoronoiSkeletonGrid& grid,
                                         const VoronoiIsovistField& isovists)
{
    // The center of a gateway is on the skeleton
    cell_t pointCell = utils::global_point_to_grid_cell_round(skeletonCenter, grid);

    // The gateway is orthogonal to the direction of the gateway at this point
    float direction = isovists.contains(pointCell) ? isovists[pointCell].scalar(utils::Isovist::kMinLineNormal)
        : gateway_normal_from_source_cells(pointCell, grid);
    float minExtraDist = isovists.contains(pointCell) ? isovists[pointCell].scalar(utils::Isovist::kMinLineDist)
        : 1.0f;

    auto gateway = create_gateway_at_cell(pointCell, direction, -1, grid, minExtraDist);
    if(gateway)
    {
        return *gateway;
    }

    direction = angle_sum(voronoi_direction(pointCell, grid).left, M_PI_2);

    gateway = create_gateway_at_cell(pointCell, direction, -1, grid, minExtraDist);
    if(gateway)
    {
        return *gateway;
    }

    gateway = create_gateway_between_sources(grid.getSourceCells(pointCell.x, pointCell.y),
                                             pointCell,
                                             -1,
                                             grid,
                                             isovists);

    if(gateway)
    {
        return *gateway;
    }

    std::cerr << "ERROR: Failed to create gateway at " << skeletonCenter << '\n';
    return Gateway();
}

} // namespace hssh
} // namespace vulcan

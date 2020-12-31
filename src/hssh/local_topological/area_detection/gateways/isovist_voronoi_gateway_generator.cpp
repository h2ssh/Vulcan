/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     isovist_voronoi_gateway_generator.cpp
* \author   Collin Johnson
*
* Definition of IsovistVoronoiGatewayGenerator.
*/

#include "hssh/local_topological/area_detection/gateways/isovist_voronoi_gateway_generator.h"
#include "hssh/local_topological/area_detection/gateways/endpoint_validator.h"
#include "hssh/local_topological/area_detection/gateways/gateway_utils.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_utils.h"
#include "hssh/local_topological/gateway.h"
#include "utils/algorithm_ext.h"
#include "utils/stub.h"
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>
#include <boost/range/iterator_range.hpp>

// #define DEBUG_GATEWAYS

namespace vulcan
{
namespace hssh
{

std::pair<cell_t, cell_t> select_straightest_gateway_boundary(cell_t                     source,
                                                              cell_t                     skeleton,
                                                              const std::vector<cell_t>& skeletonCells,
                                                              const EndpointValidator&   validator,
                                                              const VoronoiSkeletonGrid& grid);
std::pair<cell_t, cell_t> select_straightest_gateway_from_sources(cell_t                     skeleton,
                                                                  const std::vector<cell_t>& sourceCells,
                                                                  const VoronoiSkeletonGrid& grid);
std::pair<cell_t, double> source_for_straightest_gateway(cell_t start,
                                                         cell_t skeleton,
                                                         CellConstIter beginSources,
                                                         CellConstIter endSources,
                                                         double originalAngle,
                                                         const EndpointValidator* validator);
double gateway_score(cell_t start, cell_t end, cell_t skeleton, double originalAngle);


IsovistVoronoiGatewayGenerator::IsovistVoronoiGatewayGenerator(const isovist_voronoi_gateway_generator_params_t& params)
: nextGatewayId_(0)
, params_(params)
{
}


std::vector<WeightedGateway> IsovistVoronoiGatewayGenerator::generateGateways(const std::vector<WeightedGateway>& priorGateways,
                                                                              const VoronoiIsovistField& isovists,
                                                                              const VoronoiSkeletonGrid& grid,
                                                                              const EndpointValidator& validator)
{
    VoronoiEdges edges(grid, SKELETON_CELL_REDUCED_SKELETON);
    VoronoiIsovistGradients gradients(edges);
    gradients.calculateGradients(utils::Isovist::kShapeEccentricity, isovists);

    VoronoiIsovistMaxima maxima(gradients, edges, grid, params_.numAboveMean, params_.saveGradientData);

    initialGateways_.clear();
    isovists_      = &isovists;
    grid_          = &grid;
    validator_     = &validator;

    sourceToCells_ = extract_source_cells(grid, SKELETON_CELL_REDUCED_SKELETON);

    for(auto& maximum : maxima)
    {
        createGatewayForMaximumIfValid(maximum);
    }

    std::cout << "INFO: IsovistVoronoiGatewayGenerator: Maxima:" << maxima.size()
        << " Num generated:" << initialGateways_.size() << '\n';

    return initialGateways_;
}


bool IsovistVoronoiGatewayGenerator::createGatewayForMaximumIfValid(const isovist_local_maximum_t& maximum)
{
    proposeGatewaysForMaximum(maximum);

    if(proposals_.empty())
    {
        return false;
    }

    // There was at least one proposal, so select the smallest proposed gateway and call it good
    auto bestProposalIt = std::max_element(proposals_.begin(), proposals_.end());
    initialGateways_.emplace_back(WeightedGateway{toGateway(*bestProposalIt), bestProposalIt->gradient, false});

#ifdef DEBUG_GATEWAYS
    std::cout << "DEBUG: GatewayGenerator: Best gateway for: " << maximum.maximum.position << "->"
        << initialGateways_.back().gateway.boundary() << '\n';
#endif

    return true;
}


void IsovistVoronoiGatewayGenerator::proposeGatewaysForMaximum(const isovist_local_maximum_t& maximum)
{
    proposals_.clear();
    sourceCells_.clear();
    edgeCells_.clear();

    // Find all valid source cells for gateways and all valid skeleton edge cells

    for(auto& cell : maximum.skeletonCells)
    {
        // Find valid source cells associated with this skeleton cell
        std::copy_if(grid_->beginSourceCells(cell.position),
                     grid_->endSourceCells(cell.position),
                     std::back_inserter(sourceCells_),
                     [this](cell_t source) {
            return validator_->isValidEndpoint(source);
        });

        // Store the cell itself for further gateway creation
        edgeCells_.push_back(cell.position);
    }
    std::sort(sourceCells_.begin(), sourceCells_.end());
    utils::erase_unique(sourceCells_);

    // For each skeleton cell, find the best possible gateway
    for(auto& cell : maximum.skeletonCells)
    {
        // Ignore any cells that fall on a junction because
        if(num_neighbor_cells_with_classification(cell.position, SKELETON_CELL_REDUCED_SKELETON, *grid_, FOUR_THEN_EIGHT_WAY) > 2)
        {
            continue;
        }

        auto gateway = proposeGatewayForSkeletonCell(cell.position);

        // A gateway isn't guaranteed to be found
        if(gateway)
        {
            gateway->gradient = cell.value;
            proposals_.push_back(*gateway);
        }
    }

#ifdef DEBUG_GATEWAYS
    if(proposals_.empty())
    {
        std::cout<<"Found no gateways for maximum centered at "<<maximum.maximum.position<<'\n';
    }
#endif // DEBUG_GATEWAYS
}


boost::optional<IsovistVoronoiGatewayGenerator::proposed_gateway_t>
IsovistVoronoiGatewayGenerator::proposeGatewayForSkeletonCell(cell_t skeletonCell)
{
    if((sourceCells_.size() < 2) || isInvalidSkeletonCell(skeletonCell))
    {
        return boost::none;
    }

    if(params_.useSkeletonBasedGateways)
    {
        auto bestGateway = select_straightest_gateway_from_sources(skeletonCell, sourceCells_, *grid_);

        if(isValidProposal(skeletonCell, bestGateway.first, bestGateway.second))
        {
            return proposed_gateway_t(bestGateway, skeletonCell);
        }
    }
    else if(params_.useSourceBasedGateways)
    {
        // Go through each gateway and find the one with the highest score
        auto bestGateway = createProposalForSource(*(grid_->beginSourceCells(skeletonCell)), skeletonCell);
        for(auto source : boost::make_iterator_range(grid_->beginSourceCells(skeletonCell) + 1, grid_->endSourceCells(skeletonCell)))
        {
            auto sourceGateway = createProposalForSource(source, skeletonCell);

            if(sourceGateway)
            {
                if(!bestGateway || (sourceGateway->score() > bestGateway->score()))
                {
                    bestGateway = sourceGateway;
                }
            }
        }

        return bestGateway;
    }
    else
    {
        std::cerr << "ERROR: IsovistVoronoiGatewayGenerator: Invalid parameters. Must have one of gateway selection flags set to true.\n";
        assert(false);
    }

    return boost::none;
}


boost::optional<IsovistVoronoiGatewayGenerator::proposed_gateway_t>
IsovistVoronoiGatewayGenerator::createProposalForSource(cell_t source, cell_t skeleton)
{
    auto bestGateway = select_straightest_gateway_boundary(source, skeleton, edgeCells_, *validator_, *grid_);

    if(isValidProposal(bestGateway.first, source, bestGateway.second))
    {
        return proposed_gateway_t(std::make_pair(source, bestGateway.second), bestGateway.first);
    }

    return boost::none;
}


bool IsovistVoronoiGatewayGenerator::isInvalidSkeletonCell(cell_t skeleton)
{
    // A skeleton cell is invalid if:
    //  - it is at the junction of the underlying skeleton
    //  - no sources on difference sides of the skeleton (angle between them and skeleton < M_PI_2), or
    //  - no sources both valid endpoints

    if(num_neighbor_cells_with_classification(skeleton, SKELETON_CELL_SKELETON, *grid_, FOUR_THEN_EIGHT_WAY) > 2)
    {
        return true;
    }

    for(auto sourceIt = grid_->beginSourceCells(skeleton), endIt = grid_->endSourceCells(skeleton);
        sourceIt != endIt;
        ++sourceIt)
    {
        // Skip any sources that aren't valid. No point in searching the next ones
        if(!validator_->isValidEndpoint(*sourceIt))
        {
            continue;
        }

        for(auto nextIt = sourceIt + 1; nextIt != endIt; ++nextIt)
        {
            if(!validator_->isValidEndpoint(*nextIt))
            {
                continue;
            }

            // Both sources were valid, but are they on separate sides of the skeleton?
            if(angle_between_points(*sourceIt, *nextIt, skeleton) > M_PI_2)
            {
                return false;
            }
        }
    }

    // No valid pair of sources was found
    return true;
}


bool IsovistVoronoiGatewayGenerator::isValidProposal(cell_t skeleton, cell_t sourceA, cell_t sourceB)
{
    return (std::abs(angle_between_points(sourceA, sourceB, skeleton)) > params_.minGatewayStraightness)
        && validator_->isValidEndpoint(sourceA)
        && validator_->isValidEndpoint(sourceB);
}


Gateway IsovistVoronoiGatewayGenerator::toGateway(const proposed_gateway_t& proposed)
{
    return Gateway(grid_->getTimestamp(),
                   nextGatewayId_++,
                   Line<int>(proposed.sources.first, proposed.sources.second),
                   proposed.skeleton,
                   *isovists_,
                   *grid_);
}


std::pair<cell_t, cell_t> select_straightest_gateway_boundary(cell_t                     source,
                                                              cell_t                     skeleton,
                                                              const std::vector<cell_t>& skeletonCells,
                                                              const EndpointValidator&   validator,
                                                              const VoronoiSkeletonGrid& grid)
{
    double originalAngle = angle_to_point(source, skeleton);

    auto   straightestBoundary = std::make_pair(source, 0.0);
    cell_t straightestSkeleton;

    for(auto cell : skeletonCells)
    {
        // Skip all junctions. They aren't valid gateway boundaries
        if(num_neighbor_cells_with_classification(cell, SKELETON_CELL_REDUCED_SKELETON, grid, FOUR_THEN_EIGHT_WAY) > 2)
        {
            continue;
        }

        auto sourceBoundary = source_for_straightest_gateway(source,
                                                             cell,
                                                             grid.beginSourceCells(cell),
                                                             grid.endSourceCells(cell),
                                                             originalAngle,
                                                             &validator);

        //         (sourceBoundary.second > straightestBoundary.second) ||
        //         (absolute_fuzzy_equal(sourceBoundary.second, straightestBoundary.second)
        //         && distance_between_points(source, sourceBoundary.first) < distance_between_points(source, straightestBoundary.first)))

        if(sourceBoundary.second > straightestBoundary.second)
        {
            straightestBoundary = sourceBoundary;
            straightestSkeleton = cell;
        }
    }

    return std::make_pair(straightestSkeleton, straightestBoundary.first);
}


std::pair<cell_t, cell_t> select_straightest_gateway_from_sources(cell_t                     skeleton,
                                                                  const std::vector<cell_t>& sourceCells,
                                                                  const VoronoiSkeletonGrid& grid)
{
    assert(sourceCells.size() > 1);

    std::pair<cell_t, cell_t> straightestBoundary;
    double maxScore = 0.0;

    for(std::size_t n = 0; n < sourceCells.size(); ++n)
    {
        auto sourceBoundary = source_for_straightest_gateway(sourceCells[n],
                                                             skeleton,
                                                             sourceCells.begin() + n + 1,
                                                             sourceCells.end(),
                                                             angle_to_point(sourceCells[n], skeleton),
                                                             nullptr);

        if(sourceBoundary.second > maxScore)
        {
            straightestBoundary = std::make_pair(sourceCells[n], sourceBoundary.first);
            maxScore = sourceBoundary.second;
        }
    }

    return straightestBoundary;
}


std::pair<cell_t, double> source_for_straightest_gateway(cell_t start,
                                                         cell_t skeleton,
                                                         CellConstIter beginSources,
                                                         CellConstIter endSources,
                                                         double originalAngle,
                                                         const EndpointValidator* validator)
{
    double maxScore = 0.0;
    cell_t maxCell;

    for(auto source : boost::make_iterator_range(beginSources, endSources))
    {
        if(validator && !validator->isValidEndpoint(source))
        {
            continue;
        }

        double sourceScore = gateway_score(start, source, skeleton, originalAngle);

        if(sourceScore > maxScore)
        {
            maxScore = sourceScore;
            maxCell  = source;
        }
    }

    return std::make_pair(maxCell, maxScore);
}


double gateway_score(cell_t start, cell_t end, cell_t skeleton, double originalAngle)
{
    double skeletonDirection = angle_to_point(start, skeleton);
    double length = distance_between_points(start, skeleton) + distance_between_points(end, skeleton);
    double angle = angle_between_points(start, end, skeleton);
    return (length > 0.0) ?  (1.0 - angle_diff_abs(originalAngle, skeletonDirection) / M_PI) * angle / length : 0.0;
}

} // namespace hssh
} // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     isovist_orientation_gateway_generator.cpp
 * \author   Collin Johnson
 *
 * Definition of IsovistOrientationGatewayGenerator.
 */

#include "hssh/local_topological/area_detection/gateways/isovist_orientation_gateway_generator.h"
#include "hssh/local_topological/area_detection/gateways/endpoint_validator.h"
#include "hssh/local_topological/area_detection/gateways/gateway_utils.h"
#include "hssh/local_topological/area_detection/gateways/isovist_gradients.h"
#include "hssh/local_topological/area_detection/gateways/isovist_maxima.h"
#include "hssh/local_topological/area_detection/local_topo_isovist_field.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_edges.h"
#include "hssh/types.h"
#include "utils/algorithm_ext.h"
#include "utils/ray_tracing.h"
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>
#include <boost/range/iterator_range.hpp>

namespace vulcan
{
namespace hssh
{

double isovist_orientation(const utils::Isovist& isovist);
bool is_shorter_gateway(const Gateway& lhs, const Gateway& rhs, cell_t maximumCell);
bool is_shorter_gateway(const Line<int>& lhs, const Line<int>& rhs, cell_t lhsCell, cell_t rhsCell, cell_t maximumCell);
bool is_better_gateway(const Gateway& lhs,
                       const Gateway& rhs,
                       cell_t lhsCell,
                       cell_t rhsCell,
                       position_value_t maximumValue);


IsovistOrientationGatewayGenerator::IsovistOrientationGatewayGenerator(
  const isovist_orientation_gateway_generator_params_t& params)
: nextGatewayId_(0)
, params_(params)
{
}


std::vector<WeightedGateway>
  IsovistOrientationGatewayGenerator::generateGateways(const std::vector<WeightedGateway>& priorGateways,
                                                       const VoronoiIsovistField& isovists,
                                                       const VoronoiSkeletonGrid& grid,
                                                       const EndpointValidator& validator)
{
    VoronoiEdges edges(grid, SKELETON_CELL_REDUCED_SKELETON);
    VoronoiIsovistGradients gradients(edges);
    gradients.calculateGradients(utils::Isovist::kShapeEccentricity, isovists);

    VoronoiIsovistMaxima maxima(gradients, edges, grid, params_.numAboveMean, params_.saveGradientData);

    gradients_ = &gradients;
    isovists_ = &isovists;
    grid_ = &grid;
    validator_ = &validator;
    sourceToSkeleton_ = extract_source_cells(grid, SKELETON_CELL_REDUCED_SKELETON);

    // Create new gateways for every new maximum
    for (auto& maximum : maxima) {
        createGatewaysForMaximum(maximum);
    }

    return gatewayHypotheses_;
}


void IsovistOrientationGatewayGenerator::createGatewaysForMaximum(const isovist_local_maximum_t& maximum)
{
    extractEdgeCellsForMaximum(maximum);

    const double kAngleSearchRange = M_PI / 4.0;   // search +/- 45 degrees from where the minimum is

    std::vector<WeightedGateway> possibleGateways;
    for (auto& c : maximumCells_) {
        // Skip cells that don't have a gradient because their weight will be invalid
        if ((c.value == 0.0) || !isovists_->contains(c.position)) {
            continue;
        }

        double isoOrientation = isovist_orientation(isovists_->at(c.position));

        auto bestGateway = gateway_boundary_line_at_cell(c.position, isoOrientation, *grid_);
        double bestNormal = isoOrientation;
        position_value_t bestCell = c;

        for (double orientation = -kAngleSearchRange; orientation < kAngleSearchRange; orientation += M_PI / 180.0f) {
            auto cellGateway = gateway_boundary_line_at_cell(c.position, orientation + isoOrientation, *grid_);

            if (cellGateway   // must have found a gateway for the cell
                              //                 && validator_->isValidGateway(*cellGateway) // and it must be valid
                && (!bestGateway
                    || is_shorter_gateway(*cellGateway,
                                          *bestGateway,
                                          c.position,
                                          bestCell.position,
                                          maximum.maximum.position)))   // and if shorter, it's the new best
            {
                bestGateway = cellGateway;
                bestNormal = orientation + isoOrientation;
                bestCell = c;
            }
        }

        if (bestGateway) {
            auto g = create_gateway_at_cell(bestCell.position, bestNormal, nextGatewayId_, *grid_);
            if (g) {
                possibleGateways.push_back({*g, std::abs(bestCell.value), false});
            }
        }
    }

    double meanDist = 0.0;
    double sumDistWeights = 0.0;
    for (auto& c : maximumCells_) {
        meanDist += c.value * grid_->getMetricDistance(c.position.x, c.position.y);
        sumDistWeights += c.value;
    }
    meanDist /= sumDistWeights;

    std::sort(possibleGateways.begin(),
              possibleGateways.end(),
              [meanDist](const WeightedGateway& lhs, const WeightedGateway& rhs) -> bool {
                  // If the differences are approximately the same, then take the gateway with the higher weight.
                  // Otherwise opt for the gateway with the closest distance to the mean.
                  double lhsDiff = std::abs(lhs.gateway.length() - meanDist) / lhs.weight;
                  double rhsDiff = std::abs(rhs.gateway.length() - meanDist) / rhs.weight;
                  return (absolute_fuzzy_equal(lhsDiff, rhsDiff) && lhs.weight > rhs.weight) || (lhsDiff < rhsDiff);
              });

    auto bestGatewayIt = possibleGateways.begin();

    //     double isoOrientation = isovist_orientation(isovists_->at(maximum.maximum.position));
    //     for(double orientation = -kAngleSearchRange; orientation < kAngleSearchRange; orientation += 0.01)
    //     {
    //         auto cellGateway = create_gateway_at_cell(maximum.maximum.position, orientation + isoOrientation,
    //         nextGatewayId_, *grid_); if(cellGateway
    //             && validator_->isValidGateway(*cellGateway)
    //             && (!haveBest || is_shorter_gateway(*cellGateway, *bestGateway, maximum.maximum.position)))
    //         {
    //             bestGateway = cellGateway;
    //             haveBest = true;
    //         }
    //     }
    //
    //     for(auto& c : maximumCells_)
    //     {
    //         double isoOrientation = isovist_orientation(isovists_->at(c));
    //
    //         for(double orientation = -kAngleSearchRange; orientation < kAngleSearchRange; orientation += 0.01)
    //         {
    //             auto cellGateway = create_gateway_at_cell(c, orientation + isoOrientation, nextGatewayId_, *grid_);
    //
    //             if(cellGateway
    //                 && validator_->isValidGateway(*cellGateway)
    //                 && (!haveBest || is_better_gateway(*cellGateway, *bestGateway, c, bestGatewayCell,
    //                 maximum.maximum)))
    //             {
    //                 bestGateway = cellGateway;
    //                 bestGatewayCell = c;
    //                 haveBest = true;
    //             }
    //         }
    //     }

    ++nextGatewayId_;

    // Now that the gateway has been found, do the expensive normal calculation
    if (bestGatewayIt != possibleGateways.end()) {
        gatewayHypotheses_.push_back({bestGatewayIt->gateway, std::abs(maximum.totalChange), false});
    }
}


void IsovistOrientationGatewayGenerator::extractEdgeCellsForMaximum(const isovist_local_maximum_t& maximum)
{
    // Remove any cells associated with the sources that aren't part of the current edge
    edgeCells_.clear();
    extract_edge_from_skeleton(maximum.maximum.position, *grid_, edgeCells_);

    if (edgeCells_.size() < 3) {
        return;
    }
    // Ignore the junctions at either end of the edge
    edgeCells_.erase(edgeCells_.begin());
    edgeCells_.pop_back();

    maximumCells_.clear();
    // Sort the maximum cells in order along the edge
    for (auto c : edgeCells_) {
        // See if this edge cell is part of the maximum
        auto maxIt =
          std::find_if(maximum.skeletonCells.begin(), maximum.skeletonCells.end(), [c](const position_value_t& pv) {
              return pv.position == c;
          });

        // If so, then add it to the maximum cells
        if (maxIt != maximum.skeletonCells.end()) {
            maximumCells_.push_back(*maxIt);
        }
    }

    // Grow out the edge cells in each direction until the gradient begins increasing again. This accounts for a
    // maxima that doesn't quite reach the associated source/skeleton that produce a valid gateway as a consequence
    // of clutter or other noise sources
    // Start growing the front
    bool madeChange = !maximumCells_.empty();
    while (madeChange) {
        madeChange = false;
        cell_t growCell = maximumCells_.front().position;
        auto growValue = gradients_->gradientAt(growCell).value;

        for (auto source :
             boost::make_iterator_range(grid_->beginSourceCells(growCell), grid_->endSourceCells(growCell))) {
            for (auto& skel : sourceToSkeleton_[source]) {
                bool onSameEdge = utils::contains(edgeCells_, skel);
                bool isAdded = utils::contains_if(maximumCells_, [skel](const position_value_t& pv) {
                    return pv.position == skel;
                });
                auto gradient = gradients_->gradientAt(skel);

                if (onSameEdge                            // only grow along the same edge
                    && !isAdded                           // don't add duplicates
                    && (gradient.value * growValue > 0)   // don't cross any zero crossings of the gradient
                    && (std::abs(gradient.value) <= std::abs(growValue)))   // only allow lower gradients
                {
                    maximumCells_.push_front(gradient);
                    madeChange = true;
                }
            }
        }
    }

    // Then grow at the back
    madeChange = !maximumCells_.empty();
    while (madeChange) {
        madeChange = false;
        cell_t growCell = maximumCells_.back().position;
        auto growValue = gradients_->gradientAt(growCell).value;

        for (auto source :
             boost::make_iterator_range(grid_->beginSourceCells(growCell), grid_->endSourceCells(growCell))) {
            for (auto& skel : sourceToSkeleton_[source]) {
                bool onSameEdge = utils::contains(edgeCells_, skel);
                bool isAdded = utils::contains_if(maximumCells_, [skel](const position_value_t& pv) {
                    return pv.position == skel;
                });
                auto gradient = gradients_->gradientAt(skel);

                if (onSameEdge                            // only grow along the same edge
                    && !isAdded                           // don't add duplicates
                    && (gradient.value * growValue > 0)   // don't cross any zero crossings of the gradient
                    && (std::abs(gradient.value) <= std::abs(growValue)))   // only allow lower gradients
                {
                    maximumCells_.push_back(gradient);
                    madeChange = true;
                }
            }
        }
    }

    // Remove any duplicate cells -- sort in decreasing order of value for equal positions, as erase unique removes all
    // after the first stored value
    std::sort(maximumCells_.begin(), maximumCells_.end(), [](const position_value_t& lhs, const position_value_t& rhs) {
        return (lhs.position < rhs.position) || ((lhs.position == rhs.position) && (lhs.value > rhs.value));
    });
    utils::erase_unique(maximumCells_);
}


double isovist_orientation(const utils::Isovist& isovist)
{
    return isovist.scalar(utils::Isovist::kMinDistOrientation) + M_PI_2;
}


bool is_shorter_gateway(const Gateway& lhs, const Gateway& rhs, cell_t maximumCell)
{
    double lhsLength = length(lhs.cellBoundary());
    double rhsLength = length(rhs.cellBoundary());

    if (lhsLength == rhsLength) {
        return distance_between_points(lhs.skeletonCell(), maximumCell)
          < distance_between_points(rhs.skeletonCell(), maximumCell);
    }

    return lhsLength < rhsLength;
}


bool is_shorter_gateway(const Line<int>& lhs, const Line<int>& rhs, cell_t lhsCell, cell_t rhsCell, cell_t maximumCell)
{
    double lhsLength = length(lhs);
    double rhsLength = length(rhs);

    if (lhsLength == rhsLength) {
        return distance_between_points(lhsCell, maximumCell) < distance_between_points(rhsCell, maximumCell);
    }

    return lhsLength < rhsLength;
}


bool is_better_gateway(const Gateway& lhs,
                       const Gateway& rhs,
                       cell_t lhsCell,
                       cell_t rhsCell,
                       position_value_t maximumValue)
{
    double lhsScore = length(lhs.cellBoundary()) + distance_between_points(lhsCell, maximumValue.position);
    double rhsScore = length(rhs.cellBoundary()) + distance_between_points(rhsCell, maximumValue.position);
    return lhsScore < rhsScore;
}

}   // namespace hssh
}   // namespace vulcan

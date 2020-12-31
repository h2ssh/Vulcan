/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     max_likelihood_csp.cpp
 * \author   Collin Johnson
 *
 * Definition of MaxLikelihoodCSP.
 */

#include "hssh/local_topological/area_detection/labeling/max_likelihood_csp.h"
#include "hssh/local_topological/area_detection/labeling/boundary.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_graph.h"
#include "hssh/local_topological/area_detection/labeling/mcmc_sampling.h"
#include "hssh/local_topological/area_detection/labeling/small_scale_star_builder.h"
#include "system/debug_communicator.h"
#include "utils/algorithm_ext.h"
#include "utils/serialized_file_io.h"
#include <boost/range/iterator_range.hpp>
#include <cassert>

namespace vulcan
{
namespace hssh
{

using HypToIdMap = std::unordered_map<AreaHypothesis*, AlignmentGraph::Id>;

void create_fixed_type_constraint(AreaHypothesis* area,
                                  HypToIdMap& hypIds,
                                  HypothesisType fixedtype,
                                  AlignmentGraph& network);
void create_constraints_for_area(AreaHypothesis* area,
                                 HypToIdMap& hypIds,
                                 const SmallScaleStarBuilder& starBuilder,
                                 AlignmentGraph& network);
void create_star_based_constraints(AreaHypothesis* area,
                                   HypToIdMap& hypIds,
                                   const SmallScaleStarBuilder& starBuilder,
                                   AlignmentGraph& network);
void create_adjacency_constraints(AreaHypothesis* area, HypToIdMap& hypIds, AlignmentGraph& network);
void create_all_neighbors_constraint(AreaHypothesis* area, HypToIdMap& hypIds, AlignmentGraph& network);
void create_exit_area_constraint(AreaHypothesis* area,
                                 const std::vector<AreaHypothesis*>& mutableAreas,
                                 HypToIdMap& hypIds,
                                 AlignmentGraph& network);


MaxLikelihoodCSP::MaxLikelihoodCSP(const std::shared_ptr<SmallScaleStarBuilder>& starBuilder,
                                   const MCMCSamplingParams& mcmcParams)
: starBuilder_(starBuilder)
, mcmcParams_(mcmcParams)
{
}


CSPSolution MaxLikelihoodCSP::solve(const std::vector<AreaHypothesis*>& fixedAreas,
                                    const std::vector<AreaHypothesis*>& mutableAreas,
                                    AreaHypothesis* exitedArea,
                                    AreaHypothesis* enteredArea,
                                    const BoundaryClassifier& boundaryClassifier)
{
    CSPSolution solution = runMCMC(fixedAreas, mutableAreas, exitedArea, enteredArea, boundaryClassifier, true);

    std::cout << "Max-likelihood solution: " << solution << '\n';

    // If a fixed area is failing, then just make them mutable and try again
    if (solution.errorCode() == CSPSolution::fixed_area_failing_constraints) {
        // First fallback to just not do the initial merge, which can overly constrain the problem
        solution = runMCMC(fixedAreas, mutableAreas, exitedArea, enteredArea, boundaryClassifier, false);

        std::cout << "Fixed-area initial recovery solution: " << solution << '\n';
    }

    // If a fixed area is failing, then just make them mutable and try again
    if (solution.errorCode() == CSPSolution::fixed_area_failing_constraints) {
        std::vector<AreaHypothesis*> allAreas;
        allAreas.insert(allAreas.end(), fixedAreas.begin(), fixedAreas.end());
        allAreas.insert(allAreas.end(), mutableAreas.begin(), mutableAreas.end());

        std::vector<AreaHypothesis*> noAreas;

        solution = runMCMC(noAreas, allAreas, exitedArea, enteredArea, boundaryClassifier, true);

        std::cout << "Fixed-area recovery solution: " << solution << '\n';
    }

    return solution;
}


void MaxLikelihoodCSP::sendDebug(system::DebugCommunicator& communicator)
{
    //     communicator.sendDebug(debugInfo_);
    std::string path = getenv("VULCAN_BIN");
    path += "/csp_debug_info.log";
    utils::save_serializable_to_file(path, debugInfo_);
}


CSPSolution MaxLikelihoodCSP::runMCMC(const std::vector<AreaHypothesis*>& fixedAreas,
                                      const std::vector<AreaHypothesis*>& mutableAreas,
                                      AreaHypothesis* exitedArea,
                                      AreaHypothesis* enteredArea,
                                      const BoundaryClassifier& boundaryClassifier,
                                      bool doInitialMerge)
{
    // Create the network
    MCMCSampling network(mcmcParams_, &boundaryClassifier);

    // Add all areas to the network
    std::unordered_map<AreaHypothesis*, AlignmentGraph::Id> hypIds;
    for (auto hyp : fixedAreas) {
        if (hyp != exitedArea) {
            assert(hyp);
            assert(hyp->getType() != HypothesisType::kArea);
            hypIds[hyp] = network.addFixedArea(hyp, hyp->getType(), false);   // don't need to be connected to graph

            std::cout << "Adding fixed:" << hyp->rectangleBoundary() << ' ' << hyp->getType() << '\n';
        }
    }

    if (exitedArea) {
        hypIds[exitedArea] = network.addFixedArea(exitedArea, exitedArea->getType(), true);   // must be connected

        std::cout << "Adding exited:" << exitedArea->rectangleBoundary() << ' ' << exitedArea->getType() << '\n';
    }

    if (enteredArea) {
        hypIds[enteredArea] = network.addArea(enteredArea, enteredArea->getType());
        std::cout << "Adding entered:" << enteredArea->rectangleBoundary() << '\n';
    }

    for (auto hyp : mutableAreas) {
        if (hyp != enteredArea) {
            assert(hyp);
            auto areaType = hyp->getType();
            hypIds[hyp] = network.addArea(hyp, HypothesisType::kArea);
            assert(areaType == hyp->getType());

            std::cout << "Adding mutable:" << hyp->rectangleBoundary() << '\n';
        }
    }

    // Once areas are added to the network, create the constraints for each of the three types of areas
    // Fixed areas only have a fixed type constraint. This constraint is done to ensure that the type of prior
    // areas doesn't affect the current graph. These fixed areas might have been chopped in half or otherwise changed
    // as a result of the shrinking of the map on exiting an area
    for (auto hyp : fixedAreas) {
        create_fixed_type_constraint(hyp, hypIds, hyp->getType(), network);

        std::cout << "Adding fixed constraint:" << hyp->rectangleBoundary() << ' ' << hyp->getType() << '\n';
    }

    // The exited area can't have its type changed, but it also needs to have the full set of constraints because it
    // separates the fixed areas from the mutable areas and thus is the graph separator between them.
    if (exitedArea) {
        create_fixed_type_constraint(exitedArea, hypIds, exitedArea->getType(), network);
        create_constraints_for_area(exitedArea, hypIds, *starBuilder_, network);
        create_exit_area_constraint(exitedArea, mutableAreas, hypIds, network);

        std::cout << "Adding exit constraint: " << exitedArea->rectangleBoundary() << ' ' << exitedArea->getType()
                  << '\n';
    }

    if (enteredArea) {
        create_constraints_for_area(enteredArea, hypIds, *starBuilder_, network);
    }

    // Mutable areas need to have the full set of constraints
    for (auto hyp : mutableAreas) {
        create_constraints_for_area(hyp, hypIds, *starBuilder_, network);
    }

    debugInfo_.iterations.clear();
    debugInfo_.extents.clear();
    return network.solve(doInitialMerge, &debugInfo_);
}


void create_fixed_type_constraint(AreaHypothesis* area,
                                  HypToIdMap& hypIds,
                                  HypothesisType fixedType,
                                  AlignmentGraph& network)
{
    AlignmentConstraint::AdjVec adjacent(area->numBoundaries());
    std::transform(area->beginBoundary(),
                   area->endBoundary(),
                   adjacent.begin(),
                   [&area, &hypIds](const AreaHypothesisBoundary* boundary) {
                       return ConstraintAdjacentArea{hypIds[area->adjacentArea(boundary->getGateway())],
                                                     boundary->getGateway().id()};
                   });

    network.addConstraint(AlignmentConstraint::CreateFixedConstraint(adjacent, hypIds[area], fixedType));
}


void create_constraints_for_area(AreaHypothesis* area,
                                 HypToIdMap& hypIds,
                                 const SmallScaleStarBuilder& starBuilder,
                                 AlignmentGraph& network)
{
    create_star_based_constraints(area, hypIds, starBuilder, network);
    create_adjacency_constraints(area, hypIds, network);
    create_all_neighbors_constraint(area, hypIds, network);
}


void create_star_based_constraints(AreaHypothesis* area,
                                   HypToIdMap& hypIds,
                                   const SmallScaleStarBuilder& starBuilder,
                                   AlignmentGraph& network)
{
    // For each area, find all unaligned gateway pairs
    // The non-through-paths are unaligned constraints.
    AlignmentGraph::Id insideId = hypIds[area];
    AlignmentConstraint::AdjVec adjacent;

    auto unalignedGateways = area->pairwiseUnalignedGateways();

    for (auto& u : unalignedGateways) {
        adjacent.clear();

        AlignmentGraph::Id firstId = hypIds[area->adjacentArea(*u.first)];
        AlignmentGraph::Id secondId = hypIds[area->adjacentArea(*u.second)];

        adjacent.push_back({firstId, u.first->id()});
        adjacent.push_back({secondId, u.second->id()});
        network.addConstraint(AlignmentConstraint::CreateUnalignedConstraint(adjacent, insideId));
    }

    // For each area, also find the aligned gateway pairs
    auto alignedGateways = area->pairwiseAlignedGateways();

    for (auto& u : alignedGateways) {
        adjacent.clear();

        AlignmentGraph::Id firstId = hypIds[area->adjacentArea(*u.first)];
        AlignmentGraph::Id secondId = hypIds[area->adjacentArea(*u.second)];

        adjacent.push_back({firstId, u.first->id()});
        adjacent.push_back({secondId, u.second->id()});
        network.addConstraint(AlignmentConstraint::CreateAlignedConstraint(adjacent, insideId));
    }
}


void create_adjacency_constraints(AreaHypothesis* area, HypToIdMap& hypIds, AlignmentGraph& network)
{
    AlignmentGraph::Id insideId = hypIds[area];
    AlignmentConstraint::AdjVec endpoints;

    for (auto& boundary : boost::make_iterator_range(area->beginBoundary(), area->endBoundary())) {
        ConstraintAdjacentArea adj;
        adj.id = hypIds[area->adjacentArea(boundary->getGateway())];
        adj.gatewayId = boundary->getGateway().id();

        if (area->isEndGateway(adj.gatewayId)) {
            endpoints.push_back(adj);
        } else {
            network.addConstraint(AlignmentConstraint::CreateAdjacentConstraint(adj, insideId));
        }
    }

    if (!endpoints.empty()) {
        network.addConstraint(AlignmentConstraint::CreateEndpointsConstraint(endpoints, insideId));
    } else {
        std::cerr << "WARNING: MaxLikelihoodCSP: No endpoints for area: " << area->extent().rectangleBoundary() << '\n';
    }
}


void create_all_neighbors_constraint(AreaHypothesis* area, HypToIdMap& hypIds, AlignmentGraph& network)
{
    AlignmentGraph::Id insideId = hypIds[area];
    AlignmentConstraint::AdjVec adjacent(area->numBoundaries());

    std::transform(area->beginBoundary(),
                   area->endBoundary(),
                   adjacent.begin(),
                   [&area, &hypIds](const AreaHypothesisBoundary* boundary) {
                       return ConstraintAdjacentArea{hypIds[area->adjacentArea(boundary->getGateway())],
                                                     boundary->getGateway().id()};
                   });

    network.addConstraint(AlignmentConstraint::CreateAllNeighborsConstraint(adjacent, insideId));
}


void create_exit_area_constraint(AreaHypothesis* area,
                                 const std::vector<AreaHypothesis*>& mutableAreas,
                                 HypToIdMap& hypIds,
                                 AlignmentGraph& network)
{
    // Go through the boundary for the constraint. All adjacent areas that are mutable will get added to the adjacent
    // areas to be considered
    AlignmentGraph::Id insideId = hypIds[area];
    AlignmentConstraint::AdjVec adjacent;

    for (auto& bnd : boost::make_iterator_range(area->beginBoundary(), area->endBoundary())) {
        if (utils::contains(mutableAreas, bnd->getOtherHypothesis(*area))) {
            ConstraintAdjacentArea constraint{hypIds[bnd->getOtherHypothesis(*area)], bnd->getGateway().id()};
            adjacent.emplace_back(constraint);
        }
    }

    network.addConstraint(AlignmentConstraint::CreateExitConstraint(adjacent, insideId, area->getType()));
}

}   // namespace hssh
}   // namespace vulcan

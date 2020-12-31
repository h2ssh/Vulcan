/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     parser.cpp
 * \author   Collin Johnson
 *
 * Implementation of AreaParser.
 */

#include "hssh/local_topological/area_detection/labeling/parser.h"
#include "hssh/local_topological/area_detection/labeling/area_proposal.h"
#include "hssh/local_topological/area_detection/labeling/belief_prop.h"
#include "hssh/local_topological/area_detection/labeling/boundary.h"
#include "hssh/local_topological/area_detection/labeling/boundary_classifier.h"
#include "hssh/local_topological/area_detection/labeling/debug.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_association.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_classifier.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_factor_graph.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_graph.h"
#include "hssh/local_topological/area_detection/labeling/invalid_area.h"
#include "hssh/local_topological/debug_info.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "utils/algorithm_ext.h"
#include "utils/ptr.h"
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>
#include <boost/range/iterator_range.hpp>
#include <iostream>
#include <queue>
#include <tuple>
#include <unordered_map>

#define DEBUG_CSP
#define DEBUG_PREVIOUS_MERGE
#define DEBUG_PRIORS

namespace vulcan
{
namespace hssh
{

template <typename ClassifyFunc>
void create_debug_hypotheses(const HypothesisGraph& graph,
                             std::vector<DebugHypothesis>& debug,
                             ClassifyFunc classifier);
template <typename TypeFunc>
void assign_types_to_graph(HypothesisGraph& graph, TypeFunc assigner);
HypothesisType const_type_func(HypothesisType type, const AreaHypothesis& h)
{
    return type;
}


AreaParser::AreaParser(std::shared_ptr<SmallScaleStarBuilder> starBuilder,
                       const MCMCSamplingParams& mcmcParams,
                       const std::string& classifierFile)
: hypClassifier_(std::make_unique<HypothesisClassifier>(classifierFile))
, boundaryClassifier_(std::make_unique<BoundaryClassifier>(classifierFile))
, starBuilder_(std::move(starBuilder))
, maxLikelihoodSolver_(starBuilder_, mcmcParams)
, exitedArea_(nullptr)
, enteredArea_(nullptr)
, nextHypothesisId_(0)
{
}


AreaParser::~AreaParser(void)
{
    // For std::unique_ptr
}


ParserResult AreaParser::parseGraph(const std::vector<Gateway>& gateways,
                                    const VoronoiSkeletonGrid& grid,
                                    const VoronoiIsovistField& isovistField,
                                    local_area_debug_info_t& debug)
{
    initializeParsing(debug);

    std::unique_ptr<AreaGraph> areaGraph(new AreaGraph(grid, gateways));
    std::unique_ptr<HypothesisGraph> hypGraph(new HypothesisGraph(*areaGraph, &grid, &isovistField, starBuilder_));

    initializeWithLikelihoods(*hypGraph);
    initializeEndpoints(*hypGraph);
    initializeWithLoopyBeliefProp(*hypGraph);

    storeDebuggingHypotheses(*hypGraph, debug);
    debug.graph = *areaGraph;

    AreaHypothesisAssociation hypAssocs;

    if (prevHypGraph_) {
        hypAssocs = AreaHypothesisAssociation(*prevHypGraph_, *hypGraph, grid);
    }

    HypToHypVec priorToCurrent = associatePriorToCurrent(*hypGraph, hypAssocs);

    AreaHypothesis* currentExited = findExitedArea(*hypGraph, priorToCurrent);
    AreaHypothesis* currentEntered = findEnteredArea(priorToCurrent);
    HypVec fixed = findFixedAreas(*hypGraph, currentExited, currentEntered, hypAssocs, priorToCurrent, grid);

    LabelingError solveResult = solveNetwork(*hypGraph, currentExited, currentEntered, fixed, debug);
    //     LabelingError solveResult = LabelingError::no_labeling_solution;

    // If the network can't be solved, propagate the error upwards
    if (solveResult != LabelingError::success) {
        return ParserResult(solveResult);
    }

    for (auto& node : boost::make_iterator_range(debug.graph.beginNodes(), debug.graph.endNodes())) {
        if (node->getType() & AreaNode::kGateway) {
            node->setProbability(node->getGateway().probability());
        }
    }

    // Otherwise update the state of the parser to account for the new solutions
    // Assign a new id to each area in the graph
    assignHypothesisIds(*hypGraph);

    AreaHypothesisAssociation priorToCurrentAssoc;
    AreaHypothesisAssociation currentToPriorAssoc;

    if (prevHypGraph_) {
        priorToCurrentAssoc = AreaHypothesisAssociation(*prevHypGraph_, *hypGraph, grid);
        currentToPriorAssoc = AreaHypothesisAssociation(*hypGraph, *prevHypGraph_, prevSkeleton_);
    }

    if (exitedArea_) {
        AreaHypothesis* newExited = findExitedAreaInCurrent(priorToCurrentAssoc, *hypGraph, currentToPriorAssoc);

        if (newExited) {
            if (newExited->getType() != exitedArea_->getType()) {
                std::cerr << "\n\nWARNING: Exited area type changed from " << exitedArea_->getType() << " to "
                          << newExited->getType() << "\n\n";
            }

            std::cout << "Associated prior exited area " << exitedArea_->rectangleBoundary() << " with "
                      << newExited->rectangleBoundary() << " current areas.\n";
            exitedArea_ = newExited;
        } else {
            std::cerr << "\n\nERROR: Failed to find exit! " << exitedArea_->rectangleBoundary() << "\n\n\n";
            // If the exited area can't be found, clear it out because it won't be found on the next update either!
            exitedArea_ = nullptr;
            assert(exitedArea_);
        }
    }

    if (enteredArea_) {
        AreaHypothesis* newEntered = findEnteredAreaInCurrent(priorToCurrentAssoc, *hypGraph, currentToPriorAssoc);

        if (newEntered) {
            if (enteredArea_->getType() != newEntered->getType()) {
                std::cerr << "\n\nWARNING: Entered area type changed from " << enteredArea_->getType() << " to "
                          << newEntered->getType() << "\n\n";
            }

            std::cout << "Associated prior entered area " << enteredArea_->rectangleBoundary() << " with "
                      << newEntered->rectangleBoundary() << " current areas.\n";
            enteredArea_ = newEntered;
        } else {
            std::cerr << "\n\nERROR: Failed to find entry! " << enteredArea_->rectangleBoundary() << "\n\n\n";
            // If the exited area can't be found, clear it out because it won't be found on the next update either!
            enteredArea_ = nullptr;
        }
    }

    // WARNING: Obliterates all pointers into prevHypGraph_. Only pointers into hypGraph are safe.
    prevAreaGraph_ = std::move(areaGraph);
    prevHypGraph_ = std::move(hypGraph);
    prevSkeleton_ = grid;

    // After the hyp graph is changed, then create the new hypotheses
    // If the labeling fails, then just return no areas
    try {
        auto proposals = prevHypGraph_->createProposalsForCurrentHypotheses(grid);
        return ParserResult(proposals, logProbSolution_);
    } catch (InvalidAreaException& e) {
        std::cout << e.what() << '\n';
        return ParserResult(LabelingError::no_labeling_solution);
    }
}


void AreaParser::handleTransition(int exitedId, int enteredId, const boost::optional<Gateway>& gateway)
{
    // At least one graph must have been parsed in order for an event to have occurred
    assert(prevHypGraph_);

    bool shouldHaveExited = !enteredArea_;   // if there was a previously entered area, then must have
                                             // an exit from that area for the next transition
    exitedArea_ = nullptr;
    enteredArea_ = nullptr;

    exitTransition_ = gateway;

    for (AreaHypothesis* hyp :
         boost::make_iterator_range(prevHypGraph_->beginHypothesis(), prevHypGraph_->endHypothesis())) {
        if (hyp->getId() == exitedId) {
            exitedArea_ = hyp;
        } else if (hyp->getId() == enteredId) {
            enteredArea_ = hyp;
        }
    }

    std::cout << "\n\nEXITED:" << exitedId << "  ENTERED:" << enteredId << " Found exit? " << (exitedArea_ != nullptr)
              << " Found entered? " << (enteredArea_ != nullptr) << "\n\n\n";

    // Must have entered somewhere!
    assert(enteredArea_);
    assert(!shouldHaveExited || exitedArea_);
}


void AreaParser::sendDebug(system::DebugCommunicator& communicator)
{
    maxLikelihoodSolver_.sendDebug(communicator);
}


void AreaParser::initializeParsing(local_area_debug_info_t& debug)
{
    debug.maximumLikelihoodHypotheses.clear();
    debug.unnormalizedHypotheses.clear();
    debug.boostingHypotheses.clear();
    debug.isovistHypotheses.clear();
    haveSolution = false;
}


void AreaParser::initializeWithLikelihoods(HypothesisGraph& graph)
{
    for (auto hyp : boost::make_iterator_range(graph.beginHypothesis(), graph.endHypothesis())) {
        hyp->setTypeDistribution(hypClassifier_->calculateDistribution(hyp->features()));
        hyp->setType(hypClassifier_->classify(hyp->features()));
    }

    for (auto bnd : boost::make_iterator_range(graph.beginBoundary(), graph.endBoundary())) {
        bnd->setProbability(bnd->getGateway().probability());
    }
}


void AreaParser::initializeWithLoopyBeliefProp(HypothesisGraph& graph)
{
    const double kMaxGwyProb = 0.999;

    auto factorGraph = convert_hypothesis_graph_to_factor_graph(graph, *hypClassifier_, *boundaryClassifier_);

    std::cout << "Created factor graph containing:\n"
              << "Variables: " << factorGraph.graph.sizeVars() << '\n'
              << "Factors:   " << factorGraph.graph.sizeFactors() << '\n'
              << "Edges:     " << factorGraph.graph.sizeEdges() << '\n'
              << "for HypGraph with:\n"
              << "Areas:      " << graph.numHypotheses() << '\n'
              << "Boundaries: " << graph.numBoundaries() << '\n';

    loopy_belief_propagation(factorGraph.graph, UpdateType::sum);

    // Go through and save the marginals as a type distribution for each hypothesis
    for (auto& var : boost::make_iterator_range(factorGraph.graph.beginVars(), factorGraph.graph.endVars())) {
        if (var->numStates() == 3) {
            auto& hyp = factorGraph.varToHyp.at(var->id());
            auto marginal = var->marginal();
            HypothesisTypeDistribution dist;
            dist.path = marginal[kPathIdx];
            dist.decision = marginal[kDecisionIdx];
            dist.destination = marginal[kDestIdx];

            std::cout << "Hyp marginal: " << hyp->rectangleBoundary() << '\n';

            auto oldDist = hypClassifier_->calculateDistribution(hyp->features());
            std::cout << "Type dist:  Old\tNew\n"
                      << oldDist.path << '\t' << dist.path << '\n'
                      << oldDist.decision << '\t' << dist.decision << '\n'
                      << oldDist.destination << '\t' << dist.destination << '\n';

            hyp->setTypeDistribution(dist);
            hyp->setType(dist.mostAppropriate());
        } else if (var->numStates() == 2) {
            auto& bnd = factorGraph.varToBoundary.at(var->id());
            auto marginal = var->marginal();
            double prob = marginal[1];
            // clamp to a more reasonable range than [0, 1] as those 0 or 1 throw things off for certain
            prob = std::max(std::min(kMaxGwyProb, prob), 1.0 - kMaxGwyProb);
            bnd->setProbability(prob);
            std::cout << "Marginal for boundary " << bnd->getGateway().boundary() << ": yes: " << marginal[1]
                      << " no: " << marginal[0] << " likelihood: " << bnd->getGateway().probability() << '\n';
        }
    }
}


void AreaParser::initializeEndpoints(HypothesisGraph& graph)
{
    for (auto hyp : boost::make_iterator_range(graph.beginHypothesis(), graph.endHypothesis())) {
        hyp->findPathEndpoints();
    }
}


AreaParser::HypToHypVec AreaParser::associatePriorToCurrent(HypothesisGraph& currentGraph,
                                                            const AreaHypothesisAssociation& hypAssoc)
{
    // If there are no priors, then there can be no association with the current graph
    if (!prevHypGraph_) {
        return HypToHypVec();
    }

#ifdef DEBUG_PRIORS
    std::cout << "DEBUG: AreaParser: Associating priors to new hypotheses:\n";
#endif

    HypToHypVec prevToCurrentHyps;
    std::unordered_set<AreaHypothesis*> mergedHyps;

    // don't lose the exits due to multiple associations!
    HypVec exitAreas(hypAssoc.beginAssociated(exitedArea_), hypAssoc.endAssociated(exitedArea_));

    for (AreaHypothesis* prior :
         boost::make_iterator_range(prevHypGraph_->beginHypothesis(), prevHypGraph_->endHypothesis())) {
        HypVec associatedHyps(hypAssoc.beginAssociated(prior), hypAssoc.endAssociated(prior));

        if (prior != exitedArea_) {
            utils::erase_remove_if(associatedHyps, [&exitAreas](AreaHypothesis* h) {
                return utils::contains(exitAreas, h);
            });
        }

        prevToCurrentHyps[prior->getId()] = associatedHyps;

        // Issue a warning if there is a many to one association of prior to current. This condition shouldn't happen
        // given the maintenance of gateways between updates. The new areas should always be contained within the prior
        // areas, plus some new areas probably exist
        for (auto& hyp : associatedHyps) {
            if (mergedHyps.find(hyp) != mergedHyps.end()) {
                std::cerr << "WARNING: AreaParser:" << hyp->rectangleBoundary()
                          << " is associated with multiple priors. Ignoring update? " << utils::contains(exitAreas, hyp)
                          << '\n';
            }
        }

        mergedHyps.insert(associatedHyps.begin(), associatedHyps.end());

#ifdef DEBUG_PRIORS
        std::cout << prior->rectangleBoundary() << " -> ";
        for (auto hyp : associatedHyps) {
            std::cout << hyp->rectangleBoundary() << ' ';
        }
        std::cout << '\n';
#endif
    }

    return prevToCurrentHyps;
}


AreaHypothesis* AreaParser::findExitedArea(HypothesisGraph& currentGraph, const HypToHypVec& hypAssoc)
{
    AreaHypothesis* exited = nullptr;
    if (exitedArea_) {
        // If the prior exited is associated with current areas, then merge them to make the new exited area
        if (!hypAssoc.at(exitedArea_->getId()).empty()) {
            if (hypAssoc.at(exitedArea_->getId()).size() > 1) {
                std::cout << "Exit associated with multiple areas. MERGING. DANGER!\n";
            }
            exited = currentGraph.mergeHypotheses(hypAssoc.at(exitedArea_->getId()), exitedArea_->getType());
        }
        // Otherwise, a gateway in the exited area disappeared (not the transition), so it was merged into
        // a single current area. Thus, need to associate the current to the prior to find out which
        // current area the association was created for
        else {
            AreaHypothesisAssociation curToPrior(currentGraph, *prevHypGraph_, prevSkeleton_);

            for (auto hyp : boost::make_iterator_range(currentGraph.beginHypothesis(), currentGraph.endHypothesis())) {
                if (curToPrior.isAssociatedWith(hyp, exitedArea_) && isBoundedByExitTransition(hyp)) {
                    return hyp;
                }
            }
        }
    }

    return exited;
}


AreaHypothesis* AreaParser::findExitedAreaInCurrent(const AreaHypothesisAssociation& priorToCur,
                                                    HypothesisGraph& currentGraph,
                                                    const AreaHypothesisAssociation& curToPrior)
{
    if (exitedArea_) {
        // Associate prior entered with current because it may have been split into pieces
        for (auto newHyp : boost::make_iterator_range(priorToCur.beginAssociated(exitedArea_),
                                                      priorToCur.endAssociated(exitedArea_))) {
            if (isBoundedByExitTransition(newHyp)) {
                return newHyp;
            }
        }

        for (auto hyp : boost::make_iterator_range(currentGraph.beginHypothesis(), currentGraph.endHypothesis())) {
            if (curToPrior.isAssociatedWith(hyp, exitedArea_) && isBoundedByExitTransition(hyp)) {
                return hyp;
            }
        }
    }

    return nullptr;
}


AreaHypothesis* AreaParser::findEnteredArea(const HypToHypVec& hypAssoc)
{
    AreaHypothesis* entered = nullptr;
    if (enteredArea_ && exitTransition_)   // only can associated entered if the entry transition is known
    {
        // Go through the associations with the entered area and find which one contains a boundary similar to
        // the previously crossed transition gateway
        for (auto& newHyp : hypAssoc.at(enteredArea_->getId())) {
            if (isBoundedByExitTransition(newHyp)) {
                entered = newHyp;
                break;
            }
        }

        if (entered) {
            std::cout << "INFO: Found entered area via " << exitTransition_->boundary() << " into "
                      << entered->rectangleBoundary() << '\n';
            entered->setType(enteredArea_->getType());
        } else {
            std::cerr << "ERROR: Failed to find entered area " << enteredArea_->rectangleBoundary() << " via "
                      << exitTransition_->boundary() << '\n';
        }

        //         assert(entered);
    }

    return entered;
}


AreaHypothesis* AreaParser::findEnteredAreaInCurrent(const AreaHypothesisAssociation& priorToCur,
                                                     HypothesisGraph& currentGraph,
                                                     const AreaHypothesisAssociation& curToPrior)
{
    if (enteredArea_ && exitTransition_)   // only can associated entered if the entry transition is known
    {
        // Associate prior entered with current because it may have been split into pieces
        for (auto newHyp : boost::make_iterator_range(priorToCur.beginAssociated(enteredArea_),
                                                      priorToCur.endAssociated(enteredArea_))) {
            if (isBoundedByExitTransition(newHyp)) {
                return newHyp;
            }
        }

        // If not found, then it was likely merged into a new area, so search in the other direction
        for (auto hyp : boost::make_iterator_range(currentGraph.beginHypothesis(), currentGraph.endHypothesis())) {
            if (curToPrior.isAssociatedWith(hyp, enteredArea_) && isBoundedByExitTransition(hyp)) {
                return hyp;
            }
        }
    }

    return nullptr;
}


bool AreaParser::isBoundedByExitTransition(AreaHypothesis* hyp)
{
    for (auto& bnd : boost::make_iterator_range(hyp->beginBoundary(), hyp->endBoundary())) {
        // Find which boundary was crossed
        if (exitTransition_->isSimilarTo(bnd->getGateway())) {
            return true;
        }
    }

    return false;
}


AreaParser::HypVec AreaParser::findFixedAreas(HypothesisGraph& graph,
                                              AreaHypothesis* exitedArea,
                                              AreaHypothesis* enteredArea,
                                              const AreaHypothesisAssociation& hypAssoc,
                                              const HypToHypVec& priorToCurrent,
                                              const VoronoiSkeletonGrid& grid)
{
    // If there is no exited area, then a single connected HypothesisGraph exists, so all areas can be changed.
    if (!exitedArea) {
        std::cout << "INFO: AreaParser: Not fixing any areas. No exited area exists.\n";
        return HypVec();
    }

    // Create subgraphs in the current hypothesis graph by removing the exited area
    auto subgraphs = graph.findSubgraphs(exitedArea);

    int enteredSubgraph = findEnteredSubgraph(subgraphs, enteredArea);

    if (!enteredArea_ || (enteredSubgraph == -1)) {
        std::cout << "WARNING: AreaParser: Failed to find the entered area. Something must have changed. Not fixing "
                     "any areas.\n";
        return HypVec();
    }

    std::cout << "Entered subgraph: " << enteredSubgraph << '\n';

    // There are some

    HypVec fixedAreas;

    for (std::size_t n = 0; n < subgraphs.size(); ++n) {
        // Fixed areas belong to all subgraphs not part of the entered subgraph
        if (enteredSubgraph != static_cast<int>(n)) {
            boost::push_back(fixedAreas, subgraphs[n]);
        }
    }

    std::cout << "INFO: AreaParser: Initial fixed areas: ";
    for (auto& hyp : fixedAreas) {
        std::cout << hyp->center() << ' ';
    }
    std::cout << '\n';

    // Find the ids of all priors that should be merged in the new graph as a result of being fixed
    std::unordered_set<int> toMerge;
    for (auto& hypToVec : priorToCurrent) {
        bool shouldMerge = true;
        for (auto current : hypToVec.second) {
            // If any of the areas aren't fixed, then don't do the whole merge
            if (!utils::contains(fixedAreas, current)) {
                shouldMerge = false;
            }
        }

        if (shouldMerge) {
            toMerge.insert(hypToVec.first);
        }
    }

    // Merge each of the areas found to contain only fixed areas
    std::vector<AreaHypothesis*> mergedAreas;
    std::vector<AreaHypothesis*> unmergedAreas;
    for (auto prior : toMerge) {
        const HypVec& current = priorToCurrent.at(prior);

        // Only merge areas that haven't been merged into another area
        unmergedAreas.clear();
        std::copy_if(current.begin(),
                     current.end(),
                     std::back_inserter(unmergedAreas),
                     [&mergedAreas](AreaHypothesis* hyp) {
                         return !utils::contains(mergedAreas, hyp);
                     });

        if (unmergedAreas.size() != current.size()) {
            std::cout << "INFO:AreaParser: Multiple matches for some areas, so they were ignored.\n";
        }

        AreaHypothesis* mergedArea = graph.mergeHypotheses(current, prevHypGraph_->at(prior)->getType());

        if (mergedArea) {
            // Erase all fixed areas were merged
            utils::erase_remove_if(fixedAreas, [&current](AreaHypothesis* hyp) {
                return utils::contains(current, hyp);
            });

            mergedAreas.push_back(mergedArea);
            fixedAreas.push_back(mergedArea);
            std::cout << "Prior to merge:" << prior << " Merged boundary:" << mergedArea->rectangleBoundary() << '\n';
        } else if (!current.empty()) {
            std::cout << "WARNING: Failed to merge hypotheses:\n";
            for (auto& hyp : current) {
                std::cout << hyp->rectangleBoundary() << ' ';
            }
            std::cout << '\n';

            // Save each of the areas individually, even when a failure occurs, as they still should remain
            // individually fixed.
            boost::push_back(mergedAreas, boost::as_array(current));
            boost::push_back(fixedAreas, boost::as_array(current));
        }
    }

    // For each fixed area, if it wasn't the result of a merge, then it needs to have a type assigned
    for (auto& hyp : fixedAreas) {
        if (!utils::contains(mergedAreas, hyp)) {
            hyp->setType(hypClassifier_->classify(hyp->features()));
            std::cout << "Set " << hyp->rectangleBoundary() << " to " << hyp->getType() << '\n';
        } else {
            std::cout << "Left " << hyp->rectangleBoundary() << " as " << hyp->getType() << '\n';
        }
    }

    return fixedAreas;
}


int AreaParser::findEnteredSubgraph(const std::vector<HypVec>& subgraphs, AreaHypothesis* enteredArea)
{
    // Search through the subgraphs and the subgraph, if any contains the entered area, then it will be the
    // subgraph to use

    for (std::size_t n = 0; n < subgraphs.size(); ++n) {
        if (utils::contains(subgraphs[n], enteredArea)) {
            return n;
        }
    }

    std::cerr << "ERROR: AreaParser: No subgraphs contained the entered area.\n";
    return -1;
}


LabelingError AreaParser::solveNetwork(HypothesisGraph& graph,
                                       AreaHypothesis* exitedArea,
                                       AreaHypothesis* enteredArea,
                                       const HypVec& fixedAreas,
                                       local_area_debug_info_t& debug)
{
    // Find the areas in the graph that can actually be changed
    HypVec mutableAreas;
    for (AreaHypothesis* hyp : boost::make_iterator_range(graph.beginHypothesis(), graph.endHypothesis())) {
        // If the area isn't fixed or the exited area, then it can be changed
        if (!utils::contains(fixedAreas, hyp) && (exitedArea != hyp) && (enteredArea != hyp)) {
            mutableAreas.push_back(hyp);
        }
    }

    // Search for a solution to the problem
    auto solution = maxLikelihoodSolver_.solve(fixedAreas, mutableAreas, exitedArea, enteredArea, *boundaryClassifier_);

    // If a fixed areas is failing a constraint, then try the search again without fixed areas.
    //     if(solution.errorCode() == CSPSolution::fixed_area_failing_constraints)
    //     {
    //         std::cerr << "WARNING: AreaParser: Fixed areas are invalid. Searching for new solution without any fixed
    //         areas.\n"; mutableAreas.clear(); boost::push_back(mutableAreas,
    //         boost::make_iterator_range(graph.beginHypothesis(), graph.endHypothesis())); HypVec noFixedAreas;
    //         solution = maxLikelihoodSolver_.solve(noFixedAreas, mutableAreas, nullptr, nullptr,
    //         *boundaryClassifier_);
    //     }

    // If successful, then apply the solution.
    if (solution.errorCode() == CSPSolution::successful) {
        std::cout << "Found solution to graph.\n";
        solution.apply(graph);
        logProbSolution_ = solution.logProb();
        return LabelingError::success;
    } else {
        // no areas were fixed if a second failure occurs
        //         assert(solution.errorCode() != CSPSolution::fixed_area_failing_constraints);

        std::cerr << "ERROR: AreaParser: Failed to find solution to graph.\n";
        return LabelingError::no_labeling_solution;
    }
}


void AreaParser::assignHypothesisIds(HypothesisGraph& graph)
{
    // The ids aren't required to be monotonically increasing or in anyway incrementing by 1 every time, they just
    // need to be unique. Thus, we can assign new ids to every new area.
    for (AreaHypothesis* hyp : boost::make_iterator_range(graph.beginHypothesis(), graph.endHypothesis())) {
        hyp->setId(nextHypothesisId_++);

        std::cout << hyp->getId() << " : " << hyp->rectangleBoundary() << '\n';
    }
}


void AreaParser::storeDebuggingHypotheses(HypothesisGraph& graph, local_area_debug_info_t& debug)
{
    using namespace std::placeholders;
    using HypInfo = std::tuple<AreaHypothesis*, HypothesisType, HypothesisTypeDistribution>;

    // Save the types initially assigned to the hypotheses
    std::vector<HypInfo> initialTypes(graph.numHypotheses());
    std::transform(graph.beginHypothesis(), graph.endHypothesis(), initialTypes.begin(), [](AreaHypothesis* h) {
        return std::make_tuple(h, h->getType(), h->getTypeDistribution());
    });

    assign_types_to_graph(graph, [this](const AreaHypothesis& h) {
        return hypClassifier_->classify(h.features());
    });
    create_debug_hypotheses(graph, debug.maximumLikelihoodHypotheses, [this](const auto& hyp) {
        return hypClassifier_->calculateDistribution(hyp->features());
    });

    assign_types_to_graph(graph, [this](const AreaHypothesis& h) {
        return hypClassifier_->calculateRawDistribution(h.features()).mostAppropriate();
    });
    create_debug_hypotheses(graph, debug.unnormalizedHypotheses, [this](const auto& hyp) {
        return hypClassifier_->calculateRawDistribution(hyp->features());
    });

    assign_types_to_graph(graph, [this](const AreaHypothesis& h) {
        return hypClassifier_->calculateRawBoostingDistribution(h.features()).mostAppropriate();
    });
    create_debug_hypotheses(graph, debug.boostingHypotheses, [this](const auto& hyp) {
        return hypClassifier_->calculateRawBoostingDistribution(hyp->features());
    });

    assign_types_to_graph(graph, [this](const AreaHypothesis& h) {
        return hypClassifier_->calculateRawIsovistDistribution(h.features()).mostAppropriate();
    });
    create_debug_hypotheses(graph, debug.isovistHypotheses, [this](const auto& hyp) {
        return hypClassifier_->calculateRawIsovistDistribution(hyp->features());
    });

    // Restore the initially assigned types
    for (auto& hypToType : initialTypes) {
        std::get<0>(hypToType)->setType(std::get<1>(hypToType));
        std::get<0>(hypToType)->setTypeDistribution(std::get<2>(hypToType));
    }
}


template <typename ClassifyFunc>
void create_debug_hypotheses(const HypothesisGraph& graph, std::vector<DebugHypothesis>& debug, ClassifyFunc classifier)
{
    std::transform(graph.beginHypothesis(),
                   graph.endHypothesis(),
                   std::back_inserter(debug),
                   [&classifier](const AreaHypothesis* h) -> DebugHypothesis {
                       return h->toDebug(classifier(h));
                   });
}


template <typename TypeFunc>
void assign_types_to_graph(HypothesisGraph& graph, TypeFunc assigner)
{
    std::for_each(graph.beginHypothesis(), graph.endHypothesis(), [&assigner](AreaHypothesis* h) {
        h->setType(assigner(*h));
    });
}

}   // namespace hssh
}   // namespace vulcan

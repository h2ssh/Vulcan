/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     parser.h
* \author   Collin Johnson
*
* Definition of AreaParser.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREAS_PARSER_H
#define HSSH_LOCAL_TOPOLOGICAL_AREAS_PARSER_H

#include <hssh/local_topological/error.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <hssh/local_topological/area_detection/labeling/max_likelihood_csp.h>
#include <array>
#include <map>
#include <memory>
#include <set>
#include <vector>

namespace vulcan
{
namespace hssh
{

class BoundaryClassifier;
class AreaGraph;
class AreaHypothesisAssociation;
class HypothesisClassifier;
class HypothesisGraph;
struct hypothesis_evaluator_params_t;
struct local_area_debug_info_t;
struct MCMCSamplingParams;

struct ParserResult
{
    std::vector<AreaProposal> proposals;
    double logProb;
    LabelingError result;

    // For success, simply pass the proposals
    explicit ParserResult(const std::vector<AreaProposal>& proposals, double logProb)
    : proposals(proposals)
    , logProb(logProb)
    , result(LabelingError::success)
    {
    }

    // For errors, pass the error code. Error can't be success because there must be proposals!
    explicit ParserResult(LabelingError error)
    : result(error)
    {
        assert(result != LabelingError::success);
    }
};


/**
* AreaParser finds the areas within the AreaGraph. First, the graph is broken into hypotheses, where each hypothesis
* is segmented based on the gateways in the map. The hypotheses and boundaries are represented by AreaHypothesis and
* AreaHypothesisBoundary.
*
* Each hypothesis is classified using a set of rules described in AreaHypothesis. Each boundary is checked for validity,
* that is, whether the areas on each side of the boundary can be adjacent within the representation. Whenever the
* classification of a hypothesis or the validity of a boundary changes, the adjacent areas are marked as dirty and must
* be reconsidered. This process iterates until the no changes are made.
*/
class AreaParser
{
public:

    /**
    * Constructor for AreaParser.
    *
    * \param    starBuilder             Strategy to use for building the small-scale stars
    * \param    mcmcParams              Parameters controlling the MCMC sampling algorithm used internally
    * \param    classifierFile          File containing parameters for hypothesis classifier
    */
    AreaParser(std::shared_ptr<SmallScaleStarBuilder> starBuilder,
               const MCMCSamplingParams& mcmcParams,
               const std::string& classifierFile);

    /**
    * Destructor for AreaParser.
    */
    ~AreaParser(void);

    /**
    * parseGraph parses the HypothesisGraph into a collection of AreaProposals. These proposals can
    * then be turned into actual LocalAreas.
    *
    * \param    gateways        Gateways to be parsed into areas
    * \param    grid            Skeleton from which the graph was extracted
    * \param    isovistField    Field with isovists calculated for the skeleton
    * \param    debug           Debug info created during the parsing
    * \return   AreaProposals for the graph.
    */
    ParserResult parseGraph(const std::vector<Gateway>& gateways,
                            const VoronoiSkeletonGrid&  grid,
                            const VoronoiIsovistField&  isovistField,
                            local_area_debug_info_t&    debug);

    /**
    * handleTransition incorporates information about the latest transition into the parser. The area ids are associated
    * with the most recently returned AreaProposals.
    *
    * When an area is exited, its boundaries and labels are fixed.
    *
    * NOTE: Only the most recent transition affects the labeling of the graph. Thus, if multiple events occur on a
    * single update, only the final event will be used for the remaining labeling.
    *
    * \param    exitedId            Id of the exited area
    * \param    enteredId           Id of the entered area
    * \param    gateway             Gateway that was crossed from exited to entered (will be none on the initial entered)
    */
    void handleTransition(int exitedId, int enteredId, const boost::optional<Gateway>& gateway);

    /**
    * sendDebug sends internal debugging state created during the parsing process.
    */
    void sendDebug(system::DebugCommunicator& communicator);

private:

    using HypVec = std::vector<AreaHypothesis*>;
    using HypToHypVec = std::unordered_map<int, HypVec>;

    AreaParser(const AreaParser& rhs)     = delete;
    void operator=(const AreaParser& rhs) = delete;

    bool haveSolution;
    double logProbSolution_ = 0.0;

    std::unique_ptr<HypothesisClassifier> hypClassifier_;
    std::unique_ptr<BoundaryClassifier>   boundaryClassifier_;
    std::shared_ptr<SmallScaleStarBuilder> starBuilder_;
    MaxLikelihoodCSP                       maxLikelihoodSolver_;

    std::unique_ptr<AreaGraph> prevAreaGraph_;
    std::unique_ptr<HypothesisGraph> prevHypGraph_;
    VoronoiSkeletonGrid prevSkeleton_;
    AreaHypothesis* exitedArea_;        // area last exited by the robot (can be null!) -- pointer into prevHypGraph_
    AreaHypothesis* enteredArea_;       // area last entered by the robot (can be null!) -- pointer in to prevHypGraph_
    boost::optional<Gateway> exitTransition_;   // gateway crossed from exited to entered
    int nextHypothesisId_;

    // Main processing algorithm
    void initializeParsing(local_area_debug_info_t& debug);
    void initializeWithLikelihoods(HypothesisGraph& graph);
    void initializeWithLoopyBeliefProp(HypothesisGraph& graph);
    void initializeEndpoints(HypothesisGraph& graph);
    HypToHypVec associatePriorToCurrent(HypothesisGraph& currentGraph, const AreaHypothesisAssociation& hypAssoc);
    AreaHypothesis* findExitedArea(HypothesisGraph& currentGraph, const HypToHypVec& hypAssoc);
    AreaHypothesis* findExitedAreaInCurrent(const AreaHypothesisAssociation& priorToCur,
                                            HypothesisGraph& currentGraph,
                                            const AreaHypothesisAssociation& curToPrior);
    AreaHypothesis* findEnteredArea(const HypToHypVec& hypAssoc);
    AreaHypothesis* findEnteredAreaInCurrent(const AreaHypothesisAssociation& priorToCur,
                                             HypothesisGraph& currentGraph,
                                             const AreaHypothesisAssociation& curToPrior);
    bool isBoundedByExitTransition(AreaHypothesis* hyp);
    HypVec findFixedAreas(HypothesisGraph& graph,
                          AreaHypothesis* exitedArea,
                          AreaHypothesis* enteredArea,
                          const AreaHypothesisAssociation& hypAssoc,
                          const HypToHypVec& priorToCurrent,
                          const VoronoiSkeletonGrid& grid);
    int findEnteredSubgraph(const std::vector<HypVec>& subgraphs, AreaHypothesis* enteredArea);
    LabelingError solveNetwork(HypothesisGraph& graph,
                               AreaHypothesis* exitedArea,
                               AreaHypothesis* enteredArea,
                               const std::vector<AreaHypothesis*>& fixedAreas,
                               local_area_debug_info_t& debug);
    void assignHypothesisIds(HypothesisGraph& graph);
    void storeDebuggingHypotheses(HypothesisGraph& graph, local_area_debug_info_t& debug);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREAS_PARSER_H

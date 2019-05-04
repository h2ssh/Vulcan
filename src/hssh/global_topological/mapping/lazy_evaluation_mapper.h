/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lazy_evaluation_mapper.h
* \author   Collin Johnson
*
* Declaration of LazyEvaluationMapper.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LAZY_EVALUATION_MAPPER_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LAZY_EVALUATION_MAPPER_H

#include <deque>
#include <queue>
#include <fstream>
#include <hssh/global_topological/mapping/topological_mapper.h>
#include <hssh/global_topological/mapping/probability_evaluator.h>
#include <hssh/global_topological/actions.h>
#include <hssh/global_topological/measurements.h>
#include <hssh/global_topological/params.h>

namespace vulcan
{
namespace hssh
{

class MapOptimizer;
class TopologicalLocalizer;

const std::string LAZY_EVALUATION_MAPPER_TYPE("lazy-evaluation");

/**
* LazyEvaluationMapper is a topological mapping algorithm the uses map likelihoods as a heuristic
* for searching the space of potential map hypotheses. The algorithm focuses the expansion of map
* hypotheses on the most likely map hypotheses based on current measurements or a heuristic for
* older hypotheses.
*
* Each hypothesis has both a calculated and estimated likelihood. The calculated likelihood
* incorporates all measurements up through the last topology event included in the map hypothesis.
* The estimated likelihood guesses how the likelihood of a hypothesis will evolve as new measurements
* are incorporated by considering how the likelihood of already expanded maps changes.
*
* The configuration parameters for LazyEvaluationMapper are:
*
*   [LazyEvaluationMapperParameters]
*   min_hypotheses_to_expand = minimum number of hypotheses to expand for a given event, even if they are less likely than the current best
*
*/
class LazyEvaluationMapper : public TopologicalMapper
{
public:

    /**
    * Constructor for LazyEvaluationMapper.
    *
    * \param    params          Parameters for the mapper
    * \param    manager         Place manager that maintains the LPMs for the topological places
    */
    LazyEvaluationMapper(const topological_mapper_params_t& params, MetricMapCache& manager);

    // TopologicalMapper interface
    void setCorrectMap(uint32_t id);
    void setCorrectMap(TopoMapPtr map);
    void updateMap    (const topology_action_t& action, const topology_measurements_t& measurements);

    TopoMapPtr getUsableHypothesis(void) const { return mostLikelyHypothesis; }

private:

    typedef std::vector<topology_action_t> reverse_actions_t;

    // topology_event_t stores all data associated with a given place
    struct topology_event_t
    {
        topology_action_t       enteredAction;
        topology_action_t       exitedAction;
        topology_measurements_t measurements;

        topology_event_t(void)
        {
        }

        topology_event_t(const topology_action_t& entered, const topology_measurements_t& measurements)
            : enteredAction(entered)
            , measurements(measurements)
        {
        }
    };

    class TopoMapPriority
    {
    public:

        bool operator()(const TopoMapPtr& lhs, const TopoMapPtr& rhs)
        {
            return (lhs->getEstimatedLogPosterior() < rhs->getEstimatedLogPosterior()) ||
                   ((lhs->getEstimatedLogPosterior() == rhs->getEstimatedLogPosterior()) && (lhs.get() < rhs.get()));
        }
    };

    void addEventToQueue   (const topology_action_t& action, const topology_measurements_t& measurements);
    void handleExitedEvent (const topology_action_t& action, const topology_measurements_t& measurements);
    void handleEnteredEvent(const topology_action_t& action, const topology_measurements_t& measurements);
    void handleReverseEvent(const topology_action_t& action, const topology_measurements_t& measurements);
    bool initializeIfNeeded(const topology_action_t& action, const topology_measurements_t& measurements);

    void constructHypothesisQueue(void);

    void expandHypothesis  (TopoMapPtr& hypothesis);
    void localizeHypothesis(TopoMapPtr& hypothesis);
    void movedToKnownPlace (TopoMapPtr& hypothesis, const LargeScaleStar& placeStar, const topology_event_t& event);
    void addNewPlace       (TopoMapPtr& hypothesis, const LargeScaleStar& placeStar, const topology_measurements_t& measurements);
    void closeLoops        (TopoMapPtr& hypothesis, const LargeScaleStar& placeStar, const topology_measurements_t& measurements);

    void   processNewMapHypothesis      (TopoMapPtr& child, TopoMapPtr& parent, const topology_measurements_t& measurements);
    void   calculateHypothesisLikelihood(TopoMapPtr& child, TopoMapPtr& parent, const topology_measurements_t& measurements);
    void   calculateEstimatedProbabilities(uint32_t depth, hypothesis_probability_t& probability);
    void   updateHeuristic              (TopoMapPtr& child, TopoMapPtr& parent);
    double likelihoodHeurstic           (uint32_t hypothesisDepth);   // calc from depth+1 to height
    double priorHeuristic               (uint32_t depth, double calculatedPrior);

    bool isValidHypothesis(TopoMapPtr& hypothesis, const LargeScaleStar& placeStar);
    bool haveFinishedUpdate(void);

    bool hypothesisContainsAllEvents(uint32_t depth) { return depth+1 == events.size(); }

    topology_event_t&  enteredPlaceEvent(const TopoMapPtr& hyp) { return events[hyp->getDepth()+1];     }
    topology_event_t&  exitedPlaceEvent (const TopoMapPtr& hyp) { return events[hyp->getDepth()];       }
    reverse_actions_t& reverseActions   (const TopoMapPtr& hyp) { return pathEvents[hyp->getDepth()+1]; }

    TopoMapPtr mostLikelyHypothesis;
    uint32_t   numHypothesesExpanded;
    uint32_t   numHypothesesEvaluated;

    std::priority_queue<TopoMapPtr, std::vector<TopoMapPtr>, TopoMapPriority> mapQueue;
    std::vector<double>             heuristicLikelihood;
    std::vector<double>             heuristicPrior;
    std::deque<topology_event_t>    events;
    std::deque<reverse_actions_t>   pathEvents;

    // INVARIANT: pathEvents.size() == events.size()+1
    // pathEvents[n] is all path events that occurred between events[n-1] and events[n]

    std::unique_ptr<TopologicalLocalizer> localizer;
    std::unique_ptr<MapOptimizer>         optimizer;
    HypothesisProbabilityEvaluator        probabilityEvaluator;

    lazy_evaluation_mapper_params_t params;

    std::ofstream evaluationLog;
    std::ofstream leafDepthLog;
};

}
}

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LAZY_EVALUATION_MAPPER_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     hypothesis_generator.h
 * \author   Collin Johnson
 *
 * Declaration of HypothesisGenerator abstract base class.
 */

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_HYPOTHESIS_GENERATOR_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_HYPOTHESIS_GENERATOR_H

#include "hssh/global_topological/localization/location_distribution.h"
#include "hssh/global_topological/state.h"
#include "hssh/global_topological/utils/visit.h"
#include <vector>

namespace vulcan
{
namespace hssh
{

class ProbabilityHeuristics;

/**
 * HypothesisGenerator is a base class that creates new topological states from a map, when given a state,
 * location distribution, and visit. The state is the state of the previous map. The location distribution provides the
 * estimate of where the robot is now located, and the visit is the next area visited by the robot in the sequence
 * of areas.
 *
 * The generator is responsible for generating all consistent child hypotheses, given this input. The generator provides
 * access to these children one-by-one via the nextState method. Along with the children, the generator has a
 * probability that provides the probability estimate for the next child that will be generated.
 *
 * A completed generator has no remaining child hypotheses to create. Whenever the probability heuristics are updated,
 * the probability needs to be recomputed for the generator via the computeProbability method.
 */
class HypothesisGenerator
{
public:
    /**
     * Constructor for HypothesisGenerator.
     *
     * \pre depth >= 0
     * \param    state           Parent state from which to generate new hypotheses
     * \param    locations       Distribution of possible locations for the robot
     * \param    exitVisit       Visit for the exited area (corresponds to state->visitDepth).
     *                               Visit that corresponds to the parent state's creation.
     * \param    entryVisit      Newly entered visit for which new hypotheses are generated
     */
    HypothesisGenerator(const TopologicalState* state,
                        const GlobalLocationDistribution& locations,
                        const TopologicalVisit::Ptr& exitVisit,
                        const TopologicalVisit::Ptr& entryVisit);

    /**
     * Destructor for HypothesisGenerator.
     */
    virtual ~HypothesisGenerator(void);

    /**
     * nextState generates the next child state. Note that this both returns and destroys the child, rather than the
     * top/pop distinction in std::queue.
     *
     * \return   The next child state. If completed() == true, then an invalid state is returned.
     */
    TopologicalState nextState(void);

    /**
     * completed checks if all child states have been generated.
     */
    bool completed(void) const;

    /**
     * logProbability retrieves the probability of the next state to be generated. This probability should be used for
     * determining the location of the generator in the GeneratorQueue.
     */
    double logProbability(void) const;

    /**
     * computeProbability calculates the probability of this generator based on the probability of the parent state and
     * the current heuristics about the evolution of a map's probability.
     *
     * \param    heuristics          The new heuristics to apply to this generator
     */
    void computeProbability(const ProbabilityHeuristics& heuristics);

    /**
     * parent retrieves the pointer to the parent state from which children are being generated.
     */
    const TopologicalState* parent(void) const { return parent_; }

protected:
    /**
     * computeChildLogLikelihood computes the likelihood of a child hypotheses created by the generator. The default
     * behavior is to assign all children likelihoods to 1.0, meaning they take on the same value before evaluation as
     * the parent hypothesis.
     *
     * NOTE: This log-likelihood is just the predicted log-likelihood of the child hypothesis, not the full probability
     * as computed by the HypothesisProbabilityEvaluator. This log-likelihood provides heuristic information to change
     * the order in which hypotheses are generated in the tree.
     *
     * \param    parent          Parent state
     * \param    child           Child state
     * \param    visit           Visit that generated the child from the parent
     * \return   Probability of this child being the result of parent + visit.
     */
    virtual double computeChildLogLikelihood(const TopologicalState& parent,
                                             const TopologicalState& child,
                                             const TopologicalVisit& visit);

private:
    using Child = std::pair<double, TopologicalState>;

    // INVARIANT: children are sorted in order of ascending probability. nextState is .back()
    // NOTE: Children are lazily evaluated once a request for nextState() is made.
    int depth_ = 0;
    double heuristicPrior_ = 0.0;
    double heuristicLikelihood_ = 0.0;
    bool haveGeneratedChildren_ = false;   // flag indicating
    const TopologicalState* parent_;
    GlobalLocationDistribution locations_;
    TopologicalVisit::Ptr exitVisit_;
    TopologicalVisit::Ptr entryVisit_;
    std::vector<Child> children_;

    void generateChildren(void);
    void generateChildrenForLocation(const WeightedGlobalLocation& location);
    std::vector<TopologicalState> establishPossibleLoopClosures(const GlobalLocation& location);
    TopologicalState createNewAreaChild(const GlobalLocation& location);
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_HYPOTHESIS_GENERATOR_H

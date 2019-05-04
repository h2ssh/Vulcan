/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     probability_heursitics.h
* \author   Collin Johnson
* 
* Declaration of ProbabilityHeuristics.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_PROBABILITY_HEURISTICS_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_PROBABILITY_HEURISTICS_H

#include <vector>

namespace vulcan
{
namespace hssh 
{

/**
* ProbabilityHeuristics maintains the heuristics used for evaluating map hypotheses. The heuristics used during the
* computation are:
* 
*   - likelihood : for each depth, maintain the minimum change percentage from a parent to child hypothesis, which is
*       max(child_likelihood / parent_likelihood) for all child at the given depth
*   - prior : the maximum prior computed for a complete hypothesis
*/
class ProbabilityHeuristics
{
public:
    
    /**
    * updatePriorHeurstic udpates the prior heuristic. The prior is changed if newPrior > priorHeuristicAtDepth(depth).
    * 
    * \param    depth           Depth of this prior value
    * \param    newPrior        New prior value for a complete hypothesis
    * \return   True if the heuristic was updated.
    */
    bool updatePriorHeuristic(int depth, double newPrior);
    
    /**
    * updateLikelihoodHeuristic updates the likelihood heuristic for the given depth. The likelihood change is the ratio
    * of child likelihood (depth) over parent likelihood (depth - 1). The ratio is 1.0 for depth = 0 by definition.
    * 
    * \param    depth                   Depth of the hypothesis for this likelihood change
    * \param    logLikelihoodChange     Change in log-likelihood for hypothesis at depth from depth-1 parent
    * \return   True if the heuristic was updated.
    */
    bool updateLikelihoodHeuristic(int depth, double logLikelihoodChange);
    
    /**
    * priorHeurstic computes the prior heuristic for a hypothesis at the specified depth of the tree. The prior
    * heuristic is the maximum value of the prior amongst all priors computed at the maximum depth of the tree.
    *
    * \param    depth               Depth of the tree that the hypothesis exists at
    * \return   The computed prior if a heuristic value has been provided for maps at the specified depths. 1.0 if no
    *   heuristic has been computed yet for the desired depth.
    */
    double logPriorHeuristic(int depth) const;
    
    /**
    * likelihoodHeuristic computes the likelihood heuristic for a hypothesis at the provided depth. The 
    * heuristic is the product of likelihood changes from startDepth + 1 to endDepth
    * 
    * \param    depth               Depth of the tree that the hypothesis exists at
    * \return   The computed likelihood heuristic. The 
    */
    double logLikelihoodHeuristic(int depth) const;

private:

    double prior_ = 0.0;
    int priorDepth_ = 0;            // depth of the prior heuristic -- only the max depth matters
    std::vector<double> likelihood_;
    std::vector<bool> haveChange_;      // flags indicating if a likelihood change has been seen for a certain depth
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_PROBABILITY_HEURISTICS_H

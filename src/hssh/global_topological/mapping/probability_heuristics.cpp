/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     probability_heursitics.cpp
 * \author   Collin Johnson
 *
 * Definition of ProbabilityHeuristics.
 */

#include "hssh/global_topological/mapping/probability_heuristics.h"
#include <cmath>

namespace vulcan
{
namespace hssh
{

bool ProbabilityHeuristics::updatePriorHeuristic(int depth, double newPrior)
{
    bool changed = false;
    // If the depth increases, then the heuristic is always updated
    if (depth > priorDepth_) {
        prior_ = newPrior;
        priorDepth_ = depth;
        changed = true;
    }
    // Else if at the same depth, but this prior is larger, then reassign
    else if ((depth == priorDepth_) && (newPrior > prior_)) {
        prior_ = newPrior;
        changed = true;
    }
    // Otherwise at a lower depth, so it can't be heuristic or it is a lower prior value than the current best
    // so there isn't a change

    return changed;
}


bool ProbabilityHeuristics::updateLikelihoodHeuristic(int depth, double logLikelihoodChange)
{
    // Nothing needs to be done for the initial depth of the tree, just ignore it
    if (depth <= 0) {
        return true;
    }

    logLikelihoodChange =
      std::min(logLikelihoodChange, 0.0);   // the change can't increase. In general, there are
                                            // some weird cases where the optimizer gets stuck and then adding a lambda
                                            // gets it unstuck, but the likelihood is a generally monotonically
                                            // decreasing quantity, so it needs to be reflected in the heuristic

    // The likelihood is applied to parent->child relations and thus applies only to the edges, thus for a depth the
    // corresponding index is depth-1. There isn't a heuristic for the root, thus everything shifts by 1.
    if (depth > static_cast<int>(likelihood_.size())) {
        // The change between levels defaults to no change
        likelihood_.resize(depth, 0.0);
        haveChange_.resize(depth, false);
    }

    // If no change has been set, or if the likelihood change will increase, then the heuristic needs to be updated.
    if (!haveChange_[depth - 1] || (likelihood_[depth - 1] < logLikelihoodChange)) {
        likelihood_[depth - 1] = logLikelihoodChange;
        haveChange_[depth - 1] = true;
        return true;
    }

    return false;
}


double ProbabilityHeuristics::logPriorHeuristic(int depth) const
{
    return (depth > priorDepth_) ? 1.0 : prior_;
}


double ProbabilityHeuristics::logLikelihoodHeuristic(int depth) const
{
    double likelihoodChange = 0.0;
    depth = std::max(depth, 0);

    // Index in likelihood_ is depth-1, so depth+1,max iterates starting at index=depth.
    for (std::size_t n = depth; n < likelihood_.size(); ++n) {
        likelihoodChange += likelihood_[n];
    }

    return likelihoodChange;
}

}   // namespace hssh
}   // namespace vulcan

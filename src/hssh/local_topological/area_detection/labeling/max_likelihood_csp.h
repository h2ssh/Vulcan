/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     max_likelihood_csp.h
* \author   Collin Johnson
*
* Declaration of MaxLikelihoodCSP.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_MAX_LIKELIHOOD_CSP_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_MAX_LIKELIHOOD_CSP_H

#include "hssh/local_topological/area_detection/labeling/csp_solution.h"
#include "hssh/local_topological/area_detection/labeling/mcmc_sampling.h"
#include "hssh/local_topological/area_detection/labeling/csp_debug.h"

namespace vulcan
{
namespace system { class DebugCommunicator; }
namespace hssh
{

/**
* MaxLikelihoodCSP
*/
class MaxLikelihoodCSP
{
public:

    /**
    * Constructor for MaxLikelihoodCSP.
    */
    MaxLikelihoodCSP(const std::shared_ptr<SmallScaleStarBuilder>& starBuilder,
                     const MCMCSamplingParams& mcmcParams);

    /**
    * solve solves the currently defined CSP. The solution can then be applied to create the final version of the graph
    * to produce the current proposed areas.
    *
    * NOTE: The most recently exited area should not be in either the fixedAreas or the mutableAreas. The fixed areas
    * should include areas not connected to the subgraph of the robot's current area.
    *
    * \pre  All fixed areas must have a type assigned amongst {path, dest, decision}. Anything is invalid.
    *
    * \param    fixedAreas          Areas in the graph that have a fixed type
    * \param    mutableAreas        Areas in the graph that can potentially have their types changed
    * \param    exitedArea          The most recently exited area (can be null if no exited area exists)
    * \param    enteredArea         The most recently entered area (can be null if no entered area exists)
    * \param    boundaryClassifier  Classifier to use for computing distribution of on/off boundaries
    */
    CSPSolution solve(const std::vector<AreaHypothesis*>& fixedAreas,
                      const std::vector<AreaHypothesis*>& mutableAreas,
                      AreaHypothesis* exitedArea,
                      AreaHypothesis* enteredArea,
                      const BoundaryClassifier& boundaryClassifier);

    /**
    * sendDebug sends any debugging information generated during the solution of the CSP.
    */
    void sendDebug(system::DebugCommunicator& communicator);

private:

    const std::shared_ptr<SmallScaleStarBuilder> starBuilder_;
    const MCMCSamplingParams mcmcParams_;
    CSPDebugInfo debugInfo_;

    CSPSolution runMCMC(const std::vector<AreaHypothesis*>& fixedAreas,
                        const std::vector<AreaHypothesis*>& mutableAreas,
                        AreaHypothesis* exitedArea,
                        AreaHypothesis* enteredArea,
                        const BoundaryClassifier& boundaryClassifier,
                        bool doInitialMerge);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_MAX_LIKELIHOOD_CSP_H

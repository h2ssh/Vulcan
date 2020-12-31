/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     boundary.h
* \author   Collin Johnson
*
* Definition of AreaHypothesisBoundary.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREAS_BOUNDARY_H
#define HSSH_LOCAL_TOPOLOGICAL_AREAS_BOUNDARY_H

#include "hssh/local_topological/area_detection/labeling/hypothesis_type.h"
#include <array>
#include <memory>

namespace vulcan
{
namespace hssh
{

class AreaNode;
class AreaHypothesis;
class Gateway;
class AreaHypothesisBoundary;

using Hypotheses = std::array<AreaHypothesis*, 2>;

/**
* BoundaryMergeCondition defines the conditions underwhich a boundary is mergeable. When checking shouldMerge,
* one of these conditions should be provided to determine if the boundary can be merged.
*/
enum class BoundaryMergeCondition : unsigned char
{
    kInvalid,
    kFrontier,
    kSameType
};

/**
* AreaHypothesisBoundary represents the boundary between two AreaHypotheses. A boundary must be a GATEWAY_NODE.
* Each hypothesis must contain one of the nodes adjacent edges.
*
* The following are reasons that a boundary should be merged:
*
*   - The areas on both sides of the boundary are frontiers  (kFrontier)
*   - The areas on both sides of the boundary have the same type (only useful AFTER the final assignment is chosen)  (kSameType)
*   - One of the areas is invalid  (kInvalid)
*
* The boundary is created by providing a gateway node and the two adjacent hypotheses. During execution of
* the parsing phase, one of the hypotheses may be changed due to other merging that occurs. In this case,
* the changeArea() method should be called to replace the previous hypothesis with the updated representation.
*/
class AreaHypothesisBoundary
{
public:

    using HypothesisIter = Hypotheses::const_iterator;

    /**
    * Constructor for AreaHypothesisBoundary.
    *
    * \param    node        Gateway node for the boundary
    * \param    hypotheses   Hypotheses associated with the boundary
    */
    AreaHypothesisBoundary(AreaNode* node, const Hypotheses& hypotheses);

    // Observers
    /**
    * getNode retrieves the gateway node associated with the boundary.
    */
    const AreaNode* getNode(void) const { return node; }

    /**
    * getGateway retrieves the gateway that is the boundary between the two hypotheses.
    */
    const Gateway& getGateway(void) const;

    /**
    * getHypotheses retrieves the hypotheses on either side of the boundary.
    */
    Hypotheses getHypotheses(void) const { return hypotheses; }

    /**
    * getOtherHypothesis retrieves the hypothesis on the other side of the boundary from the provided hypothesis.
    * If the provided hypothesis isn't associated with the boundary, then a null hypothesis pointer is returned.
    *
    * getOtherHypothesis uses the isSimilar method to compare the hypotheses. Using this method allows a superset or subset
    * of a hypothesis to still be considered. This is particularly useful when considering if a boundary is valid.
    *
    * \param    hypothesis        Hypothesis on one side of the boundary
    * \return   Return hypothesis on the other side of the boundary. NULL if hypothesis is not associated with the boundary.
    */
    AreaHypothesis* getOtherHypothesis(const AreaHypothesis& hypothesis) const;

    // Iterators for the hypotheses associated with the boundary
    HypothesisIter beginHypotheses(void) const { return hypotheses.begin(); }
    HypothesisIter endHypotheses(void)   const { return hypotheses.end(); }

    /**
    * getBoundaryType retrieves the type of the boundary for the provided hypothesis, which must be one of the two
    * hypotheses associated with this area.
    *
    * For places, the boundary type is the same as the hypothesis type. For paths, the boundary type will be either
    * endpoint or dest, depending on the whether or not this boundary is an endpoint for the hypothesis.
    *
    * \param    hypothesis      Hypothesis to find type for
    * \return   Boundary type for the hypothesis. HypothesisType::kArea if the hypothesis isn't associated with this
    *   boundary.
    */
    HypothesisType getBoundaryType(const AreaHypothesis* hypothesis) const;

    /**
    * length retrieves the length of the boundary.
    */
    double length(void) const;

    /**
    * probability retrieves the probability of this boundary being set to the active state.
    */
    double probability(void) const { return prob_; }
    double logProb(void) const { return logProb_; }
    double logProbNot(void) const { return logProbNot_; }

    /**
    * setProbability sets the probability of this boundary being active.
    */
    void setProbability(double probability);

    /**
    * containsHypothesis checks to see if the provided hypothesis is a part of the boundary.
    *
    * \param    hypothesis        Hypothesis to check
    * \return   True if the hypothesis is either of the associated hypotheses.
    */
    bool containsHypothesis(const AreaHypothesis* hypothesis) const;

    // Operations
    /**
    * shouldMerge checks to see if the boundary is valid based on the current state of its associated hypotheses. The validity
    * check uses the conditions detailed in the class description. The validty of a boundary can change if the classification
    * of the hypotheses changes, thus the result of shouldMerge might be different between invocations.
    *
    * \param    condition           Condition to use for
    * \return   True if the boundary satisifies all of the necessary rules.
    */
    bool shouldMerge(BoundaryMergeCondition condition) const;

private:

    friend class HypothesisGraph;       // HypothesisGraph owns these pointers and can access their internal connections
                                        // to allow the public interface to be immutable

    AreaNode*  node;
    Hypotheses hypotheses;
    double     prob_ = 1.0;         // probability this boundary should exist
    double     logProb_ = 0.0;      // pre-cached log-probability
    double     logProbNot_ = -1000; // pre-cache log(1.0 - prob_)

    // Merging rules
    bool isSameType(void) const;
    bool isFrontier(void) const;
    bool isInvalid(void)  const;

    // Helpers

    /**
    * changeArea changes the hypothesis associated with a boundary from an old graph to a new graph. Changing the area
    * associated with a boundary is needed because the merge operation for hypotheses can cause a new hypothesis to be
    * associated with a particular boundary. Since boundaries are also contained in hypotheses, they would either need
    * to be replaced in the hypothesis, or the hypothesis can be swapped here. Making the local change is easier and the
    * validity can be confirmed by looking at the hypothesis, so all is well.
    *
    * \param    oldHypothesis       Previous hypothesis associated with the area
    * \param    newHypothesis       New hypothesis to associated with the area
    * \return   True if the graph was replaced. False if oldGraph wasn't associated with this boundary.
    */
    bool changeArea(AreaHypothesis* oldHypothesis, AreaHypothesis* newHypothesis);
};
}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_AREAS_BOUNDARY_H

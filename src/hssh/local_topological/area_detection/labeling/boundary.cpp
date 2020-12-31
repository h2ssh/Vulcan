/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     boundary.cpp
* \author   Collin Johnson
*
* Implementation of AreaHypothesisBoundary.
*/

#include "hssh/local_topological/area_detection/labeling/boundary.h"
#include "hssh/local_topological/area_detection/labeling/area_graph.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis.h"
#include <cassert>
#include <iostream>
#include <memory>

// #define DEBUG_MERGE_RULES
#define WARNING_OTHER_SUBGRAPH

namespace vulcan
{
namespace hssh
{

inline bool are_hypotheses_of_types(const Hypotheses& graphs, const HypothesisType first, const HypothesisType second)
{
    return ((graphs[0]->getType() == first) && (graphs[1]->getType() == second)) ||
            ((graphs[1]->getType() == first) && (graphs[0]->getType() == second));
}


AreaHypothesisBoundary::AreaHypothesisBoundary(AreaNode* node, const Hypotheses& hypotheses)
: node(node)
, hypotheses(hypotheses)
{
    assert(node->getType() & AreaNode::kGateway);
    assert(hypotheses[0]);
    assert(hypotheses[1]);
}


const Gateway& AreaHypothesisBoundary::getGateway(void) const
{
    return node->getGateway();
}


AreaHypothesis* AreaHypothesisBoundary::getOtherHypothesis(const AreaHypothesis& hypothesis) const
{
    if(hypothesis.isSimilar(*hypotheses[0]))
    {
        return hypotheses[1];
    }
    else if(hypothesis.isSimilar(*hypotheses[1]))
    {
        return hypotheses[0];
    }

//     if(hypotheses[0] == &hypothesis)
//     {
//         return hypotheses[1];
//     }
//     else
//     {
//         assert(hypotheses[1] == &hypothesis);
//         return hypotheses[0];
//     }

#ifdef WARNING_OTHER_SUBGRAPH
    std::cerr<<"WARNING:AreaHypothesisBoundary: getOtherHypothesis called with hypothesis parameter that was neither of the associated hypotheses.\n";
    assert(false);
#endif

    return 0;
}


HypothesisType AreaHypothesisBoundary::getBoundaryType(const AreaHypothesis* hypothesis) const
{
    if(!containsHypothesis(hypothesis))
    {
        return HypothesisType::kArea;
    }

    // Paths are split into destination vs endpoint
    if(hypothesis->getType() == HypothesisType::kPath)
    {
        return hypothesis->isEndGateway(getGateway().id()) ? HypothesisType::kPathEndpoint : HypothesisType::kPathDest;
    }
    // All other types remain unchanged
    else
    {
        return hypothesis->getType();
    }
}


double AreaHypothesisBoundary::length(void) const
{
    return getGateway().length();
}


void AreaHypothesisBoundary::setProbability(double probability)
{
    // The probability can never be 0 or 1 as a boundary might always be able to fiddle
    prob_ = std::max(probability, 1e-3);
    prob_ = std::min(probability, 1.0 - 1e-3);

    logProb_ = std::log(prob_);
    logProbNot_ = std::log(1.0 - prob_);
    node->setProbability(prob_);
}


bool AreaHypothesisBoundary::containsHypothesis(const AreaHypothesis* hypothesis) const
{
    return hypotheses[0]->isSimilar(*hypothesis) || hypotheses[1]->isSimilar(*hypothesis);
}


bool AreaHypothesisBoundary::shouldMerge(BoundaryMergeCondition condition) const
{
    switch(condition)
    {
    case BoundaryMergeCondition::kSameType:
        return isSameType();

    case BoundaryMergeCondition::kFrontier:
        return isFrontier();

    case BoundaryMergeCondition::kInvalid:
        return isInvalid();

    default:
        std::cerr<<"ERROR::AreaHypothesisBoundary: Invalid merge condition:"<<static_cast<int>(condition)<<'\n';
    }

    return false;
}


bool AreaHypothesisBoundary::isSameType(void) const
{
    return (hypotheses[0]->getType() == hypotheses[1]->getType()) && (hypotheses[0]->getType() != HypothesisType::kArea);
}


bool AreaHypothesisBoundary::isFrontier(void) const
{
    return hypotheses[0]->isFrontier() && hypotheses[1]->isFrontier();
}


bool AreaHypothesisBoundary::isInvalid(void) const
{
    return !hypotheses[0]->isValid() || !hypotheses[1]->isValid();
}


bool AreaHypothesisBoundary::changeArea(AreaHypothesis* oldHypothesis, AreaHypothesis* newHypothesis)
{
    bool foundHyp = false;

    if(oldHypothesis == hypotheses[0])
    {
        hypotheses[0] = newHypothesis;
        foundHyp   = true;
    }
    else if(oldHypothesis == hypotheses[1])
    {
        hypotheses[1] = newHypothesis;
        foundHyp   = true;
    }

    return foundHyp;
}

} // namespace hssh
} // namespace vulcan

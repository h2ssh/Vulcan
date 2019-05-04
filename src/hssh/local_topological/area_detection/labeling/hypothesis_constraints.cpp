/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_constraints.cpp
* \author   Collin Johnson
*
* Definition of functions to check constraints for an AreaHypothesis in relation to its neighbors.
*/

#include <hssh/local_topological/area_detection/labeling/hypothesis_constraints.h>
#include <hssh/local_topological/area_detection/labeling/hypothesis.h>
#include <hssh/local_topological/area_extent.h>

namespace vulcan
{
namespace hssh
{

bool satisfies_all_constraints(const AreaHypothesis* hypothesis)
{
    return satisfies_path_alignment_constraint(hypothesis)
        && satisifies_through_path_constraint(hypothesis)
        && satisfies_path_destination_constraint(hypothesis)
        && satisfies_decision_constraint(hypothesis)
        && satisfies_complete_path_graph_constraint(hypothesis);
}


bool satisfies_path_alignment_constraint(const AreaHypothesis* hypothesis)
{
    if(hypothesis->getType() == HypothesisType::kPath)
    {
        int numDecisions = hypothesis->countGatewaysWithMask(HypothesisType::kDecision);
//         int numDecisionEndpoints = hypothesis->numEndpointsWithType(HypothesisType::kDecision);

        int numFrontierEnds = hypothesis->numNonGatewayEnds();

        return numDecisions > 0 && numFrontierEnds + numDecisions <= 2;
    }
    else
    {
        return true;
    }
}


bool satisifies_through_path_constraint(const AreaHypothesis* hypothesis)
{
    // If the area is labeled as a destination, then it cannot serve to connect the end of a path or a decision point
    // to another path or decision point.
    if(hypothesis->getType() == HypothesisType::kDest)
    {
        int numEnds = hypothesis->countGatewaysWithMask(HypothesisType::kPathEndpoint | HypothesisType::kDecision);
        int numDests = hypothesis->countGatewaysWithMask(HypothesisType::kPathDest);

        return (numEnds == 0 && numDests > 0) || (numEnds == 1 && numDests == 0);
    }
    // If not a destination, then the constraint doesn't apply
    else
    {
        return true;
    }
}


bool satisfies_path_destination_constraint(const AreaHypothesis* hypothesis)
{
    // If there is a path destination relation, then this must be a destination. Otherwise, the constraint doesn't
    // apply
    if(hypothesis->countGatewaysWithMask(HypothesisType::kPathDest) > 0)
    {
        return hypothesis->getType() == HypothesisType::kDest;
    }
    else
    {
        return true;
    }
}


bool satisfies_decision_constraint(const AreaHypothesis* hypothesis)
{
    auto star = hypothesis->calculateStar(HypothesisType::kPathEndpoint | HypothesisType::kDecision);
    
    // If there are intersecting paths and decision points, then it must be a decision point
    if((star.getNumPaths() > 1) && (hypothesis->countGatewaysWithMask(HypothesisType::kPathEndpoint) > 0))
    {
        return hypothesis->getType() == HypothesisType::kDecision;
    }
    // Otherwise, it can't be a decision point
    else
    {
        return hypothesis->getType() != HypothesisType::kDecision;
    }
}


bool satisfies_complete_path_graph_constraint(const AreaHypothesis* hypothesis)
{
    // TODO: Determine the DFS for performing this action
    return true;
}

} // namespace hssh
} // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     hypothesis_constraints.h
 * \author   Collin Johnson
 *
 * Declaration of functions to check constraints for an AreaHypothesis in relation to its neighbors:
 *
 *   - satisfies_all_constraints : check if all constraints are satisfied
 *   - satisfies_path_alignment_constraint : one or two gateways aligned to the axis
 *   - satisifies_through_path_constraint : destinations can't have more than one path endpoint/decision point adjacent
 *   - satisfies_path_destination_constraint : areas adjacent to path not at endpoints must be destinations
 *   - satisfies_decision_constraint : intersecting paths or decisions must be a decision point
 *   - satisfies_complete_path_graph_constraint : paths + decision points must form a complete graph
 *
 * All constraints take a hypothesis to check + adjacent hypotheses.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_CONSTRAINTS_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_CONSTRAINTS_H

#include <vector>

namespace vulcan
{
namespace hssh
{

class AreaHypothesis;


bool satisfies_all_constraints(const AreaHypothesis* hypothesis);

bool satisfies_path_alignment_constraint(const AreaHypothesis* hypothesis);

bool satisifies_through_path_constraint(const AreaHypothesis* hypothesis);

bool satisfies_path_destination_constraint(const AreaHypothesis* hypothesis);

/**
 * satisifies_decision_constraint is true if:
 *
 *   - The hypothesis has two or more intersecting paths and decision points
 *   - At least one incident path
 */
bool satisfies_decision_constraint(const AreaHypothesis* hypothesis);

bool satisfies_complete_path_graph_constraint(const AreaHypothesis* hypothesis);

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_CONSTRAINTS_H

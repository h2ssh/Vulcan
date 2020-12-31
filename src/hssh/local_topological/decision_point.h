/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef HSSH_LOCAL_TOPOLOGICAL_DECISION_POINT_H
#define HSSH_LOCAL_TOPOLOGICAL_DECISION_POINT_H

#include <boost/shared_ptr.hpp>
#include <set>

namespace vulcan
{
namespace hssh
{

class GatewayHypothesisNode;

/**
 * decision_point_t represents a decision point amongst gateways. A decision point is an
 * intersection of multiple gateways in the world. These gateways bound a region where
 * multiple paths intersect. Such regions are places.
 *
 * A decision point contains two additional pieces of information, an appropriateness measure and
 * the number of intersections between gateway normals that occurs at the gateway. These
 * values provide a means of determining how good a description of space the decision point is.
 */
struct decision_point_t
{
    decision_point_t(void) : appropriateness(0.0f), numIntersections(0) { }

    std::set<boost::shared_ptr<GatewayHypothesisNode>> gateways;

    float appropriateness;
    int numIntersections;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_DECISION_POINT_H

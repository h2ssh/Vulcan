/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     affordance_visitor.h
 * \author   Collin Johnson
 *
 * Declaration of NavigationAffordanceVisitor interface.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AFFORDANCE_VISITOR_H
#define HSSH_LOCAL_TOPOLOGICAL_AFFORDANCE_VISITOR_H

namespace vulcan
{
namespace hssh
{

class ExplorationAffordance;
class MoveAlongAffordance;
class TransitionAffordance;

/**
 * NavigationAffordanceVisitor is the interface for visitors handling classes in the NavigationAffordance hierarchy.
 */
class NavigationAffordanceVisitor
{
public:
    virtual ~NavigationAffordanceVisitor(void) { }

    // Visitation methods called by classes of the corresponding types
    virtual void visitExploration(const ExplorationAffordance& affordance) = 0;
    virtual void visitMoveAlong(const MoveAlongAffordance& affordance) = 0;
    virtual void visitTransition(const TransitionAffordance& affordance) = 0;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_AFFORDANCE_VISITOR_H

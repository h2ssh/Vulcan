/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     area_visitor.h
 * \author   Collin Johnson
 *
 * Declaration of LocalAreaVisitor interface.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_VISITOR_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_VISITOR_H

namespace vulcan
{
namespace hssh
{

class LocalDestination;
class LocalDecisionPoint;
class LocalPathSegment;

/**
 * LocalAreaVisitor is the Visitor interface for the LocalArea class hierarchy. There
 * is a virtual visitXXXX method for each type in the hierarchy. They are declared pure virtual
 * to ensure all are subclassed by an implementation of the interface.
 */
class LocalAreaVisitor
{
public:
    virtual ~LocalAreaVisitor(void) { }

    virtual void visitDestination(const LocalDestination& destination) = 0;
    virtual void visitDecisionPoint(const LocalDecisionPoint& decision) = 0;
    virtual void visitPathSegment(const LocalPathSegment& path) = 0;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_AREA_VISITOR_H

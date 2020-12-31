/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     dynamic_object_visitor.h
 * \author   Collin Johnson
 *
 * Definition of DynamicObjectVisitor interface.
 */

#ifndef TRACKER_DYNAMIC_OBJECT_VISITOR_H
#define TRACKER_DYNAMIC_OBJECT_VISITOR_H

namespace vulcan
{
namespace tracker
{

class Person;
class RigidObject;
class UnclassifiedObject;
class PivotingObject;
class SlidingObject;

/**
 * DynamicObjectVisitor is a visitor for the DynamicObject class hierarchy. The methods are virtual to enforces
 * implementors to handle all objects in the event a new object type is added to the tracker.
 */
class DynamicObjectVisitor
{
public:
    virtual void visitPerson(const Person& person) = 0;
    virtual void visitRigid(const RigidObject& object) = 0;
    virtual void visitUnclassified(const UnclassifiedObject& object) = 0;
    virtual void visitPivotingObject(const PivotingObject& door) = 0;
    virtual void visitSlidingObject(const SlidingObject& door) = 0;
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_DYNAMIC_OBJECT_VISITOR_H

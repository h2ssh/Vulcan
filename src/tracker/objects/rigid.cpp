/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     rigid.cpp
* \author   Collin Johnson
* 
* Definition of RigidObject.
*/

#include <tracker/objects/rigid.h>
#include <tracker/dynamic_object_visitor.h>

namespace vulcan
{
namespace tracker 
{

RigidObject::RigidObject(ObjectId id, int64_t timestamp, const SteadyMotion& motion, const ObjectBoundary& boundary)
: BaseType(timestamp, motion, boundary)
, id_(id)
{
}


std::unique_ptr<DynamicObject> RigidObject::clone(void) const
{
    return std::unique_ptr<DynamicObject>(new RigidObject(*this));
}


void RigidObject::accept(DynamicObjectVisitor& visitor) const
{
    visitor.visitRigid(*this);
}

}
}

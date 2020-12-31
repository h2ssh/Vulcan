/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     object_factory.cpp
 * \author   Collin Johnson
 *
 * Definition of DynamicObjectFactory.
 */

#include "tracker/objects/object_factory.h"
#include "tracker/motions/fixed_endpoint.h"
#include "tracker/object_motion.h"
#include "tracker/objects/person.h"
#include "tracker/objects/pivoting_object.h"
#include "tracker/objects/rigid.h"
#include "tracker/objects/sliding_object.h"
#include "tracker/objects/unclassified.h"
#include <iostream>

namespace vulcan
{
namespace tracker
{

DynamicObjectFactory::~DynamicObjectFactory(void)
{
    // For std::unique_ptr
}


std::unique_ptr<DynamicObject> DynamicObjectFactory::createDynamicObject(ObjectId id,
                                                                         int64_t timestamp,
                                                                         const ObjectMotion& motion,
                                                                         const ObjectBoundary& boundary)
{
    id_ = id;
    timestamp_ = timestamp;
    boundary_ = &boundary;
    motion.accept(*this);
    return std::move(createdObject_);
}


void DynamicObjectFactory::visitFixedEndpoint(const FixedEndpointMotion& motion)
{
    if (motion.isPivoting()) {
        createdObject_.reset(new PivotingObject(id_, timestamp_, motion.model()));
    } else if (motion.isSliding()) {
        createdObject_.reset(new SlidingObject(id_, timestamp_, motion.model()));
    } else {
        std::cerr << "WARNING: DynamicObjectFactory: Attempted to create FixedObject with an invalid type of motion."
                  << " The motion was not sliding or pivoting. Creating an unclassified object instead.\n";

        StationaryMotion stationary(motion.model().position());
        visitStationary(stationary);
    }
}


void DynamicObjectFactory::visitSteady(const SteadyMotion& motion)
{
    createdObject_.reset(new RigidObject(id_, timestamp_, motion, *boundary_));
}


void DynamicObjectFactory::visitStriding(const StridingMotion& motion)
{
    createdObject_.reset(new Person(id_, timestamp_, motion, *boundary_));
}


void DynamicObjectFactory::visitStationary(const StationaryMotion& motion)
{
    createdObject_.reset(new UnclassifiedObject(id_, timestamp_, motion, *boundary_));
}

}   // namespace tracker
}   // namespace vulcan

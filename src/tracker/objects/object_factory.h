/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object_factory.h
* \author   Collin Johnson
* 
* Declaration of DynamicObjectFactory.
*/

#ifndef TRACKER_OBJECTS_OBJECT_FACTORY_H
#define TRACKER_OBJECTS_OBJECT_FACTORY_H

#include "tracker/types.h"
#include "tracker/motions/visitor.h"
#include <memory>

namespace vulcan
{
namespace tracker 
{
    
class DynamicObject;
class ObjectBoundary;
class ObjectMotion;

/**
* DynamicObjectFactory creates DynamicObjects based on the type of motion it has.
*/
class DynamicObjectFactory : public ObjectMotionVisitor
{
public:

    virtual ~DynamicObjectFactory(void);
    
    /**
    * createDynamicObject creates a DynamicObject of the appropriate type based on its motion and boundary.
    * 
    * \param    id          Unique id to assign to the object
    * \param    timestamp   Time at which the object was created
    * \param    motion      Detected motion for the object 
    * \param    boundary    Boundary of the object
    */
    std::unique_ptr<DynamicObject> createDynamicObject(ObjectId              id,
                                                       int64_t               timestamp,
                                                       const ObjectMotion&   motion, 
                                                       const ObjectBoundary& boundary);
    
private:
    
    ObjectId id_;
    int64_t timestamp_;
    const ObjectBoundary* boundary_;
    std::unique_ptr<DynamicObject> createdObject_;
    
    // ObjectMotionVisitor interface
    void visitFixedEndpoint(const FixedEndpointMotion& motion) override;
    void visitSteady(const SteadyMotion& motion) override;
    void visitStriding(const StridingMotion& motion) override;
    void visitStationary(const StationaryMotion& motion) override;
};

}
}

#endif // TRACKER_OBJECTS_OBJECT_FACTORY_H

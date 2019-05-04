/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     rigid.h
* \author   Collin Johnson
* 
* Declaration of RigidObject.
*/

#ifndef TRACKER_OBJECTS_RIGID_H
#define TRACKER_OBJECTS_RIGID_H

#include <tracker/objects/bounded_moving.h>
#include <tracker/motions/steady.h>
#include <cereal/access.hpp>

namespace vulcan
{
namespace tracker 
{

/**
* RigidObject
*/
class RigidObject : public BoundedMovingObject<SteadyMotion>
{
public:
    
    /**
    * Constructor for RigidObject.
    */
    RigidObject(ObjectId id, int64_t timestamp, const SteadyMotion& motion, const ObjectBoundary& boundary);

    // Expose state of current tracking status
    MultivariateGaussian slowMotionState(void) const { return motion_.slowMotionState(); }
    MultivariateGaussian fastMotionState(void) const { return motion_.fastMotionState(); }
    
    // DynamicObject interface
    ObjectId id(void) const override { return id_; }
    std::unique_ptr<DynamicObject> clone(void) const override;
    void accept(DynamicObjectVisitor& visitor) const override;
    
private:
    
    using BaseType = BoundedMovingObject<SteadyMotion>;

    ObjectId id_;
    
    // Serialization support
    friend class cereal::access;

    RigidObject(void) { }
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( cereal::base_class<BaseType>(this),
            id_
        );
    }
};

}
}

// Smart pointer serialization support
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::tracker::RigidObject)

#endif // TRACKER_OBJECTS_RIGID_H

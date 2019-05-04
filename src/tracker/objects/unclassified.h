/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     unclassified.h
* \author   Collin Johnson
* 
* Declaration of UnclassifiedObject.
*/

#ifndef TRACKER_OBJECTS_UNCLASSIFIED_H
#define TRACKER_OBJECTS_UNCLASSIFIED_H

#include <tracker/objects/bounded_moving.h>
#include <tracker/motions/stationary.h>
#include <cereal/access.hpp>
#include <cereal/types/base_class.hpp>

namespace vulcan
{
namespace tracker
{

/**
* UnclassifiedObject is an object that has yet to be classified as one of the other DynamicObject types. Consequently,
* an unclassified object represents a dynamic blob of cells in the map. The blob isn't tracked, so it has no velocity.
* The unclassified object ensures that all dynamic rays in a laser scan are represented by some sort of DynamicObject.
* 
* An UnclassifiedObject satisifies the requirement that all dynamic laser rays be represented by some DynamicObject.
*
* UnclassifiedObjects never match other laser objects. They are intended to be create each iteration by a
* LaserObjectClassifier.
*/
class UnclassifiedObject : public BoundedMovingObject<StationaryMotion>
{
public:
    
    /**
    * Constructor for UnclassifiedObject.
    */
    UnclassifiedObject(ObjectId id, int64_t timestamp, const StationaryMotion& motion, const ObjectBoundary& boundary);
    
    // DynamicObject interface
    ObjectId id(void) const override { return id_; }
    std::unique_ptr<DynamicObject> clone(void) const override;
    void accept(DynamicObjectVisitor& visitor) const override;
    
private:
    
    using BaseType = BoundedMovingObject<StationaryMotion>;

    ObjectId id_;
    
    // Serialization support
    friend class cereal::access;

    UnclassifiedObject(void) { }
    
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

CEREAL_REGISTER_TYPE(vulcan::tracker::UnclassifiedObject)

#endif // TRACKER_OBJECTS_UNCLASSIFIED_H

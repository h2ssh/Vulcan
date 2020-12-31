/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     sliding_object.h
* \author   Collin Johnson
* 
* Declaration of SlidingObject.
*/

#ifndef TRACKER_OBJECTS_SLIDING_DOOR_H
#define TRACKER_OBJECTS_SLIDING_DOOR_H

#include "tracker/objects/fixed_object.h"

namespace vulcan
{
namespace tracker 
{
    
class BoundedSlidingMotion;

/**
* SlidingObject is a FixedObject that emerges from a fixed point and slides for a bit before going back. Sliding doors
* and elevators doors are the most common instance of a SlidingObject. The SlidingObject has a small range through
* which it moves, unlike a RigidObject which can move freely in the environment.
* 
* A SlidingObject has the following properties in addition to the fixedPosition of FixedObject:
* 
*   - length    : length of the sliding
*   - direction : direction the object slides
*/
class SlidingObject : public FixedObject
{
public:
    
    /**
    * Constructor for SlidingObject.
    */
    SlidingObject(ObjectId id, int64_t timestamp, const FixedObjectModel& model);
    
    /**
    * length retrieves the maximum length the object slides.
    */
    float length(void) const;
    
    /**
    * direction retrieves the direction the object slides from the fixedPosition.
    */
    float direction(void) const;
    
    // DynamicObject interface
    ObjectId id(void) const override { return id_; }
    std::unique_ptr<DynamicObject> clone(void) const override;
    void accept(DynamicObjectVisitor& visitor) const override;
    
private:

    ObjectId id_;
    
    // FixedObject interface
    velocity_t estimateVelocity(int64_t timestamp) const override;
    
    // Serialization support
    SlidingObject(void) { }
    
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( cereal::base_class<FixedObject>(this),
            id_
        );
    }
};

}
}

// Smart pointer serialization support
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::tracker::SlidingObject)

#endif // TRACKER_OBJECTS_SLIDING_DOOR_H

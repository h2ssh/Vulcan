/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     pivoting_object.h
 * \author   Collin Johnson
 *
 * Declaration of PivotingObject.
 */

#ifndef TRACKER_OBJECTS_PIVOTING_DOOR_H
#define TRACKER_OBJECTS_PIVOTING_DOOR_H

#include "math/angle_range.h"
#include "tracker/objects/fixed_object.h"

namespace vulcan
{
namespace tracker
{

/**
 * PivotingObject is a FixedObject that is rotating about some fixed position. The most common pivoting object is a
 * door.
 *
 * A PivotingObject has the following properties in addition to the fixedPosition of FixedObject:
 *
 *   - radius        : estimated radius of the object
 *   - angleRange    : range of angles through which the object swing
 */
class PivotingObject : public FixedObject
{
public:
    /**
     * Constructor for PivotingObject.
     */
    PivotingObject(ObjectId id, int64_t timestamp, const FixedObjectModel& model);

    /**
     * arc retrieves the arc swept out by the PivotingObject.
     */
    Arc arc(void) const;

    // DynamicObject interface
    ObjectId id(void) const override { return id_; }
    std::unique_ptr<DynamicObject> clone(void) const override;
    void accept(DynamicObjectVisitor& visitor) const override;

private:
    ObjectId id_;

    // FixedObject interface
    velocity_t estimateVelocity(int64_t timestamp) const override;

    // Serialization support
    PivotingObject(void){};

    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::base_class<FixedObject>(this), id_);
    }
};

}   // namespace tracker
}   // namespace vulcan

// Smart pointer serialization support
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::tracker::PivotingObject)

#endif   // TRACKER_OBJECTS_PIVOTING_DOOR_H

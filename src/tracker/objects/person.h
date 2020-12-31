/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     person.h
 * \author   Collin Johnson
 *
 * Declaration of Person.
 */

#ifndef TRACKER_OBJECTS_PERSON_H
#define TRACKER_OBJECTS_PERSON_H

#include "tracker/boundaries/circle.h"
#include "tracker/motions/striding.h"
#include "tracker/objects/bounded_moving.h"
#include <cereal/access.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/base_class.hpp>

namespace vulcan
{
namespace tracker
{

/**
 * Person represents a person. A Person contains two LegModels that are used for estimating the motion of the
 * object through the environment.
 */
class Person : public BoundedMovingObject<StridingMotion>
{
public:
    //     using Legs = std::array<LegModel, 2>;       // Every person has two Legs, no we can't track cats.

    /**
     * Constructor for Person.
     *
     * Create a Person where only one leg initially is found.
     *
     * \param    id          Id assigned to the object
     * \param    leg         Observed leg for the person
     */
    Person(ObjectId id, int64_t timestamp, const StridingMotion& motion, const ObjectBoundary& boundary);

    // DynamicObject interface
    ObjectId id(void) const override { return id_; }
    void updateModel(const LaserObject& object) override;
    std::unique_ptr<DynamicObject> clone(void) const override;
    void accept(DynamicObjectVisitor& visitor) const override;

private:
    using BaseType = BoundedMovingObject<StridingMotion>;

    ObjectId id_;
    std::vector<CircleBoundary> legModels_;
    std::vector<Position> lastLegPositions_;


    bool canSeeTwoLegs(const LaserObject& object) const;
    void updateSingleLeg(Arc measuredLeg);
    void updateTwoLegs(TwoArcs measuredLegs);

    void createBoundaryForLegs(void);

    // Serialization support
    friend class cereal::access;

    Person(void) { }

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::base_class<BaseType>(this), id_);
    }
};

}   // namespace tracker
}   // namespace vulcan

// Smart pointer serialization support
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::tracker::Person)

#endif   // TRACKER_OBJECTS_PERSON_H

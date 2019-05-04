/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     fixed_object.h
* \author   Collin Johnson
* 
* Declaration of FixedObject.
*/

#ifndef TRACKER_OBJECTS_FIXED_OBJECT_H
#define TRACKER_OBJECTS_FIXED_OBJECT_H

#include <tracker/dynamic_object.h>
#include <tracker/object_state.h>
#include <tracker/objects/fixed_object_model.h>
#include <cereal/access.hpp>
#include <cereal/types/deque.hpp>

namespace vulcan
{
namespace tracker 
{

/**
* FixedObject is an abstract base class representing an object in the environment that moves around some fixed position.
* Such fixed objects are typically door, either sliding or rotating, though other such objects surely exist.
*
* FixedObject implements most of the DynamicObject interface. The remaining methods that must be implemented by
* subclasses are:
*
*   - estimateStateAt
*   - clone
*   - accept
*/
class FixedObject : public DynamicObject
{
public:
    
    /**
    * fixedPosition is the position on the object that is fixed to the environment.
    */
    Position fixedPosition(void) const { return model_.fixedPosition(); }

    // DynamicObject interface
    void updateModel(const LaserObject& object) override;
    void setGoals(const ObjectGoalDistribution& goals) override;
    float overlapWithObject(const LaserObject& object) const override;
    Position position(void) const override;
    velocity_t velocity(void) const override;
    object_motion_state_t motionState(void) const override;
    ObjectBoundary boundary(void) const override;
    double radius(void) const override;
    ObjectGoalDistribution goals(void) const { return goals_; }
    int64_t timeLastSeen(void) const override { return lastUpdateTime_; }
    int64_t totalTimeSeen(void) const override { return totalTimeSeen_; }


protected:
    
    /**
    * Default constructor for FixedObject.
    */
    FixedObject(void) { }
    
    /**
    * Constructor for FixedObject.
    * 
    * \param    timestamp           Time at which the model was last updated
    * \param    model               Current model of the object
    */
    FixedObject(int64_t timestamp, const FixedObjectModel& model);

    /**
    * model retrieves the underlying model of the object to be used by subclasses for knowing the current and future
    * state of the object.
    */
    const FixedObjectModel& model(void) const { return model_; }
    
    /**
    * estimateVelocity estimates the current velocity of the object. When called, the model() has already been updated
    * with the most recent information from a LaserObject. Thus, the model() can be used for determining the object
    * position at the current timestamp.
    * 
    * \param    timestamp           Current timestamp of the model
    * \return   The current velocity estimate for the object.
    */
    virtual velocity_t estimateVelocity(int64_t timestamp) const = 0;

private:

    int64_t          lastUpdateTime_;
    int64_t          totalTimeSeen_;
    FixedObjectModel model_;
    object_state_t   state_;
    ObjectGoalDistribution goals_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( lastUpdateTime_,
            totalTimeSeen_,
            model_,
            goals_);
    }
};

}
}

#endif // TRACKER_OBJECTS_FIXED_OBJECT_H

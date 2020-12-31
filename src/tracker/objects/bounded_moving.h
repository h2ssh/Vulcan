/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     bounded_moving_object.h 
* \author   Collin Johnson
* 
* Definition of BoundedMovingObject abstract base class for DynamicObjects that estimate their state using an
* ObjectBoundary and ObjectMotion subclass.
*/

#ifndef TRACKER_OBJECTS_BOUNDED_MOVING_OBJECT_H
#define TRACKER_OBJECTS_BOUNDED_MOVING_OBJECT_H

#include "tracker/dynamic_object.h"
#include "tracker/laser_object.h"
#include "tracker/object_state.h"
#include <cereal/types/deque.hpp>

namespace vulcan
{
namespace tracker 
{

/**
* BoundedMovingObject is a mixin class for DynamicObject instances that estimate their motion using a subclass of
* ObjectMotion and their boundary with an ObjectBoundary.
* 
* For such classes, subclass BoundedMovingObject with the appropriate ObjectMotion subclass.
* 
* The unimplemented methods are:
* 
*   - clone
*   - accept
*/
template <class MotionType>
class BoundedMovingObject : public DynamicObject
{
public:
    
    /**
    * Constructor for BoundedMovingObject.
    */
    BoundedMovingObject(int64_t timestamp, const MotionType& motion, const ObjectBoundary& boundary)
    : timestamp_(timestamp)
    , totalTime_(0)
    , motion_(motion)
    , boundary_(boundary)
    {
    }
    
    // DynamicObject interface
    void updateModel(const LaserObject& object) override
    {
        motion_.updateModel(object);
        boundary_.updateBoundary(object.minErrorBoundary());
        boundary_.updateApproximation(object.circleApproximation());
        totalTime_ += (object.timestamp() - timestamp_);
        timestamp_ = object.timestamp();
    }
    
    void setGoals(const ObjectGoalDistribution& goals) override
    {
        goals_ = goals;
    }
    
    float overlapWithObject(const LaserObject& object) const override
    {
        return 0.0f;
    }
    
    int64_t timeLastSeen(void) const override
    {
        return timestamp_;
    }
    
    int64_t totalTimeSeen(void) const override
    {
        return totalTime_;
    }
    
    Position position(void) const override
    {
        return motion_.position();
    }
    
    velocity_t velocity(void) const override
    {
        return motion_.velocity();
    }

    object_motion_state_t motionState(void) const override
    {
        return motion_.motion();
    }
    
    ObjectBoundary boundary(void) const override
    {
        return boundary_;
    }

    double radius(void) const override
    {
        return boundary_.circleApproximation().radius();
    }
    
    ObjectGoalDistribution goals(void) const override
    {
        return goals_;
    }
    
protected:
    
    int64_t        timestamp_;
    int64_t        totalTime_;
    MotionType     motion_;
    ObjectBoundary boundary_;
    ObjectGoalDistribution goals_;

    BoundedMovingObject(void) { }
    
private:
    
    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( timestamp_,
            totalTime_,
            motion_,
            boundary_,
            goals_);
    }
};

}
}

#endif // TRACKER_OBJECTS_BOUNDED_MOVING_OBJECT_H

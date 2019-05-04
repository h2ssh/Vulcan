/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     dynamic_object.h
* \author   Collin Johnson
*
* Declaration of DynamicObject base class.
*/

#ifndef TRACKER_DYNAMIC_OBJECT_H
#define TRACKER_DYNAMIC_OBJECT_H

#include <tracker/goal.h>
#include <tracker/object_boundary.h>
#include <tracker/object_state.h>
#include <tracker/types.h>
#include <core/pose.h>
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <memory>

namespace vulcan
{
namespace tracker
{

class DynamicObjectVisitor;
class LaserObject;
class ObjectMotion;

/**
* DynamicObject is the base class for objects being tracked by the object_tracker module. A DynamicObject corresponds
* to something moving in the environment. The object might be temporarily static, but isn't part of the static
* environment itself.
*
* Each DynamicObject has the following properties associated with it:
*
*   - an id                 -> unique identifier assigned to the object to be able to keep track of it over time
*   - a pose                -> where the object is in the environment
*   - a velocity            -> in the direction of the pose
*   - a boundary shape      -> portion of space occupied by the robot
*   - a recent trajectory   -> poses where the object has been in the recent past
*/
class DynamicObject
{
public:

    using Ptr      = std::shared_ptr<DynamicObject>;
    using ConstPtr = std::shared_ptr<const DynamicObject>;

    /**
    * id retrieves the unique id assigned to the object.
    */
    virtual ObjectId id(void) const = 0;

    /**
    * updateModel updates the model of the DynamicObject using a new laser object. The update will produce new
    * estimates of the object's position, velocity, and boundary.
    *
    * \param    object          Detected object that matches the dynamic object
    */
    virtual void updateModel(const LaserObject& object) = 0;

    /**
    * setGoal sets the goal associated with the dynamic object.
    */
    virtual void setGoals(const ObjectGoalDistribution& goals) = 0;

    /**
    * overlapWithObject calculates the amount of overlap this object has with the provided object. The number ranges
    * from 0 (no overlap) to 1 (completely overlapping boundaries).
    *
    * \param    object          LaserObject being matched
    * \return   Portion of the total area that overlaps with the provided object.
    */
    virtual float overlapWithObject(const LaserObject& object) const = 0;

    /**
    * timeLastSeen retrieves the last time the object was observed.
    */
    virtual int64_t timeLastSeen(void) const = 0;

    /**
    * totalTimeSeen retrieves the total time the object has been observed./
    */
    virtual int64_t totalTimeSeen(void) const = 0;

    /**
    * position retrieves the current position of the DynamicObject.
    */
    virtual Position position(void) const = 0;

    /**
    * velocity retrieves the estimated velocity of the object in meters per second.
    */
    virtual velocity_t velocity(void) const = 0;

    /**
    * motionState retrieves the full motion state estimate of the object -- position, velocity, acceleration.
    */
    virtual object_motion_state_t motionState(void) const = 0;

    /**
    * boundary retrieves the object's boundary.
    */
    virtual ObjectBoundary boundary(void) const = 0;

    /**
    * radius retrieves the object's estimated radius when a circle is fit to the boundary.
    */
    virtual double radius(void) const = 0;

    /**
    * goal retrieves the current goal estimate for the object.
    */
    virtual ObjectGoalDistribution goals(void) const = 0;

    /**
    * clone creates a copy of the DynamicObject.
    *
    * \return   A deep-copied instance of the appropriate DynamicObject subclass.
    */
    virtual std::unique_ptr<DynamicObject> clone(void) const = 0;

    /**
    * accept implements the accept function from the Visitor pattern. The appropriate visitXXX method will be called
    * for the visitor.
    *
    * \param    visitor         Visitor to be visited by this object
    */
    virtual void accept(DynamicObjectVisitor& visitor) const = 0;

    /**
    * Destructor for DynamicObject.
    */
    virtual ~DynamicObject(void) { }
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_DYNAMIC_OBJECT_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object.h
* \author   Collin Johnson
*
* Declaration of TrackingObject.
*/

#ifndef TRACKER_TRACKING_OBJECT_H
#define TRACKER_TRACKING_OBJECT_H

#include <tracker/laser_object.h>
#include <tracker/object_boundary.h>
#include <tracker/types.h>
#include <math/mathfwd.h>
#include <memory>

namespace vulcan
{
namespace tracker
{

class DynamicObject;
class DynamicObjectFactory;
class ObjectMotion;
class ObjectMotionClassifier;
class TrackingObject;

using TrackingObjectCollection = std::vector<std::shared_ptr<TrackingObject>>;

/**
* TrackingObject estimates the motion and boundary of an object over time. A TrackingObject maintains the state of a
* detected object and estimates the type of motion and boundary of the object. These properties are used to convert
* the TrackingObject into an appropriate instance of DynamicObject.
*/
class TrackingObject
{
public:

    /**
    * Constructor for TrackingObject.
    *
    * \param    id              Unique id assigned to the tracking object
    * \param    object          Newly detected LaserObject that will be tracked over time
    * \param    classifier      Classifier to use for the object's motion
    */
    TrackingObject(ObjectId id,
                   const LaserObject&                      object,
                   std::unique_ptr<ObjectMotionClassifier> classifier);

    /**
    * Copy constructor for TrackingObject.
    *
    * \param    rhs         Tracking object to be copied
    */
    TrackingObject(const TrackingObject& rhs);

    /**
    * Copy assignment operator for TrackingObject.
    */
    TrackingObject& operator=(const TrackingObject& rhs);

    // Move constructors can use the defaults
    TrackingObject(TrackingObject&& rhs) = default;
    TrackingObject& operator=(TrackingObject&& rhs) = default;

    /**
    * Destructor for TrackingObject.
    */
    ~TrackingObject(void);

    /**
    * id retrieves the id associated with the object.
    */
    ObjectId id(void) const { return id_; }

    /**
    * boundary retrieves the current boundary estimate.
    */
    const ObjectBoundary& boundary(void) const { return boundary_; }

    /**
    * motion retrieves the estimated motion type for the object.
    *
    * \return   The current type of estimated motion. nullptr if no such motion has been found.
    */
    const ObjectMotion& motion(void) const;

    /**
    * updateModel updates the current model of the object using a newly detected LaserObject.
    *
    * \param    object          Newly detected LaserObject that matches this tracking object
    */
    LaserObject updateModel(LaserObject object);

    /**
    * distanceTo calculates how far it is from this object to the specified object.
    *
    * \param    object          Object to determine distance to
    * \return   The min distance and avg distance to the object.
    */
    std::tuple<double, double> distanceTo(const LaserObject& object) const;

    /**
    * distanceTo calculates how far it is between the boundary of two objects
    *
    * \param    object          Object for which to calculate the distance
    * \return   The distance in meters between the two objects,
    */
    std::tuple<double, double> distanceTo(const TrackingObject& object) const;

    /**
    * lastUpdateTime retrieves the timestamp at which the TrackingObject was last updated via the updateModel method.
    */
    int64_t lastUpdateTime(void) const { return updateTime_; }

    /**
    * updateCount retrieves the total number of associations made with this object.
    */
    int updateCount(void) const { return updateCount_; }

    /**
    * startTime retrieve the time at which the object started being tracked.
    */
    int64_t startTime(void) const { return startTime_; }

    /**
    * toDynamicObject converts the TrackingObject into a DynamicObject.
    */
    std::shared_ptr<DynamicObject> toDynamicObject(DynamicObjectFactory& factory) const;

private:

    ObjectId id_;    ///< Unique identifier assigned to the object

    mutable std::unique_ptr<ObjectMotion>   currentMotion_; ///< Most probable current motion estimate
    ObjectBoundary                          boundary_;      ///< Current model of the boundary
    std::vector<Position>                   mostRecentPoints_;  ///< Points from most recent measurement
    std::unique_ptr<ObjectMotionClassifier> classifier_;    ///< Classifier for the motion
    bool                                    haveNewMotion_; ///< Flag indicating if the motion classification has
                                                            ///< changed, which means creating a new DynObj
    int64_t                                 updateTime_;    ///< Time at which the last update occurred
    int                                     updateCount_;   ///< Number of model updates that occurred
    int64_t                                 startTime_;     ///< Time the tracking object started being tracked

    bool isTwoCircles_ = false;
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_TRACKING_OBJECT_H

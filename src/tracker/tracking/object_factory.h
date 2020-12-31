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
* Declaration of TrackingObjectFactory.
*/

#ifndef TRACKER_TRACKING_OBJECT_FACTORY_H
#define TRACKER_TRACKING_OBJECT_FACTORY_H

#include "tracker/types.h"
#include <memory>

namespace vulcan
{
namespace tracker
{

class DataAssociationStrategy;
class LaserObject;
class ObjectMotionClassifier;
class TrackingObject;
class TrackingObjectSet;

/**
* TrackingObjectFactory is a factory for creating classes associated with TrackingObjects. The instances created by the
* factory are:
* 
*   - TrackingObjectSet
*   - TrackingObject
*
* A TrackingObject is created via the createTrackingObject method. All dependencies for a TrackingObject are injected
* by the TrackingObjectFactory.
* 
* A TrackingObjectSet is created via the createTrackingObjectSet method. All dependencies are injected by the factory.
*/
class TrackingObjectFactory
{
public:

    /**
    * Constructor for TrackingObjectFactory.
    *
    * \param    classifier              Motion classifier to be used for tracking objects
    * \param    associationStrategy     Data association strategy to use for object sets
    */
    TrackingObjectFactory(std::unique_ptr<ObjectMotionClassifier> classifier,
                          std::unique_ptr<DataAssociationStrategy> associationStrategy);

    /**
    * Destructor for TrackingObjectFactory.
    */
    ~TrackingObjectFactory(void);

    /**
    * createTrackingObject creates a new TrackingObject from a newly detected LaserObject.
    *
    * \param    object      Detected laser object that doesn't match an existing object
    * \return   A new instance of TrackingObject.
    */
    std::unique_ptr<TrackingObject> createTrackingObject(const LaserObject& object);
    
    /**
    * createTrackingObjectSet creates a new TrackingObjectSet for tracking TrackingObjects using a single laser.
    * 
    * \return A new instance of TrackingObjectSet.
    */
    std::unique_ptr<TrackingObjectSet> createTrackingObjectSet(void) const;
    
private:

    ObjectId nextId_ = 0;
    std::unique_ptr<ObjectMotionClassifier> classifier_;
    std::unique_ptr<DataAssociationStrategy> associationStrategy_;
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_TRACKING_OBJECT_FACTORY_H

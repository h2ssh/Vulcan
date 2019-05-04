/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object_set.h
* \author   Collin Johnson
*
* Declaration of TrackingObjectSet.
*/

#ifndef TRACKER_TRACKING_OBJECT_SET_H
#define TRACKER_TRACKING_OBJECT_SET_H

#include <tracker/tracking/object.h>
#include <boost/optional.hpp>
#include <memory>
#include <vector>

namespace vulcan
{
namespace tracker
{

class DataAssociationStrategy;
class ObjectBoundary;
class TrackingObject;

/**
* TrackingObjectSet contains a set of TrackingObjects. The objects in the set can be looked up via an ObjectBoundary.
* Tracked objects can be removed individually or in bulk using removeObjectsBeforeTime.
*
* Set intersection and set union operations are supported, which allow for tracking objects on a per laser basis and
* then combining them at the end by taking the union.
*
* The objects can also be iterated over.
*/
class TrackingObjectSet
{
public:

    using ConstIter = TrackingObjectCollection::const_iterator;
    
    /**
    * Constructor for TrackingObjectSet.
    * 
    * \param    associationStrategy         Data association strategy for finding objects in the set
    */
    TrackingObjectSet(std::unique_ptr<DataAssociationStrategy> associationStrategy);
    
    // Copy/move construction
    TrackingObjectSet(const TrackingObjectSet& rhs);
    TrackingObjectSet& operator=(const TrackingObjectSet& rhs);
    
    TrackingObjectSet(TrackingObjectSet&& rhs);
    TrackingObjectSet& operator=(TrackingObjectSet&& rhs);
    
    /**
    * Destructor for TrackingObjectSet.
    */
    ~TrackingObjectSet(void);
    
    /**
    * findObject finds the object that has the greatest intersection with the provided laser object. If no objects
    * intersect the laser object, then the optional will not contain an object.
    *
    * \param    object          Object to be matched
    * \return   A pointer to the matching object if one exists. Otherwise nullptr.
    */
    TrackingObject* findObject(const LaserObject& object) const;
    
    /**
    * addObject adds a new object to the set. If object == nullptr, then it won't be added.
    *
    * \param    object      Object to be added to the set
    * \return   True if the object was added. False if the object was nullptr or otherwise couldn't be added.
    */
    bool addObject(std::unique_ptr<TrackingObject> object);

    /**
    * removeObject removes an object from the set.
    *
    * \param    object      Object to be removed from the set
    * \return   True if the object was in the set and has been removed. False if the object wasn't found in the set.
    */
    bool removeObject(TrackingObject* object);

    /**
    * removeObjectsBeforeTime removes all objects from the set whose last update time was before the specified time.
    *
    * \param    time        Removal time to use -- if lastUpdateTime < time, the objects are removed
    * \return   The number of objects removed.
    */
    std::size_t removeObjectsBeforeTime(int64_t time);

    // Iterator support for looking at the objects in the set
    std::size_t size(void)  const { return objects_.size(); }
    ConstIter   begin(void) const { return objects_.cbegin(); }
    ConstIter   end(void)   const { return objects_.cend(); }
    
    /**
    * clear erases all objects from the set.
    * 
    * \post this.size() == 0
    */
    void clear(void) { objects_.clear(); }

private:

    std::vector<std::shared_ptr<TrackingObject>> objects_;
    std::unique_ptr<DataAssociationStrategy> associationStrategy_;
    
    std::shared_ptr<TrackingObject> matchingObject(const TrackingObject& object) const;
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_TRACKING_OBJECT_SET_H

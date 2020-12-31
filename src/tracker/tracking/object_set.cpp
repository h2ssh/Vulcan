/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object_set.cpp
* \author   Collin Johnson
*
* Definition of TrackingObjectSet.
*/

#include "tracker/tracking/object_set.h"
#include "tracker/tracking/data_association.h"
#include "tracker/laser_object.h"
#include <iostream>

// #define DEBUG_ASSOCIATION

namespace vulcan
{
namespace tracker
{
    
TrackingObjectSet::TrackingObjectSet(std::unique_ptr<DataAssociationStrategy> associationStrategy)
: associationStrategy_(std::move(associationStrategy))
{
}


TrackingObjectSet::TrackingObjectSet(const TrackingObjectSet& rhs)
: objects_(rhs.objects_)
, associationStrategy_(rhs.associationStrategy_->clone())
{
}


TrackingObjectSet& TrackingObjectSet::operator=(const TrackingObjectSet& rhs)
{
    objects_ = rhs.objects_;
    associationStrategy_ = rhs.associationStrategy_->clone();
    
    return *this;
}


TrackingObjectSet::TrackingObjectSet(TrackingObjectSet&& rhs)
: objects_(std::move(rhs.objects_))
, associationStrategy_(std::move(rhs.associationStrategy_))
{
}


TrackingObjectSet& TrackingObjectSet::operator=(TrackingObjectSet&& rhs)
{
    std::swap(objects_, rhs.objects_);
    std::swap(associationStrategy_, rhs.associationStrategy_);
    
    return *this;
}


TrackingObjectSet::~TrackingObjectSet(void)
{
    // For std::unique_ptr
}


TrackingObject* TrackingObjectSet::findObject(const LaserObject& object) const
{
    if(objects_.empty())
    {
        return nullptr;
    }
    
    auto assoc = associationStrategy_->associateLaserWithTracked(object, objects_);
    
    // If the returned index is valid, then an association was made
    return (assoc.index >= 0) && (assoc.index < static_cast<int>(objects_.size())) ? objects_[assoc.index].get() : nullptr;
}


bool TrackingObjectSet::addObject(std::unique_ptr<TrackingObject> object)
{
    // Check if an object already exists that overlaps
    if(object && !matchingObject(*object))
    {
        // If not, then add this object to the set
        objects_.emplace_back(std::move(object));
        return true;
    }
    
    return false;
}


bool TrackingObjectSet::removeObject(TrackingObject* object)
{
    // See if an object matches the pointer in the objects
    auto removedIt = std::remove_if(objects_.begin(), objects_.end(),
                                    [object](std::shared_ptr<TrackingObject>& o)
                                    {
                                        return o.get() == object;
                                    });
    
    // If so, then erase it
    bool erased = objects_.end() != removedIt;
    objects_.erase(removedIt, objects_.end());
    return erased;
}


std::size_t TrackingObjectSet::removeObjectsBeforeTime(int64_t time)
{
    auto removedIt = std::remove_if(objects_.begin(), objects_.end(),
                                    [time](std::shared_ptr<TrackingObject>& o)
                                    {
                                        return o->lastUpdateTime() < time;
                                    });
    std::size_t numErased = std::distance(removedIt, objects_.end());
    objects_.erase(removedIt, objects_.end());
    return numErased;
}


std::shared_ptr<TrackingObject> TrackingObjectSet::matchingObject(const TrackingObject& object) const
{
    auto association = associationStrategy_->associateObjectWithTracked(object, objects_);
    
    if((association.index >= 0) && (association.index < static_cast<int>(objects_.size())))
    {
        return objects_[association.index];
    }
    
    return nullptr;
}

} // namespace tracker
} // namespace vulcan

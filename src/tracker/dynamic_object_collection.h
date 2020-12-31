/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     dynamic_object_collection.h
 * \author   Collin Johnson
 *
 * Declaration of DynamicObjectCollection.
 */

#ifndef TRACKER_DYNAMIC_OBJECT_COLLECTION_H
#define TRACKER_DYNAMIC_OBJECT_COLLECTION_H

#include "system/message_traits.h"
#include "tracker/dynamic_object.h"
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace tracker
{

/**
 * DynamicObjectCollection
 */
class DynamicObjectCollection
{
public:
    using Collection = std::vector<DynamicObject::ConstPtr>;
    using ConstIter = Collection::const_iterator;

    /**
     * Default constructor for DynamicObjectCollection.
     */
    explicit DynamicObjectCollection(int64_t timestamp = -1) : timestamp_(timestamp) { }

    /**
     * Constructor for DynamicObjectCollection.
     *
     * \param    objects         Objects to initially put in the collection
     * \param    timestamp       Time at which the scan was taken for these objects
     */
    DynamicObjectCollection(Collection objects, int64_t timestamp) : timestamp_(timestamp), objects_(std::move(objects))
    {
    }

    /**
     * push adds an object to the collection.
     *
     * \param    object      Object to add to the collection
     */
    void push(DynamicObject::ConstPtr object)
    {
        assert(object);
        objects_.push_back(std::move(object));
    }

    /**
     * timestamp retrieves the timestamp of the collection.
     */
    int64_t timestamp(void) const { return timestamp_; }

    /**
     * setTimestamp adjusts the timestamp of the collection.
     */
    void setTimestamp(int64_t newTime) { timestamp_ = newTime; }

    /**
     * visitAll visits each object in the collection.
     */
    void visitAll(DynamicObjectVisitor& visitor) const
    {
        for (auto& o : objects_) {
            o->accept(visitor);
        }
    }

    // Iterator support for the collection
    std::size_t size(void) const { return objects_.size(); }
    bool empty(void) const { return objects_.empty(); }
    ConstIter begin(void) const { return objects_.cbegin(); }
    ConstIter end(void) const { return objects_.cend(); }

private:
    int64_t timestamp_;
    Collection objects_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(timestamp_, objects_);
    }
};

}   // namespace tracker
}   // namespace vulcan

DEFINE_SYSTEM_MESSAGE(tracker::DynamicObjectCollection, ("TRACKER_DYNAMIC_OBJECTS"))

#endif   // TRACKER_DYNAMIC_OBJECT_COLLECTION_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     laser_object_collection.h
 * \author   Collin Johnson
 *
 * Definition of LaserObjectCollection.
 */

#ifndef TRACKER_LASER_OBJECT_COLLECTION_H
#define TRACKER_LASER_OBJECT_COLLECTION_H

#include "system/message_traits.h"
#include "tracker/laser_object.h"
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace tracker
{

/**
 * LaserObjectCollection represents the objects detected in a single laser scan. The collection maintains the id of the
 * laser along with the time at which the scan was taken.
 */
class LaserObjectCollection
{
public:
    using ObjectConstIter = std::vector<LaserObject>::const_iterator;

    /**
     * Default constructor for LaserObjectCollection.
     *
     * Creates an empty collection.
     */
    LaserObjectCollection(void) : scanTimestamp_(0), laserId_(-1) { }

    /**
     * Constructor for LaserObjectCollection.
     *
     * \param    objects             Objects for the collection
     * \param    laserId             Id of the laser that detected the objects
     * \param    timestamp           Timestamp of the scan
     */
    LaserObjectCollection(std::vector<LaserObject> objects, int32_t laserId, int64_t timestamp)
    : scanTimestamp_(timestamp)
    , laserId_(laserId)
    , objects_(std::move(objects))
    {
    }

    /**
     * timestamp retrieves the time at which the scan containing the objects was measured.
     */
    int64_t timestamp(void) const { return scanTimestamp_; }

    /**
     * laserId retrieves the id of the laser that took the scan.
     */
    int32_t laserId(void) const { return laserId_; }

    // Iterator access to the objects
    std::size_t size(void) const { return objects_.size(); }
    bool empty(void) const { return objects_.empty(); }
    ObjectConstIter begin(void) const { return objects_.cbegin(); }
    ObjectConstIter end(void) const { return objects_.cend(); }

private:
    int64_t scanTimestamp_;
    int32_t laserId_;
    std::vector<LaserObject> objects_;


    // Serialization support
    friend class cereal::access;

    template <typename Archive>
    void serialize(Archive& ar)
    {
        ar(scanTimestamp_, laserId_, objects_);
    }
};

}   // namespace tracker
}   // namespace vulcan

DEFINE_DEBUG_MESSAGE(tracker::LaserObjectCollection, ("DEBUG_TRACKER_LASER_OBJECTS"))

#endif   // TRACKER_LASER_OBJECT_COLLECTION_H

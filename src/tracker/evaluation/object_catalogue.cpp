/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     object_catalogue.cpp
 * \author   Collin Johnson
 *
 * Definition of ObjectCatalogue.
 */

#include "tracker/evaluation/object_catalogue.h"
#include "tracker/dynamic_object_collection.h"
#include "utils/algorithm_ext.h"

#define DEBUG_RESET

namespace vulcan
{
namespace tracker
{

ObjectCatalogue::ObjectCatalogue(int64_t maxForwardTimeJumpMs)
: lastUpdateTime_(0)
, maxForwardJumpUs_(maxForwardTimeJumpMs * 1000ll)
{
}


int ObjectCatalogue::addObjects(const DynamicObjectCollection& objects)
{
    // Check if time has gone backward or jumped ahead
    if ((objects.timestamp() < lastUpdateTime_) || (objects.timestamp() - lastUpdateTime_ > maxForwardJumpUs_)) {
        tracks_.clear();

#ifdef DEBUG_RESET
        std::cout << "DEBUG:ObjectCatalogue: Time jumped " << ((objects.timestamp() - lastUpdateTime_) / 1000)
                  << "ms. Resetting the catalogue.\n";
#endif
    }

    int numAdded = 0;

    for (auto& o : objects) {
        auto trackIt = std::find_if(tracks_.begin(), tracks_.end(), [&o](const ObjectTrack& t) {
            return t.id() == o->id();
        });

        if (trackIt != tracks_.end()) {
            tracks_.emplace_back(o);
            ++numAdded;
        } else {
            trackIt->addEstimate(o);
        }
    }

    lastUpdateTime_ = objects.timestamp();
    return numAdded;
}


bool ObjectCatalogue::hasTrackForObject(ObjectId id) const
{
    return utils::contains_if(tracks_, [id](const ObjectTrack& t) {
        return t.id() == id;
    });
}


boost::optional<ObjectTrack> ObjectCatalogue::trackForObject(ObjectId id)
{
    auto trackIt = std::find_if(tracks_.begin(), tracks_.end(), [id](const ObjectTrack& t) {
        return t.id() == id;
    });

    if (trackIt != tracks_.end()) {
        return *trackIt;
    } else {
        return boost::none;
    }
}

}   // namespace tracker
}   // namespace vulcan

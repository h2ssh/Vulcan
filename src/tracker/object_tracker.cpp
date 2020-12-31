/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     object_tracker.cpp
 * \author   Collin Johnson
 *
 * Definition of ObjectTracker.
 */

#include "tracker/object_tracker.h"
#include "tracker/laser_object_collection.h"
#include "tracker/objects/object_factory.h"
#include "tracker/tracking/object_factory.h"
#include "utils/config_file.h"
#include <boost/range/iterator_range.hpp>
#include <unordered_map>

#define DEBUG_TRACKING

namespace vulcan
{
namespace tracker
{

////////////////// object_tracker_params_t /////////////////////////

const std::string TRACKER_HEADING("ObjectTrackerParameters");
const std::string ASSOCIATION_TYPE_KEY("data_association_type");
const std::string MAX_MERGE_KEY("max_merge_distance_m");
const std::string MAX_TRACK_TIME_KEY("max_tracking_time_ms");
const std::string MAX_RADIUS_KEY("max_object_radius_m");
const std::string MIN_RADIUS_KEY("min_object_radius_m");
const std::string FALLOFF_GAIN_KEY("propagation_falloff_gain");

object_tracker_params_t::object_tracker_params_t(const utils::ConfigFile& config)
{
    dataAssociationType = config.getValueAsString(TRACKER_HEADING, ASSOCIATION_TYPE_KEY);
    maxMergeDistance = config.getValueAsFloat(TRACKER_HEADING, MAX_MERGE_KEY);
    maxTrackingTime = config.getValueAsInt32(TRACKER_HEADING, MAX_TRACK_TIME_KEY) * 1000l;
    maxObjectRadius = config.getValueAsFloat(TRACKER_HEADING, MAX_RADIUS_KEY);
    minObjectRadius = config.getValueAsFloat(TRACKER_HEADING, MIN_RADIUS_KEY);
    propagationFalloffGain = config.getValueAsFloat(TRACKER_HEADING, FALLOFF_GAIN_KEY);
}

//////////////////// ObjectTracker ////////////////////////////////

ObjectTracker::ObjectTracker(const object_tracker_params_t& params,
                             std::unique_ptr<TrackingObjectFactory> trackingObjFactory)
: params_(params)
, objectSet_(trackingObjFactory->createTrackingObjectSet())
, trackingObjectFactory_(std::move(trackingObjFactory))
{
}


ObjectTracker::~ObjectTracker(void)
{
    // For std::unique_ptr
}


TrackingObjectSet ObjectTracker::trackObjects(const LaserObjectCollection& objects, LaserObjectCollection* finalObjects)
{
    lastUpdateTime_ = objects.timestamp();

#ifdef DEBUG_TRACKING
    std::cout << "DEBUG:ObjectTracker: Incorporating " << objects.size() << " new objects from laser "
              << objects.laserId() << " into set with " << objectSet_->size() << " objects.\n";
#endif

    // Match laser ray clusters to existing tracking objects
    // Create new tracking objects for unmatched clusters
    trackNewClusters(objects, finalObjects);

    // Remove old objects that haven't been seen in awhile
    removeOldObjects();

#ifdef DEBUG_TRACKING
    std::cout << "Final set size: " << objectSet_->size() << '\n';
#endif

    return *objectSet_;
}


void ObjectTracker::clearObjects(void)
{
    objectSet_->clear();
}


void ObjectTracker::trackNewClusters(const LaserObjectCollection& objects, LaserObjectCollection* mergedObjects)
{
    // Each LaserObject matches to the closest TrackingObject.
    // If multiple LaserObjects match the same TrackingObject, then those LaserObjects need to be merged
    // and then the TrackingObject is updated.
    // Don't pay attention to multiple TrackingObjects matching a single LaserObject. That case naturally works out
    // as the TrackingObjects that aren't updated just fade away
    int numAdded = 0;
    int numUpdated = 0;

    std::unordered_multimap<TrackingObject*, const LaserObject*> trackingToLaser;

    for (auto& laserObj : objects) {
        auto matchedObject = objectSet_->findObject(laserObj);

        if (matchedObject) {
            trackingToLaser.insert(std::make_pair(matchedObject, &laserObj));
            ++numUpdated;
        } else {
            addObjectForClusterToSet(laserObj);
            ++numAdded;
        }
    }

    std::vector<const LaserObject*> toMerge;
    std::vector<TrackingObject*> invalidTrackedObjects;
    std::vector<LaserObject> finalLaserObjects;
    // Go through the tracking objects
    for (auto& trackingObj : *objectSet_) {
        // Find all matching laser objects
        auto laserMatches = trackingToLaser.equal_range(trackingObj.get());
        int numMatches = std::distance(laserMatches.first, laserMatches.second);

        // If two objects match, then two legs likely split apart and need to be re-grouped into a single object
        //         if(numMatches == 2)
        if (numMatches > 1) {
            toMerge.resize(numMatches);
            std::transform(laserMatches.first,
                           laserMatches.second,
                           toMerge.begin(),
                           [](const std::pair<TrackingObject*, const LaserObject*>& o) {
                               return o.second;
                           });

            LaserObject mergedObject(toMerge);
            auto finalObject = trackingObj->updateModel(mergedObject);

            if (mergedObjects) {
                finalLaserObjects.push_back(finalObject);
            }
        }
        // If there's a single match, update the model using the matched laser object
        else if (numMatches == 1) {
            auto finalObject = trackingObj->updateModel(*laserMatches.first->second);

            if (mergedObjects) {
                finalLaserObjects.push_back(finalObject);
            }
        }
        // If more than two objects match, the existing tracking object is in some sort of invalid state, likely because
        // it was created from an incorrectly. In this case, the tracked object should be removed and it should be
        // turned back into individual objects
        else if (numMatches > 2) {
            for (auto& laserObj : boost::make_iterator_range(laserMatches.first, laserMatches.second)) {
                addObjectForClusterToSet(*laserObj.second, trackingObj.get());
                ++numAdded;
            }

            invalidTrackedObjects.push_back(trackingObj.get());
        }
    }

    for (auto& invalid : invalidTrackedObjects) {
        objectSet_->removeObject(invalid);
    }

    if (mergedObjects) {
        *mergedObjects = LaserObjectCollection(finalLaserObjects, objects.laserId(), objects.timestamp());
    }

#ifdef DEBUG_TRACKING
    std::cout << "Matched: " << numUpdated << " Added:" << numAdded << " Invalid:" << invalidTrackedObjects.size()
              << '\n';
#endif
}


void ObjectTracker::addObjectForClusterToSet(const LaserObject& cluster, TrackingObject* parent)
{
    // Otherwise create a new object via the factory
    if (parent) {
        // Spawn
        std::cout << "Want to spawn from " << parent->boundary().circleApproximation() << '\n';
    } else {
        objectSet_->addObject(trackingObjectFactory_->createTrackingObject(cluster));
    }
}


void ObjectTracker::removeOldObjects(void)
{
    auto numRemoved = objectSet_->removeObjectsBeforeTime(lastUpdateTime_ - params_.maxTrackingTime);

#ifdef DEBUG_TRACKING
    std::cout << "Removed:" << numRemoved << '\n';
#endif
}

}   // namespace tracker
}   // namespace vulcan

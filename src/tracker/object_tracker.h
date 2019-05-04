/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object_tracker.h
* \author   Collin Johnson
*
* Declaration of ObjectTracker.
*/

#ifndef TRACKER_OBJECT_TRACKER_H
#define TRACKER_OBJECT_TRACKER_H

#include <tracker/tracking/object_set.h>
#include <map>

namespace vulcan
{
namespace utils { class ConfigFile; }
namespace tracker
{

class LaserObjectCollection;
class TrackingObjectFactory;

/**
* object_tracker_params_t
*/
struct object_tracker_params_t
{
    std::string dataAssociationType;
    float   maxMergeDistance;
    int64_t maxTrackingTime;
    float   maxObjectRadius;
    float   minObjectRadius;
    float   propagationFalloffGain;

    object_tracker_params_t(void) = default;
    object_tracker_params_t(const utils::ConfigFile& config);
};

/**
* ObjectTracker is responsible for tracking moving objects in the vicinity of the robot. The tracker takes LaserObjects
* extracted from the dynamic laser rays in the map and maintains their state over time by converting them into
* DynamicObjects.
*
* The tracking process is completed in a few steps:
*
*   1) Classify LaserObjects (might take a few updates)
*   2) After classification, create an appropriate DynamicObject.
*   3) Build the model of the DynamicObject over time.
*
* The ObjectTracker matches LaserObjects to TrackingObjects. There are four situations that can occur when doing this
* matching process:
*
*   # LO    # TO     Result
*  --------------------------
*     1  ->   1  ->  1 TO updated
*    >1  ->   1  ->  Merge LO, 1 TO updated
*     1  ->  >1  ->  Update closest TO, others will eventually die off
*     1  ->   0  ->  Add new TO
*
*/
class ObjectTracker
{
public:

    /**
    * Constructor for ObjectTracker.
    *
    * \param    params                  Parameters controlling the operation of the tracker
    * \param    trackingObjFactory      Tracking object factory to use for creating new tracking objects
    */
    ObjectTracker(const object_tracker_params_t& params, std::unique_ptr<TrackingObjectFactory> trackingObjFactory);

    /**
    * Destructor for ObjectTracker.
    */
    ~ObjectTracker(void);

    /**
    * trackObjects updates the tracking information for a set of LaserObject segmented from a
    * laser scan. The steps for the tracking are detailed in the class description.
    *
    * \param    objects             Objects segmented from a laser scan
    * \param    mergedObjects       The tracker may merge individual objects during the data association, these
    *       objects are optionally stored here for further debugging
    * \return   The current set of objects being tracked.
    */
    TrackingObjectSet trackObjects(const LaserObjectCollection& objects,
                                   LaserObjectCollection* mergedObjects = nullptr);

    /**
    * clearObjects clears all objects that are being tracked.
    */
    void clearObjects(void);

private:

    object_tracker_params_t params_;
    std::unique_ptr<TrackingObjectSet> objectSet_;
    std::unique_ptr<TrackingObjectFactory> trackingObjectFactory_;

    int64_t lastUpdateTime_ = 0;

    void trackNewClusters(const LaserObjectCollection& objects, LaserObjectCollection* mergedObjects);
    void addObjectForClusterToSet(const LaserObject& cluster, TrackingObject* parent = nullptr);
    void removeOldObjects(void);
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_OBJECT_TRACKER_H

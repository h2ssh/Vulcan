/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object.cpp
* \author   Collin Johnson
*
* Definition of TrackingObject.
*/

#include "tracker/tracking/object.h"
#include "tracker/motions/classifier.h"
#include "tracker/objects/object_factory.h"
#include "tracker/laser_object.h"
#include "tracker/object_motion.h"
#include "tracker/dynamic_object.h"
#include <iostream>

namespace vulcan
{
namespace tracker
{

TrackingObject::TrackingObject(ObjectId id,
                               const LaserObject&                      object,
                               std::unique_ptr<ObjectMotionClassifier> classifier)
: id_(id)
, boundary_(object.minErrorBoundary())
, mostRecentPoints_(object.begin(), object.end())
, classifier_(std::move(classifier))
, haveNewMotion_(false)
, updateTime_(object.timestamp())
, updateCount_(0)
, startTime_(object.timestamp())
{
    boundary_.assignApproximation(object.circleApproximation());
    updateModel(object);
}


TrackingObject::TrackingObject(const TrackingObject& rhs)
: id_(rhs.id_)
, boundary_(rhs.boundary_)
, mostRecentPoints_(rhs.mostRecentPoints_)
, classifier_(new ObjectMotionClassifier(*rhs.classifier_))
, haveNewMotion_(rhs.haveNewMotion_)
, updateTime_(rhs.updateTime_)
, startTime_(rhs.startTime_)
{
    if(rhs.currentMotion_)
    {
        currentMotion_ = rhs.currentMotion_->clone();
    }
}


TrackingObject& TrackingObject::operator=(const TrackingObject& rhs)
{
    id_ = rhs.id_;
    boundary_ = rhs.boundary_;
    classifier_.reset(new ObjectMotionClassifier(*rhs.classifier_));
    haveNewMotion_ = rhs.haveNewMotion_;
    updateTime_ = rhs.updateTime_;
    startTime_ = rhs.startTime_;

    if(rhs.currentMotion_)
    {
        currentMotion_ = rhs.currentMotion_->clone();
    }

    return *this;
}


TrackingObject::~TrackingObject(void)
{
    // For std::unique_ptr
}


const ObjectMotion& TrackingObject::motion(void) const
{
    if(!currentMotion_)
    {
        currentMotion_ = classifier_->createObjectMotion();
        assert(currentMotion_);
    }

    return *currentMotion_;
}


LaserObject TrackingObject::updateModel(LaserObject object)
{
//     if(isTwoCircles_ && (object.minErrorBoundary().type() != BoundaryType::two_circles))
//     {
//         object = object.generateShadowedCircle();
//     }

    updateTime_ = object.timestamp();
    ++updateCount_;

    // Update the model of the object --- boundary + motion
    boundary_.updateBoundary(object.minErrorBoundary());
    boundary_.updateApproximation(object.circleApproximation());
    mostRecentPoints_.clear();
    mostRecentPoints_.insert(mostRecentPoints_.end(), object.begin(), object.end());
    classifier_->updateClassification(object);

    if(!currentMotion_ || classifier_->hasClassificationChanged())
    {
        currentMotion_ = classifier_->createObjectMotion();
    }

    currentMotion_->updateModel(object);
    isTwoCircles_ |= boundary_.type() == BoundaryType::two_circles;

    return object;
}


std::tuple<double, double> TrackingObject::distanceTo(const LaserObject& object) const
{
    auto trackedToLaser = boundary_.distanceFromBoundary(object.begin(), object.end());
    auto laserToTracked = object.minErrorBoundary().distanceFromBoundary(mostRecentPoints_.begin(),
                                                                       mostRecentPoints_.end());

    return std::make_tuple(std::min(std::get<0>(trackedToLaser), std::get<0>(laserToTracked)),
                           std::min(std::get<1>(trackedToLaser), std::get<1>(laserToTracked)));
}


std::tuple<double, double> TrackingObject::distanceTo(const TrackingObject& object) const
{
    auto trackedToObj = boundary_.distanceFromBoundary(object.mostRecentPoints_.begin(), object.mostRecentPoints_.end());
    auto objToTracked = object.boundary_.distanceFromBoundary(mostRecentPoints_.begin(), mostRecentPoints_.end());

    return std::make_tuple(std::min(std::get<0>(trackedToObj), std::get<0>(objToTracked)),
                           std::min(std::get<1>(trackedToObj), std::get<1>(objToTracked)));
}


std::shared_ptr<DynamicObject> TrackingObject::toDynamicObject(DynamicObjectFactory& factory) const
{
    return factory.createDynamicObject(id_, updateTime_, *currentMotion_, boundary_);
}

} // namespace tracker
} // namespace vulcan

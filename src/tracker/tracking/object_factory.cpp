/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object_factory.cpp
* \author   Collin Johnson
*
* Definition of TrackingObjectFactory.
*/

#include <tracker/tracking/object_factory.h>
#include <tracker/tracking/data_association.h>
#include <tracker/tracking/object.h>
#include <tracker/tracking/object_set.h>
#include <tracker/motions/classifier.h>
#include <iostream>

namespace vulcan
{
namespace tracker
{

TrackingObjectFactory::TrackingObjectFactory(std::unique_ptr<ObjectMotionClassifier> classifier,
                                             std::unique_ptr<DataAssociationStrategy> associationStrategy)
: classifier_(std::move(classifier))
, associationStrategy_(std::move(associationStrategy))
{
}


TrackingObjectFactory::~TrackingObjectFactory(void)
{
    // For std::unique_ptr
}


std::unique_ptr<TrackingObject> TrackingObjectFactory::createTrackingObject(const LaserObject& object)
{
    ++nextId_;
    return std::make_unique<TrackingObject>(nextId_,
                                            object,
                                            std::make_unique<ObjectMotionClassifier>(*classifier_));
}


std::unique_ptr<TrackingObjectSet> TrackingObjectFactory::createTrackingObjectSet(void) const
{
    return std::unique_ptr<TrackingObjectSet>(new TrackingObjectSet(associationStrategy_->clone()));
}

} // namespace tracker
} // namespace vulcan

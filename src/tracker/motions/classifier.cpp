/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     classifier.cpp
 * \author   Collin Johnson
 *
 * Definition of ObjectMotionClassifier.
 */

#include "tracker/motions/classifier.h"
#include "tracker/laser_object.h"
#include "tracker/motions/fixed_endpoint.h"
#include "tracker/motions/stationary.h"
#include "tracker/motions/steady.h"
#include "tracker/motions/striding.h"
#include "utils/config_file.h"
#include <cassert>

// #define DEBUG_CLASSIFICATION

namespace vulcan
{
namespace tracker
{

const std::string kMotionClassifierHeading("ObjectMotionClassifierParameters");
const std::string kDurationKey("max_trajectory_duration_ms");


object_motion_classifier_params_t::object_motion_classifier_params_t(const utils::ConfigFile& config)
: maxTrajectoryDurationMs(config.getValueAsInt32(kMotionClassifierHeading, kDurationKey))
{
    assert(maxTrajectoryDurationMs > 0);
}


ObjectMotionClassifier::ObjectMotionClassifier(const object_motion_classifier_params_t& params,
                                               const tracking_filter_params_t& filterParams)
: kMaxTrajectoryDurationUs(params.maxTrajectoryDurationMs * 1000)
, type_(MotionType::unclassified)
, lastUpdateTime_(0)
, startTime_(0)
, maxSize_(0)
, lastPosition_(0, 0)
, hasTypeChanged_(false)
, filterParams_(filterParams)
{
}


ObjectMotionClassifier::ObjectMotionClassifier(const ObjectMotionClassifier& rhs)
: kMaxTrajectoryDurationUs(rhs.kMaxTrajectoryDurationUs)
, type_(rhs.type_)
, lastUpdateTime_(rhs.lastUpdateTime_)
, startTime_(rhs.startTime_)
, maxSize_(rhs.maxSize_)
, lastPosition_(rhs.lastPosition_)
, hasTypeChanged_(rhs.hasTypeChanged_)
, filterParams_(rhs.filterParams_)
{
    if (rhs.fixed_) {
        fixed_.reset(new FixedEndpointMotion(*rhs.fixed_));
    }

    if (rhs.striding_) {
        striding_.reset(new StridingMotion(*rhs.striding_));
    }

    if (rhs.steady_) {
        steady_.reset(new SteadyMotion(*rhs.steady_));
    }

    if (rhs.stationary_) {
        stationary_.reset(new StationaryMotion(*rhs.stationary_));
    }
}


ObjectMotionClassifier::~ObjectMotionClassifier(void)
{
    // For std::unique_ptr
}


void ObjectMotionClassifier::updateClassification(const LaserObject& object)
{
    // Store information about the current measured position of the object
    lastPosition_ = object.center();
    lastUpdateTime_ = object.timestamp();

    if (startTime_ == 0) {
        startTime_ = object.timestamp();
    }

    initializeModelsIfNeeded();

    auto currentType = determineCurrentMotionType(object);

    if (shouldChangeMotionType(currentType)) {
        type_ = currentType;
        hasTypeChanged_ = true;
    }
}


bool ObjectMotionClassifier::hasClassificationChanged(void) const
{
    return hasTypeChanged_;
}


std::unique_ptr<ObjectMotion> ObjectMotionClassifier::createObjectMotion(void)
{
    hasTypeChanged_ = false;

    switch (type_) {
    case MotionType::striding:
        assert(striding_);
        return std::unique_ptr<ObjectMotion>(new StridingMotion(*striding_));

    case MotionType::fixed_endpoint:
        assert(fixed_);
        return std::unique_ptr<ObjectMotion>(new FixedEndpointMotion(*fixed_));

    case MotionType::steady:
        assert(steady_);
        return std::unique_ptr<ObjectMotion>(new SteadyMotion(*steady_));

    case MotionType::stationary:
        assert(stationary_);
        return std::unique_ptr<ObjectMotion>(new StationaryMotion(*stationary_));

    case MotionType::unclassified:
        assert(stationary_);
        return std::unique_ptr<ObjectMotion>(new StationaryMotion(*stationary_));
    }

    std::cerr << "ERROR! ObjectMotionClassifier::createObjectMotion: Unknown motion type\n";
    assert(false);
    return std::unique_ptr<ObjectMotion>();
}


void ObjectMotionClassifier::initializeModelsIfNeeded(void)
{
    if (!steady_) {
        steady_.reset(new SteadyMotion(lastUpdateTime_, lastPosition_, filterParams_));
    }

    if (!striding_) {
        striding_.reset(new StridingMotion(lastUpdateTime_, lastPosition_, filterParams_));
    }

    if (!fixed_) {
        fixed_.reset(new FixedEndpointMotion());
    }

    if (!stationary_) {
        stationary_.reset(new StationaryMotion(lastPosition_));
    }
}


ObjectMotionClassifier::MotionType ObjectMotionClassifier::determineCurrentMotionType(const LaserObject& object)
{
    assert(steady_);
    assert(striding_);
    assert(fixed_);
    assert(stationary_);

    enum
    {
        kSteadyIndex,
        kStridingIndex,
        kFixedIndex,
        kStationaryIndex,
        kNumModels
    };

    maxSize_ = std::max(object.size(), maxSize_);

    if ((maxSize_ < 5) || (object.timestamp() - startTime_ < 100000)) {
        return MotionType::stationary;
    } else {
        return MotionType::steady;
    }


    //     std::array<ObjectMotionStatus, kNumModels> statuses;
    //     std::array<std::string,        kNumModels> names = { "steady", "striding", "fixed", "stationary" };
    //
    //     statuses[kSteadyIndex]     = steady_->updateModel(object);
    //     statuses[kStridingIndex]   = striding_->updateModel(object);
    //     statuses[kFixedIndex]      = fixed_->updateModel(object);
    //     statuses[kStationaryIndex] = stationary_->updateModel(object);
    //
    // #ifdef DEBUG_CLASSIFICATION
    //     std::cout << "DEBUG: ObjectMotionClassifier: Object classifications for " << lastPosition_ << ":\n";
    //     for(std::size_t n = 0; n < statuses.size(); ++n)
    //     {
    //         std::cout << names[n] << " : " << statuses[n] << '\n';
    //     }
    //     std::cout << '\n';
    // #endif
    //
    //     // If any of the models are currently undetermined, then the overall motion remains unclassified
    //     int numUndetermined = std::count(statuses.begin(), statuses.end(), ObjectMotionStatus::undetermined);
    //     if(numUndetermined > 0)
    //     {
    // #ifdef DEBUG_CLASSIFICATION
    //         std::cout << "DEBUG: ObjectMotionClassifier: Found " << numUndetermined << " undetermined motion
    //         types.\n";
    // #endif
    //         return MotionType::unclassified;
    //     }
    //
    //     int numValid = std::count(statuses.begin(), statuses.end(), ObjectMotionStatus::valid);
    //
    //     // If none are deemed valid, then still unclassified
    //     if(numValid == 0)
    //     {
    //         std::cout << "INFO: ObjectMotionClassifier: No valid classifications found for the motion.\n";
    //         return MotionType::unclassified;
    //     }
    //     else if(numValid > 2)
    //     {
    //         std::cout << "INFO:ObjectMotionClassifier: Too many classifications. Waiting to see if one goes away.\n";
    //         return MotionType::unclassified;
    //     }
    //
    //     // If either striding or fixed are valid by themselves, then that's the classification
    //     // Otherwise, steady is almost always valid and stationary is almost never valid.
    //     if(statuses[kFixedIndex] == ObjectMotionStatus::valid)
    //     {
    //         return MotionType::fixed_endpoint;
    //     }
    //     else if(statuses[kStridingIndex] == ObjectMotionStatus::valid)
    //     {
    //         return MotionType::striding;
    //     }
    //     else if(statuses[kStationaryIndex] == ObjectMotionStatus::valid)
    //     {
    //         return MotionType::stationary;
    //     }
    //     else // steady is always valid at this point
    //     {
    //         return MotionType::steady;
    //     }
}


bool ObjectMotionClassifier::shouldChangeMotionType(MotionType currentType)
{
    /*
     * The following changes in motion type classifications are allowed:
     *
     * - unclassified -> anything  :  once a motion is classified, it should no longer be unclassified
     * - stationary -> anything    :  an object can start moving at any time
     * - steady -> striding,fixed_endpoint : steady motion is the default motion, but once more information is obtained,
     *       in particular, if the stop-swing phase is observed, or the limited motion of a fixed_endpoint is observed,
     *       then the steady motion can be converted
     * - striding/fixed_endpoint -> nothing else
     */

    if (currentType == type_) {
        return false;
    }

    switch (type_) {
    case MotionType::unclassified:
        return true;

    case MotionType::stationary:
        return true;

    case MotionType::steady:
        return (currentType == MotionType::fixed_endpoint) || (currentType == MotionType::striding);

    case MotionType::fixed_endpoint:
        return (currentType != MotionType::stationary);

    case MotionType::striding:
        return (currentType != MotionType::stationary);
    }

    return false;
}

}   // namespace tracker
}   // namespace vulcan

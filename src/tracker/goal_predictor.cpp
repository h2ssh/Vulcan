/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     goal_predictor.cpp
 * \author   Collin Johnson
 *
 * Definition of GoalPredictor.
 */

#include "tracker/goal_predictor.h"
#include "tracker/goals/goal_estimator.h"
#include "tracker/goals/goal_estimator_factory.h"
#include "tracker/objects/object_factory.h"
#include "tracker/tracking/object_set.h"
#include "tracker/tracking_environment.h"
#include "utils/algorithm_ext.h"

namespace vulcan
{
namespace tracker
{

GoalPredictor::GoalPredictor(std::unique_ptr<GoalEstimatorFactory> estimatorFactory,
                             std::unique_ptr<DynamicObjectFactory> dynObjFactory)
: estimatorFactory_(std::move(estimatorFactory))
, dynObjFactory_(std::move(dynObjFactory))
{
}


GoalPredictor::~GoalPredictor(void)
{
    // For std::unique_ptr
}

/**
 * predictGoals predicts goals for the current set of tracking objects in the environment. The objects will then
 * be converted into full DynamicObjects by having an estimated motion and an estimated goal.
 *
 * \param    objects         Tracked objects in the environment to predict goals for
 * \param    environment     Static environment representation in which the robot is operating
 * \return   The collection of DynamicObjects being tracked in the environment with motion states and goals.
 */
DynamicObjectCollection GoalPredictor::predictGoals(const TrackingObjectSet& objects,
                                                    const tracking_environment_t& environment)
{
    DynamicObjectCollection dynObjects(environment.timestamp);

    // Store the active ids so no longer active objects can have their goal predictors erased
    std::vector<ObjectId> inactiveIds(estimators_.size());
    std::transform(estimators_.begin(), estimators_.end(), inactiveIds.begin(), [](const auto& objToEst) {
        return objToEst.first;
    });

    for (auto& obj : objects) {
        auto estimatorIt = estimators_.find(obj->id());

        // The object exists in the set, so it isn't one of the inactive ids
        utils::erase_remove(inactiveIds, obj->id());

        // Create a new estimator based on the motion and environment.
        auto newEstimator = estimatorFactory_->createGoalEstimator(obj->motion(), environment);

        // If no goal estimator exists yet for this object
        // Or if the type of estimator changes then reassign the
        if ((estimatorIt == estimators_.end()) || (typeid(*newEstimator) != typeid(*estimatorIt->second))) {
            estimators_[obj->id()] = std::move(newEstimator);
            estimatorIt = estimators_.find(obj->id());
            assert(estimatorIt != estimators_.end());
        }

        // Estimate the goals for the object using the object's GoalEstimator
        auto goals = estimatorIt->second->estimateGoal(obj->motion(), 5.0, environment, nullptr);

        // If the object has been around for long enough, then report it
        if ((obj->updateCount() > 3) && ((environment.timestamp - obj->lastUpdateTime()) < 200000)) {
            // Convert the obj to a full DynamicObject
            auto dynObj = obj->toDynamicObject(*dynObjFactory_);
            // Set the goals to complete the object
            dynObj->setGoals(goals);
            // Add it to the collection of dynamic objects in the environment
            dynObjects.push(dynObj);
        }
    }

    // Erase any invalid estimators -- those not belonging to the current object set
    for (auto& id : inactiveIds) {
        estimators_.erase(id);
    }

    return dynObjects;
}

}   // namespace tracker
}   // namespace vulcan

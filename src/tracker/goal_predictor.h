/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_predictor.h
* \author   Collin Johnson
* 
* Declaration of GoalPredictor and goal_predictor_params_t.
*/

#ifndef TRACKER_GOAL_PREDICTOR_H
#define TRACKER_GOAL_PREDICTOR_H

#include "tracker/dynamic_object_collection.h"
#include "tracker/types.h"
#include <memory>
#include <unordered_map>

namespace vulcan
{
namespace hssh { class LocalPerceptualMap; }
namespace hssh { class LocalTopoMap; }
namespace utils { class ConfigFile; }
namespace tracker 
{
    
class DynamicObjectFactory;
class GoalEstimator;
class GoalEstimatorFactory;
class TrackingObjectSet;
struct tracking_environment_t;
    
/**
* goal_predictor_params_t 
*/
struct goal_predictor_params_t
{
    
    goal_predictor_params_t(const utils::ConfigFile& config);
    goal_predictor_params_t(void) = default;
};
    
/**
* GoalPredictor is responsible for predicting the goal for each DynamicObject in the environment. The predictions
* produce an ObjectGoal for each of the DynamicObjects currently being tracked.
* 
* The tracker only produces goals for Striding or Steady objects. FixedEndpoint and Stationary objects don't have
* goals that are estimated.
* 
* The goal predictions take one of two forms. If there is a LocalTopoMap available, then goals will be estimated by
* considering gateways in the environment. If there isn't a LocalTopoMap, then objects are assumed to move
* ballistically in a straight line for the specified time horizon.
*/
class GoalPredictor
{
public:
    
    /**
    * Constructor for GoalPredictor.
    * 
    * \param    estimatorFactory    Factory for creating GoalEstimators for the tracked objects
    * \param    dynObjFactory       Factory to use for producing the final dynamic objects
    */
    GoalPredictor(std::unique_ptr<GoalEstimatorFactory> estimatorFactory, 
                  std::unique_ptr<DynamicObjectFactory> dynObjFactory);
    
    /**
    * Destructor for GoalPredictor.
    */
    ~GoalPredictor(void);
    
    /**
    * predictGoals predicts goals for the current set of tracking objects in the environment. The objects will then
    * be converted into full DynamicObjects by having an estimated motion and an estimated goal.
    * 
    * \param    objects         Tracked objects in the environment to predict goals for
    * \param    environment     Static environment representation in which the robot is operating
    * \return   The collection of DynamicObjects being tracked in the environment with motion states and goals.
    */
    DynamicObjectCollection predictGoals(const TrackingObjectSet& objects, 
                                         const tracking_environment_t& environment);
    
private:

    std::unique_ptr<GoalEstimatorFactory> estimatorFactory_;
    std::unique_ptr<DynamicObjectFactory> dynObjFactory_;
    std::unordered_map<ObjectId, std::unique_ptr<GoalEstimator>> estimators_;
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_GOAL_PREDICTOR_H

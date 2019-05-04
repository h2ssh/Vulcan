/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     classifier.h
* \author   Collin Johnson
*
* Declaration for ObjectMotionClassifier.
*/

#ifndef TRACKER_MOTIONS_CLASSIFIER_H
#define TRACKER_MOTIONS_CLASSIFIER_H

#include <tracker/types.h>
#include <tracker/object_state.h>
#include <tracker/motions/motion_tracker.h>

namespace vulcan
{
namespace utils { class ConfigFile; }
namespace tracker
{

class FixedEndpointMotion;
class LaserObject;
class ObjectMotion;
class StationaryMotion;
class SteadyMotion;
class StridingMotion;

/**
* object_motion_classifier_params_t contains all parameters relevant to object motion classification.
* 
* The current parameters are:
* 
*   [ObjectMotionClassifierParameters]
*   max_trajectory_duration_ms = maximum duration to store a trajectory for analysis
*/
struct object_motion_classifier_params_t
{
    int64_t maxTrajectoryDurationMs;
    
    object_motion_classifier_params_t(void) = default;
    object_motion_classifier_params_t(const utils::ConfigFile& config);
};

/**
* ObjectMotionClassifier determines the type of motion an object is exhibiting. The classifier looks at the recent
* history of the object's trajectory to decide the motion type. Over the course of the tracking process, the type of
* motion might change.
*/
class ObjectMotionClassifier
{
public:

    /**
    * Constructor for ObjectMotionClassifier.
    *
    * \param    params          Parameters for classification 
    * \param    filterParams    Parameters for use in estimating SteadyMotion
    */
    ObjectMotionClassifier(const object_motion_classifier_params_t& params, 
                           const tracking_filter_params_t&          filterParams);
    
    /**
    * Copy constructor for ObjectMotionClassifier.
    * 
    * \param    rhs             Classifier to be copied
    */
    ObjectMotionClassifier(const ObjectMotionClassifier& rhs);
    
    /**
    * Destructor for ObjectMotionClassifier.
    */
    ~ObjectMotionClassifier(void);

    /**
    * updateClassification updates the classification with a new observation.
    *
    * \param    object          Detected object with which to update the classification
    */
    void updateClassification(const LaserObject& object);

    /**
    * hasClassificationChanged checks if the classification has changed since the last time the model of the object
    * motion was created via the createObjectMotion method.
    */
    bool hasClassificationChanged(void) const;

    /**
    * createObjectMotion creates an instance of ObjectMotion based on the current classification of the object's motion.
    *
    * This method is potentially expensive to compute, so it should be called in conjunction with
    * hasClassificationChanged.
    *
    * \return   Instance of ObjectMotion as determined by the classification algorithm.
    */
    std::unique_ptr<ObjectMotion> createObjectMotion(void);

private:

    enum class MotionType
    {
        striding,
        fixed_endpoint,
        steady,
        stationary,
        unclassified
    };

    const int64_t kMaxTrajectoryDurationUs;

    MotionType type_;
    int64_t    lastUpdateTime_;
    int64_t    startTime_;
    std::size_t maxSize_;
    Position   lastPosition_;
    bool       hasTypeChanged_;
    
    std::unique_ptr<FixedEndpointMotion> fixed_;
    std::unique_ptr<StridingMotion>      striding_;
    std::unique_ptr<SteadyMotion>        steady_;
    std::unique_ptr<StationaryMotion>    stationary_;
    
    tracking_filter_params_t filterParams_;
    
    
    void       initializeModelsIfNeeded(void);
    MotionType determineCurrentMotionType(const LaserObject& object);
    bool       shouldChangeMotionType(MotionType currentType);
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_MOTIONS_CLASSIFIER_H

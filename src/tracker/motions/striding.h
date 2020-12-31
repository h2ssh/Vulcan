/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     striding.h
* \author   Collin Johnson
*
* Declaration of StridingMotion.
*/

#ifndef TRACKER_MOTIONS_STRIDING_H
#define TRACKER_MOTIONS_STRIDING_H

#include "tracker/object_motion.h"
#include "tracker/motions/motion_tracker.h"
#include "tracker/objects/leg.h"
#include "math/smoothed_derivative_sequence.h"
#include "math/uncertain_value.h"
#include <vector>

namespace vulcan
{
namespace tracker
{

class ObjectBoundary;

/**
* StrideState defines the possible states a striding object can be in. All strides are initially undetermined. Once
* enough information is gathered, then they can be in either the stop or swing phase.
*/
enum class StrideState
{
    undetermined,       ///< Not enough information is known about the stride to know the state
    stop,               ///< The object is in the stop phase of its motion
    swing               ///< The object is in the swing phase of its motion
};

/**
* StridingMotion is the motion exhibited by something that walks. A stride has a period and a distance, from which the
* velocity is derived. The striding motion will be in either a stop phase or a swing phase.
*/
class StridingMotion : public ObjectMotion
{
public:

    /**
    * Default constructor for StridingMotion.
    */
    StridingMotion(void);

    /**
    * Constructor for StridingMotion.
    *
    * \param    timestamp       Time initial position was measured
    * \param    position        Initial position of the observed object
    * \param    params          Parameters need for the internal Kalman filter
    */
    StridingMotion(int64_t timestamp, const Position& position, const tracking_filter_params_t& params);

    /**
    * Copy constructor for StridingMotion.
    *
    * \param    lhs
    */
    StridingMotion(const StridingMotion& lhs);

    /**
    * Destructor for StridingMotion.
    */
    virtual ~StridingMotion(void);

    /**
    * direction retrieves the direction the object is moving in radians.
    */
    float direction(void) const;

    /**
    * period retrieves the period of the object's swing phase in seconds.
    */
    float period(void) const;

    /**
    * stride retrieves the estimated length of the object's stride in meters.
    */
    float stride(void) const;
    
    /**
    * width retrieves the estimated width of the striding object.
    */
    float width(void) const;
    
    // ObjectMotion interface
    void accept(ObjectMotionVisitor& visitor) const override;
    std::unique_ptr<ObjectMotion> clone(void) const override;

private:

    using Endpoints     = std::array<Position, 2>;
    using AngleSequence = math::SmoothedDerivativeSequence<double>;

    struct stride_values_t
    {
        std::vector<int64_t>   timestamps;
        AngleSequence          angles;
        std::vector<Endpoints> endpoints;
    };

    math::UncertainValue<> width_;  // width of the object being tracked
    math::UncertainValue<> period_;
    math::UncertainValue<> maxAngle_;
    math::UncertainValue<> strideLength_;

    Position  swingStartPosition_;
    Endpoints lastEndpoints_;
    float     lastEndpointAngle_;
    int       numEndpoints_;

    std::size_t     numDataPoints_;
    stride_values_t strideAngles_;

    LinearMotionTracker motionTracker_;
    
    // ObjectMotion interface
    ObjectMotionStatus modelStatus(void) const override;
    object_motion_state_t updateMotionEstimate(const LaserObject& object) override;
    Position estimateFuturePosition(int deltaTimeMs) const override;

    bool haveEnoughData(void) const;
    bool haveValidModel(void) const;
    
    Endpoints   findCurrentEndpoints(const ObjectBoundary& boundary);
    void        addStrideAngle(int64_t timestamp, const Endpoints& endpoints);
    std::size_t findPhaseEndIndex(void);
    
    void printStrideInfo(void);

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( cereal::base_class<ObjectMotion>(this),
            width_,
            period_,
            maxAngle_,
            strideLength_,
            lastEndpoints_,
            motionTracker_);
    }
};

}
}

#endif // TRACKER_MOTIONS_STRIDING_H


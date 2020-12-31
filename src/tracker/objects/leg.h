/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     leg.h
* \author   Collin Johnson
*
* Declaration of LegModel.
*/

#ifndef TRACKER_OBJECTS_LEG_H
#define TRACKER_OBJECTS_LEG_H

#include "tracker/boundaries/shapes.h"
#include <cereal/access.hpp>
#include <deque>

namespace vulcan
{
namespace tracker
{

/**
* LegState defines the possible states a leg can be in. All legs are initially undetermined. Once enough information is
* gathered, then they can be in either the stance or swing phase.
*/
enum class LegState
{
    undetermined,       ///< Not enough information is known about the leg to know the state
    stance,             ///< The leg is in the stance phase of its motion
    swing               ///< The leg is in the swing phase of its motion
};

/**
* LegModel models the motion of a single leg for a person. A leg is represented using a circle and a simple state
* machine that determines the properties of the leg's motion. A leg can be viewed as being in one of two states, swing
* or stop. The leg's motion switches between the two states. The duration of and distance traveled by the leg define
* its period and stride. These two values can be used to estimate the velocity of the robot.
*
*
*/
class LegModel
{
public:

    /**
    * Default constructor for LegModel.
    */
    LegModel(void);

    /**
    * Constructor for LegModel.
    *
    * \param    boundary            Circle defining the initial position and boundary of the leg
    * \param    measurementTime     Time at which the boundary was found
    */
    LegModel(const Circle& boundary, int64_t measurementTime);

    /**
    * position retrieves the current position of the leg.
    */
    Position position(void) const { return boundary_.center(); }

    /**
    * boundary retrieves the circle boundary of the leg.
    */
    Circle boundary(void) const { return boundary_; }

    /**
    * velocity retrieves the velocity of the leg in meters per second.
    */
    float velocity(void) const { return velocity_; }

    /**
    * direction retrieves the direction the leg is moving in radians.
    */
    float direction(void) const { return direction_; }

    /**
    * period retrieves the period of the leg's swing phase in seconds.
    */
    float period(void) const { return period_; }

    /**
    * stride retrieves the estimated length of the leg's stride in meters.
    */
    float stride(void) const { return stride_; }

    /**
    * isUndetermined checks if the leg model has been determined to be reasonable or not. If it hasn't been determined
    * yet, then more data is needed to decide if it's a leg model. Typically, one stride's worth of data is necessary.
    */
    bool isUndetermined(void) const;
    
    /**
    * isReasonableModel checks if the leg model is reasonable, given the current estimates of stride and period. If the
    * stride is really long or the period is really long or short, then it is unlikely the object being tracked is
    * actually a leg.
    */
    bool isReasonableModel(void) const;

    /**
    * updateModel updates the model of the leg, incorporating a new measured position of the leg.
    *
    * \param    boundary            Measured boundary of the leg
    * \param    measurementTime     Time at which the measurement was taken -- absolute time in microseconds
    */
    void updateModel(const Circle& boundary, int64_t measurementTime);
    
    /**
    * estimateFutureBoundary estimates the boundary of the leg some time in the future.
    * 
    * \param    deltaTimeMs         Number of milliseconds into the future to predict the boundary of the leg
    * \return   Estimated boundary of the leg in the future.
    */
    Circle estimateFutureBoundary(int deltaTimeMs) const;

    /**
    * findMatchScore calculates the score for how well the provided boundary fits the leg model. The score is a
    * combination of the closeness of the new boundary to the estimated leg position at measurementTime and the
    * similarity in the boundary radius with the leg's radius.
    *
    * \param    boundary            Measured boundary of the leg
    * \param    measurementTime     Time at which the measurement was taken -- absolute time in microseconds
    * \return   Match score indicating how well this boundary fits the leg's model. Lower is better.
    */
    float findMatchScore(const Circle& boundary, int64_t measurementTime);

private:

    struct stop_state_t
    {
        int64_t  startTime = 0;
        int64_t  endTime   = 0;
        Position position;
    };

    LegState     state_;
    int64_t      startStartTime_;   ///< Time the current state started
    int64_t      timestamp_;        ///< Time last updated -- microseconds
    Circle       boundary_;
    stop_state_t lastStop_;         ///< State of leg when last stopped
    float        direction_;        ///< radians
    float        velocity_;         ///< m/s
    float        period_;           ///< sec
    float        stride_;           ///< m
    float        currentStride_;    ///< Length (so far) of the current stride

    std::deque<stop_state_t> stopPositions_;     ///< Last N stop positions for the leg


    void updateUndetermined(const Circle& boundary, int64_t measurementTime);
    void updateStop        (const Circle& boundary, int64_t measurementTime);
    void updateSwing       (const Circle& boundary, int64_t measurementTime);

    bool isLegStopped       (const Position& position, const Position& stopPosition);
    void changeToStanceState(const Circle& boundary, int64_t measurementTime);

    void calculateStrideValues(void);
    
    Position runStateFor(const Position& position, int64_t duration, LegState state) const;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( state_,
            timestamp_,
            boundary_,
            direction_,
            velocity_,
            period_,
            stride_);
    }
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_OBJECTS_LEG_H

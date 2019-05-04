/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     leg.cpp
* \author   Collin Johnson
*
* Definition of LegModel.
*/

#include <tracker/objects/leg.h>
#include <utils/timestamp.h>
#include <limits>
#include <cassert>

#define DEBUG_LEG_MODEL

namespace vulcan
{
namespace tracker
{
    
inline LegState switch_leg_state(LegState state)
{
    return (state == LegState::swing) ? LegState::stance : LegState::swing;
}
    

LegModel::LegModel(void)
: state_(LegState::undetermined)
, timestamp_(0)
, direction_(0.0f)
, velocity_(0.0f)
, period_(0.0f)
, stride_(0.0f)
{
}


LegModel::LegModel(const Circle& boundary, int64_t measurementTime)
: LegModel()
{
    timestamp_ = measurementTime;
    boundary_  = boundary;
}


bool LegModel::isUndetermined(void) const
{
    return state_ == LegState::undetermined;
}


bool LegModel::isReasonableModel(void) const
{
    const float kMaxStride = 1.5f;
    const float kMinPeriod = 0.0f;
    const float kMaxPeriod = 2.0f;

    return (state_ == LegState::undetermined) ||
            ((stride_ < kMaxStride) &&
             (period_ > kMinPeriod) &&
             (period_ < kMaxPeriod) &&
             (currentStride_ < kMaxStride));
}


void LegModel::updateModel(const Circle& boundary, int64_t measurementTime)
{
    // If this measurement is the initial update, then set the initial parameters for
    // the leg and be finished
    if(timestamp_ == 0)
    {
        timestamp_ = measurementTime;
        boundary_  = boundary;
        return;
    }

    switch(state_)
    {
    case LegState::undetermined:
        updateUndetermined(boundary, measurementTime);
        break;

    case LegState::stance:
        updateStop(boundary, measurementTime);
        break;

    case LegState::swing:
        updateSwing(boundary, measurementTime);
        break;
    }

    timestamp_ = measurementTime;
    boundary_  = boundary;
}


Circle LegModel::estimateFutureBoundary(int deltaTimeMs) const
{
    // If the leg model is undetermined, then no future prediction can be made
    if(state_ == LegState::undetermined)
    {
        return boundary_;
    }
    
    // The future boundary goes through the periods of motion of the leg
    // Need to finish the current period
    // Then go through N full periods
    // Then partway into a final period
    // Accumulate the motion across all of the swing periods to get the future boundary
    
    int64_t periodUs    = utils::sec_to_usec(period_);
    int64_t deltaTimeUs = deltaTimeMs * 1000;
    
    int64_t timeToPeriodEnd = periodUs - (timestamp_ - startStartTime_);
    deltaTimeUs -= timeToPeriodEnd;
    
    int     numFullPeriods      = deltaTimeUs / periodUs;
    int64_t timeIntoFinalPeriod = deltaTimeUs % periodUs;
    
    auto posAtStateEnd = runStateFor(boundary_.center(), timeToPeriodEnd, state_);
    auto nextState     = switch_leg_state(state_);
    
    for(int n = 0; n < numFullPeriods; ++n)
    {
        posAtStateEnd = runStateFor(posAtStateEnd, periodUs, nextState);
        nextState     = switch_leg_state(nextState);
    }
    
    posAtStateEnd = runStateFor(posAtStateEnd, timeIntoFinalPeriod, nextState);
    
    return Circle(boundary_.radius(), posAtStateEnd);
}


float LegModel::findMatchScore(const Circle& boundary, int64_t measurementTime)
{
    // If timestamp_ == 0, then the leg model has no measurements, so it can't be a good model
    if(timestamp_ == 0)
    {
        return 1000000.0f;
    }

    // Otherwise, the score is just the distance between the boundary center and the leg center.
    // Don't account for estimated velocity of the leg at the moment.
    return distance_between_points(boundary_.center(), boundary.center()) +
            std::abs(boundary_.radius() - boundary.radius());
}


void LegModel::updateUndetermined(const Circle& boundary, int64_t measurementTime)
{
    // To see if a potentially moving leg has stopped, compare against the previous position
    // If the leg hasn't moved far enough, then it has stopped
    if(isLegStopped(boundary.center(), boundary_.center()))
    {
        changeToStanceState(boundary, measurementTime);
    }
    else
    {
        lastStop_.position = boundary.center();
        lastStop_.endTime  = measurementTime;
        state_             = LegState::swing;
    }
}


void LegModel::updateStop(const Circle& boundary, int64_t measurementTime)
{
    // When stopped, check against the lastStop position, so a slowly moving leg will still be recognized
    // as moving.
    if(!isLegStopped(boundary.center(), lastStop_.position))
    {
        state_ = LegState::swing;
        std::cout << "Switched to SWING\n";
    }
    else
    {
        lastStop_.endTime = measurementTime;
    }
}


void LegModel::updateSwing(const Circle& boundary, int64_t measurementTime)
{
    // If the leg has stopped, switch to the stop state. Check against the previous position
    // because the leg is already moving at this point
    if(isLegStopped(boundary.center(), boundary_.center()))
    {
        changeToStanceState(boundary, measurementTime);
        calculateStrideValues();
        std::cout << "Switched to STANCE\n";
    }
    else
    {
        currentStride_ = distance_between_points(boundary.center(), lastStop_.position);
    }
}


bool LegModel::isLegStopped(const Position& position, const Position& stopPosition)
{
    // A leg is stopped if it hasn't moved very far since the last update
    // TODO: This might not be enough. Will maybe need to estimate velocity of the current
    //       swing state to see if the velocity is very different than when it started.
    // NOTE: This approach should be fine for determining when the leg has started moving again though.
    const float kMaxStopDist = 0.1f;
    return distance_between_points(stopPosition, position) < kMaxStopDist;
}


void LegModel::changeToStanceState(const Circle& boundary, int64_t measurementTime)
{
    const std::size_t kMaxStopPositions = 5;

    // If coming from a swing phase, add information about the last stop
    if(state_ == LegState::swing)
    {
        stopPositions_.push_back(lastStop_);
        if(stopPositions_.size() > kMaxStopPositions)
        {
            stopPositions_.pop_front();
        }
    }

    lastStop_.position  = boundary.center();
    lastStop_.startTime = measurementTime;

    state_ = LegState::stance;
}


void LegModel::calculateStrideValues(void)
{
    // No values to calculate if there aren't enough stop positions for determining means and such
    if(stopPositions_.size() < 2)
    {
        return;
    }

    double totalTime  = 0.0;
    double totalXDiff = 0.0;
    double totalYDiff = 0.0;

    for(std::size_t n = 1; n < stopPositions_.size(); ++n)
    {
        totalTime  += utils::usec_to_sec(stopPositions_[n].startTime - stopPositions_[n-1].endTime);
        totalXDiff += stopPositions_[n].position.x - stopPositions_[n-1].position.x;
        totalYDiff += stopPositions_[n].position.y - stopPositions_[n-1].position.y;
    }

    assert(totalTime > 0.0); // the robot doesn't have to move, but time does!

    double totalDist = std::sqrt(totalXDiff*totalXDiff + totalYDiff*totalYDiff);

    direction_ = std::atan2(totalYDiff, totalXDiff);
    velocity_  = totalDist / totalTime;
    period_    = totalTime / stopPositions_.size();
    stride_    = totalDist / stopPositions_.size();

#ifdef DEBUG_LEG_MODEL    
    std::cout << "Leg " << boundary_.center() << ":Dir:" << direction_ << " Vel:" << velocity_ << " Period:" << period_
              << " Stride:" << stride_ << '\n';
#endif
}


Position LegModel::runStateFor(const Position& position, int64_t duration, LegState state) const
{
    // If not swinging or no estimated period, then in the exact same spot
    if((state_ != LegState::swing) || (period_ == 0.0f))
    {
        return position;
    }
    
    float periodPortion = utils::usec_to_sec(duration) / period_;
    float deltaX        = std::cos(direction_) * stride_ * periodPortion;
    float deltaY        = std::sin(direction_) * stride_ * periodPortion;
    
    return Position(position.x + deltaX, position.y + deltaY);
}

} // namespace tracker
} // namespace vulcan

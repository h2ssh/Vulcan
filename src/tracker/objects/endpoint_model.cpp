/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     endpoint_model.cpp
 * \author   Collin Johnson
 *
 * Definition of EndpointModel.
 */

#include "tracker/objects/endpoint_model.h"
#include "math/geometry/shape_fitting.h"
#include "math/regression.h"
#include "math/statistics.h"

// #define DEBUG_MEASUREMENTS
// #define DEBUG_TYPE

namespace vulcan
{
namespace tracker
{

const std::size_t kMinModelData = 10;


EndpointModel::EndpointModel(double maxRadius)
: haveNewMeasurements_(false)
, type_(EndpointType::undetermined)
, pointError_(0.0)
, lineError_(0.0)
, circleError_(0.0)
, kMaxRadius_(maxRadius)
{
}


EndpointModel::EndpointModel(Position position, double maxRadius)
: haveNewMeasurements_(false)
, type_(EndpointType::undetermined)
, currentPosition_(position)
, pointError_(0.0)
, lineError_(0.0)
, circleError_(0.0)
, kMaxRadius_(maxRadius)
{
    addPositionMeasurement(position);
    std::fill(positionExtrema_.begin(), positionExtrema_.end(), position);
}


EndpointType EndpointModel::type(void) const
{
    updateModelIfNeeded();
    return type_;
}


Position EndpointModel::position(void) const
{
    updateModelIfNeeded();

    // If the endpoint is stationary, then the position is the mean of all measurements.
    if (type_ == EndpointType::stationary) {
        return pointEstimate_;
    }
    // Otherwise, the endpoint is currently located at the last of the measured positions.
    else {
        return currentPosition_;
    }
}


math::angle_range_t EndpointModel::angleRange(Position origin) const
{
    math::angle_range_t extremaRange(angle_to_point(origin, positionExtrema_[0]));
    extremaRange.expand(angle_to_point(origin, positionExtrema_[1]));
    return extremaRange;
}


Arc EndpointModel::estimatedArc(void) const
{
    updateModelIfNeeded();
    return Arc(circleEstimate_.radius(), circleEstimate_.center(), angleRange(circleEstimate_.center()));
}


float EndpointModel::distanceRange(void) const
{
    return distance_between_points(positionExtrema_[0], positionExtrema_[1]);
}


Line<float> EndpointModel::estimatedSegment(void) const
{
    updateModelIfNeeded();
    return lineEstimate_;
}


void EndpointModel::addPositionMeasurement(Position position)
{
    if (changeExtremaIfNeeded(position) || measurements_.empty()) {
        xStats_(position.x);
        yStats_(position.y);
        measurements_.push_back(position);
        haveNewMeasurements_ = true;

#ifdef DEBUG_MEASUREMENTS
        std::cout << "DEBUG:EndpointModel: Measurements:\n";
        std::copy(measurements_.begin(), measurements_.end(), std::ostream_iterator<Position>(std::cout, "\n"));
#endif
    }

    currentPosition_ = position;
}


std::array<float, 2> EndpointModel::calculateAssociationScores(const Endpoints& measurements) const
{
    // Three different ways to calculate the association score:
    //
    //  - undetermined/sliding - distance from last position
    //  - stationary           - distance from mean position
    //  - rotating             - distance from arc boundary

    std::array<float, 2> scores;

    if (type_ == EndpointType::rotating) {
        scores[0] =
          std::abs(distance_between_points(circleEstimate_.center(), measurements_[0]) - circleEstimate_.radius());
        scores[1] =
          std::abs(distance_between_points(circleEstimate_.center(), measurements_[1]) - circleEstimate_.radius());
    }
    // position returns last measurement for sliding, but the mean position for stationary
    // so it can be used for both cases
    else {
        scores[0] = distance_between_points(measurements[0], position());
        scores[1] = distance_between_points(measurements[1], position());
    }

    return scores;
}


bool EndpointModel::changeExtremaIfNeeded(Position position)
{
    // If the distance from the new position to one of the existing extrema is greater than the distance between
    // the extrema, then an extrema needs to be changed

    float posToZero = distance_between_points(position, positionExtrema_[0]);
    float posToOne = distance_between_points(position, positionExtrema_[1]);
    float zeroToOne = distance_between_points(positionExtrema_[0], positionExtrema_[1]);

    bool isExtrema = false;

    if (posToZero > zeroToOne) {
        positionExtrema_[1] = position;
        isExtrema = true;
    } else if (posToOne > zeroToOne) {
        positionExtrema_[0] = position;
        isExtrema = true;
    }

    return isExtrema;
}


void EndpointModel::updateModelIfNeeded(void) const
{
    using namespace boost::accumulators;

    const double kMinVariance = 1e-4;

    if (haveNewMeasurements_) {
        // If enough data exists to estimate the different types of endpoints
        if (measurements_.size() >= kMinModelData) {
            // And there is enough variation in the data to get a meaningful estimate
            if ((variance(xStats_) > kMinVariance) || (variance(yStats_) > kMinVariance)) {
                // Estimate the three types of endpoint
                estimatePoint();
                estimateLine();
                estimateArc();

                // And decide which is the best fit for the current data
                determineType();
            }
            // Otherwise the endpoint must be stationary because there's no variation in the data
            else {
                estimatePoint();
                type_ = EndpointType::stationary;
            }
        }
        // Otherwise the endpoint type can't be determined yet
        else {
            type_ = EndpointType::undetermined;
        }

        haveNewMeasurements_ = false;
    }
}


void EndpointModel::estimatePoint(void) const
{
    using namespace boost::accumulators;

    pointEstimate_ = Position(mean(xStats_), mean(yStats_));
    //     pointError_ = std::accumulate(measurements_.begin(), measurements_.end(), 0.0,
    //                                   [this](double total, const Point<float>& p)
    //                                   {
    //                                       return total + distance_between_points(p, pointEstimate_);
    //                                   });
    pointError_ = variance(xStats_) + variance(yStats_);
}


void EndpointModel::estimateLine(void) const
{
    assert(measurements_.size() > 2);

    lineEstimate_ = math::total_least_squares(measurements_.begin(), measurements_.end());
    lineError_ =
      std::accumulate(measurements_.begin(), measurements_.end(), 0.0, [this](double total, const Point<float>& p) {
          return total + distance_to_line_segment(p, lineEstimate_);
      });
}


void EndpointModel::estimateArc(void) const
{
    const double kMinRadius = 0.05;
    const double kMaxRadius = 1.5;

    assert(measurements_.size() > 2);

    circleEstimate_ =
      math::minimum_geometric_error_circle(measurements_.begin(), measurements_.end(), kMinRadius, kMaxRadius);
    circleError_ =
      std::accumulate(measurements_.begin(), measurements_.end(), 0.0, [this](double total, const Point<float>& p) {
          return total + std::abs(distance_between_points(p, circleEstimate_.center()) - circleEstimate_.radius());
      });
}


void EndpointModel::determineType(void) const
{
    // The type is the one with the least error fitting the points
    // The arc also can't have too large a radius because a large radius arc will fit a line very very well
    if (distanceRange() < kMaxStationaryExtremaDistance) {
        type_ = EndpointType::stationary;
    } else if ((circleEstimate_.radius() < kMaxRadius_) && (circleError_ < lineError_)) {
        type_ = EndpointType::rotating;
    } else if ((length(lineEstimate_) < 2.0 * kMaxRadius_) && (lineError_ < circleError_)) {
        type_ = EndpointType::sliding;
    } else {
        type_ = EndpointType::invalid;
    }

#ifdef DEBUG_TYPE
    std::cout << "DEBUG: EndpointModel::determineType: \n"
              << " Position: " << currentPosition_ << '\n'
              << " arc:      " << arcEstimate_ << '\n'
              << " arcError: " << arcError_ << '\n'
              << " line:     " << lineEstimate_ << '\n'
              << " lineError:" << lineError_ << '\n'
              << " meanError:" << pointError_ << '\n'
              << " Dist:     " << distanceRange() << '\n'
              << " Type:     " << type_ << '\n';
#endif
}


std::ostream& operator<<(std::ostream& out, EndpointType type)
{
    switch (type) {
    case EndpointType::undetermined:
        out << "undetermined";
        break;

    case EndpointType::stationary:
        out << "stationary";
        break;

    case EndpointType::sliding:
        out << "sliding";
        break;

    case EndpointType::rotating:
        out << "rotating";
        break;

    case EndpointType::invalid:
        out << "invalid";
        break;
    }

    return out;
}

}   // namespace tracker
}   // namespace vulcan

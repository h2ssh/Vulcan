/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     endpoint_model.h
 * \author   Collin Johnson
 *
 * Declaration of EndpointModel.
 */

#ifndef TRACKER_MOTIONS_ENDPOINT_MODEL_H
#define TRACKER_MOTIONS_ENDPOINT_MODEL_H

#include "math/angle_range.h"
#include "tracker/boundaries/shapes.h"
#include "tracker/types.h"
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <cereal/access.hpp>
#include <cereal/types/array.hpp>

namespace vulcan
{
namespace tracker
{

/**
 * EndpointType defines the possible type determined for the endpoint based on the input data.
 */
enum class EndpointType
{
    undetermined,   ///< Not enough data exists to determine the type yet
    stationary,     ///< The endpoint isn't moving anywhere
    sliding,        ///< The endpoint is sliding along
    rotating,       ///< The endpoint is rotating around some fixed position
    invalid         ///< The endpoint represent a fixed endpoint or doesn't match motion around a fixed endpoint
};

// Output helper for EndpointType
std::ostream& operator<<(std::ostream& out, EndpointType type);

/**
 * EndpointModel models an endpoint for a fixed endpoint object. An endpoint has one of four types:
 *
 *   - undetermined : haven't yet received enough data yet to decide the type
 *   - stationary   : all measured points are within some small distance of one another
 *   - sliding      : the motion of the endpoint is best described by a line
 *   - rotating     : the motion of the endpoint is best described by an arc
 */
class EndpointModel
{
public:
    /// Maximum distanceRange() for a stationary point. Any endpoint with a distanceRange() less than this value
    /// will be considered stationary, once enough data have been assigned.
    constexpr static float kMaxStationaryExtremaDistance = 0.15f;

    /**
     * Default constructor for EndpointModel.
     *
     * \param    maxRadius           Maximum radius of the arc the endpoint can move along (optiona, default=1.5m)
     */
    explicit EndpointModel(double maxRadius = 1.5);

    /**
     * Constructor for EndpointModel.
     *
     * \param    position            Initial position of the endpoint
     * \param    maxRadius           Maximum radius of the arc the endpoint can move along (optional, default = 1.5m)
     */
    explicit EndpointModel(Position position, double maxRadius = 1.5);

    /**
     * type retrieves the current estimated type of the EndpointModel.
     */
    EndpointType type(void) const;

    /**
     * position retrieves the the position of the endpoint. For a stationary endpoint, the position is the mean
     * estimated position. For a moving endpoint, it is the most recently detected position of the endpoint.
     */
    Position position(void) const;

    /**
     * angleRange finds the range of angles the endpoint has passed through relative to some origin position.
     * The range starts at the .first and moves counter-clockwise to .second.
     *
     * \param    origin          Origin from which to measure the range
     * \return   Range of angles covered by the endpoint.
     */
    math::angle_range_t angleRange(Position origin) const;

    /**
     * estimatedArc finds the estimated arc along which the endpoint is moving.
     */
    Arc estimatedArc(void) const;

    /**
     * distanceRange retrieves the maximum distance detected between two measurements of the endpoint.
     */
    float distanceRange(void) const;

    /**
     * estimatedSegment retrieves the estimated line segment along which the endpoint moves.
     */
    Line<float> estimatedSegment(void) const;

    /**
     * addPositionMeasurement adds a new measurement of the endpoint position.
     *
     * \param    position        Measured position of the endpoint
     */
    void addPositionMeasurement(Position position);

    /**
     * calculateAssociationScores calculates a score for each measurement. Lower scores indicate better matches. The
     * best score is 0.
     *
     * \param    measurements            Measurement to select amongst
     * \return   Pair of selected index, association score.
     */
    std::array<float, 2> calculateAssociationScores(const Endpoints& measurements) const;

private:
    using Stats = boost::accumulators::accumulator_set<
      float,
      boost::accumulators::features<boost::accumulators::tag::mean, boost::accumulators::tag::variance>>;

    Stats xStats_;
    Stats yStats_;

    std::array<Position, 2> positionExtrema_;
    std::vector<Position> measurements_;
    mutable bool haveNewMeasurements_;

    mutable EndpointType type_;
    Position currentPosition_;

    mutable Position pointEstimate_;
    mutable Line<float> lineEstimate_;
    mutable Circle circleEstimate_;

    mutable double pointError_;
    mutable double lineError_;
    mutable double circleError_;

    double kMaxRadius_;

    bool changeExtremaIfNeeded(Position position);
    void updateModelIfNeeded(void) const;
    void estimatePoint(void) const;
    void estimateLine(void) const;
    void estimateArc(void) const;
    void determineType(void) const;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        // Don't worry about the measurements or accumulator things, as they are simply internally cached bits
        // for determining endpoint type and aren't useful outside the object_tracker
        ar(type_, positionExtrema_, currentPosition_, pointEstimate_, lineEstimate_, circleEstimate_);
    }
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_MOTIONS_ENDPOINT_MODEL_H

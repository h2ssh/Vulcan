/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     fixed_object_model.h
 * \author   Collin Johnson
 *
 * Declaration of FixedObjectModel.
 */

#ifndef TRACKER_OBJECTS_FIXED_OBJECT_MODEL_H
#define TRACKER_OBJECTS_FIXED_OBJECT_MODEL_H

#include "math/uncertain_value.h"
#include "tracker/object_boundary.h"
#include "tracker/objects/endpoint_model.h"
#include <boost/optional.hpp>
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace tracker
{

/**
 * FixedObjectModel is a model of an object that has a limited range of motion because some part of it is fixed to the
 * environment. For the most part, these objects are doors.
 *
 * The fixed object model determines whether an object is exhibiting one of three motions:
 *
 *   - stationary : while marked as static, the object doesn't seem to be moving
 *   - pivoting   : the object rotating around some fixed point
 *   - sliding    : the object is sliding along some track, it gets longer or shorter
 *
 * Until enough data is available, an object will be of undetermined type.
 *
 * The model can also be invalid. If it is determined that none of the above categories defines the object's motion,
 * then the model will be marked as invalid.
 *
 * The model of the object is updated via the updateModel method.
 */
class FixedObjectModel
{
public:
    /**
     * Default constructor for FixedObjectModel.
     */
    FixedObjectModel(void);

    /**
     * Constructor for FixedObjectModel.
     *
     * \param    boundary            Initial detected boundary of the object
     */
    FixedObjectModel(const ObjectBoundary& boundary);

    /**
     * updateModel updates the model of the object using information from the new boundary measurement.
     *
     * \param    boundary            Measured boundary to incorporate into the object
     */
    void updateModel(const ObjectBoundary& boundary);

    /**
     * boundary retrieves the current boundary of the object.
     */
    ObjectBoundary boundary(void) const;

    /**
     * position retrieves the current position of the object.
     */
    Position position(void) const { return position_; }

    /**
     * fixedPosition retrieves the position of where the object is fixed to the environment.
     */
    Position fixedPosition(void) const;

    /**
     * movingPosition retrieves the position of the moving endpoint.
     */
    Position movingPosition(void) const;

    /**
     * movingExtrema finds the extrema of the moving endpoint. The endpoint moves from first to second through the
     * positions in-between. For a rotating object, the motion is counter-clockwise. For a sliding object, it is linear.
     */
    std::pair<Position, Position> movingExtrema(void) const { return movingExtrema_; }

    /**
     * estimatedArc retrieves the arc along which the endpoint is estimated to be moving.
     */
    Arc estimatedArc(void) const;

    /**
     * angleRange retrieves the angle range through which the object might be moving.
     */
    math::angle_range_t angleRange(void) const;

    /**
     * estimatedLine is the estimated line along which the object is moving.
     */
    Line<float> estimatedSegment(void) const;

    /**
     * isValid checks if the model is a valid representation of a fixed object in the environment.
     */
    bool isValid(void) const;

    /**
     * isUndetermined checks if the model's type has been determined or not.
     */
    bool isUndetermined(void) const;

    /**
     * isStationary checks if the model is of a stationary object.
     */
    bool isStationary(void) const;

    /**
     * isPivoting checks if the model is of a pivoting object.
     */
    bool isPivoting(void) const;

    /**
     * isSliding checks if the model is of a sliding object.
     */
    bool isSliding(void) const;

private:
    enum class Type
    {
        undetermined,
        invalid,
        stationary,
        sliding,
        pivoting
    };

    Type type_;
    std::vector<EndpointModel> endpoints_;   // At most 2
    math::UncertainValue<> width_;           // Measured width of the object
    ObjectBoundary boundary_;
    Position position_;
    std::pair<Position, Position> movingExtrema_;
    int movingIndex_;
    Position fixedPosition_;

    bool updateEndpoints(const ObjectBoundary& boundary);
    void determineType(void);
    void setPivotingEndpoints(void);
    void setSlidingEndpoints(void);
    ObjectBoundary createBoundaryFromEndpoints(void) const;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(type_, endpoints_, width_, boundary_, position_, movingExtrema_, movingIndex_, fixedPosition_);
    }
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_OBJECTS_FIXED_OBJECT_MODEL_H

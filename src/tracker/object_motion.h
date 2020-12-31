/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     object_motion.h
 * \author   Collin Johnson
 *
 * Declaration of ObjectMotion interface.
 */

#ifndef TRACKER_OBJECT_MOTION_H
#define TRACKER_OBJECT_MOTION_H

#include "core/point.h"
#include "core/pose.h"
#include "tracker/object_state.h"
#include "tracker/types.h"
#include <cereal/access.hpp>
#include <cereal/types/deque.hpp>

namespace vulcan
{
namespace tracker
{

class LaserObject;
class ObjectMotionVisitor;

/**
 * ObjectMotionStatus defines the possible states of an ObjectMotion estimate.
 */
enum class ObjectMotionStatus
{
    undetermined,
    invalid,
    valid
};

// Output operator for ObjectMotionStatus
std::ostream& operator<<(std::ostream& out, ObjectMotionStatus status);

/**
 * ObjectMotion is an abstract base class describing the motion of a tracked object. An object's motion consists of the
 * following:
 *
 *   - a current position
 *   - a current velocity
 *   - a recent trajectory
 *
 * An ObjectMotion instance can also:
 *
 *   - estimate the position of the object in the future
 *   - estimate the trajectory the object will take for a span of time
 */
class ObjectMotion
{
public:
    using PositionConstIter = std::deque<Position>::const_iterator;

    // Use default copy and move
    ObjectMotion(const ObjectMotion& rhs) = default;
    ObjectMotion(ObjectMotion&& rhs) = default;

    virtual ~ObjectMotion(void) { }

    /**
     * timestamp time associated with this particular estimator of the current motion.
     */
    int64_t timestamp(void) const { return timestamp_; }

    /**
     * position retrieves the current position estimate of the object. This value is the position portion of the full
     * motion state estimate.
     */
    Position position(void) const { return Position(motion_.x, motion_.y); }

    /**
     * velocity retrieves the current velocity estimate of the object. This value is the velocity porition of the full
     * motion state estimate.
     */
    velocity_t velocity(void) const { return velocity_t(motion_.xVel, motion_.yVel); }

    /**
     * motion retrieves the currently estimate motion state of the object.
     */
    object_motion_state_t motion(void) const { return motion_; }

    /**
     * estimatePositionAt estimates the future position of the object relative to the most recent measured position.
     *
     * \param    deltaTimeMs         Milliseconds in the future to estimate object position
     * \pre      deltaTimeMs >= 0
     * \return   Estimated object position at deltaTimeMs milliseconds since the last measured time.
     */
    Position estimatePositionAt(int deltaTimeMs) const;

    /**
     * estimateTrajectoryFor estimates the future trajectory of the object for some duration of time. The offset time
     * determines when the trajectory starts.
     *
     * \param    durationMs          Milliseconds for which to estimate the trajectory
     * \param    stepMs              Milliseconds per timestep
     * \param    startOffsetMs       Milliseconds in the future to start the trajectory estimate
     * \pre      durationMs >= 0
     * \pre      0 < stepMs <= durationMs
     * \pre      startOffsetMs >= 0
     * \return   Estimated trajectory of the object represented as a sequence of positions.
     */
    std::vector<Position> estimateTrajectoryFor(int durationMs, int stepMs, int startOffsetMs) const;

    /**
     * updateModel updates the model of the motion using a LaserObject. The LaserObject was detected in the latest scan.
     *
     * \param    detectedObject          Object detected in the current laser scan
     * \return   Status of the model.
     */
    ObjectMotionStatus updateModel(const LaserObject& detectedObject);

    // Iteration support for the recently measured trajectory of the object
    PositionConstIter beginRecentTraj(void) const { return trajectory_.cbegin(); }
    PositionConstIter endRecentTraj(void) const { return trajectory_.cend(); }

    /**
     * accept accepts a visitor to the instance of ObjectMotion.
     *
     * \param    visitor         Visitor to accept
     */
    virtual void accept(ObjectMotionVisitor& visitor) const = 0;

    /**
     * clone creates a copy of the instance of ObjectMotion.
     */
    virtual std::unique_ptr<ObjectMotion> clone(void) const = 0;

protected:
    /**
     * Default constructor for ObjectMotion.
     */
    ObjectMotion(void);

    /**
     * Constructor for ObjectMotion.
     *
     * \param    position        Create object at the given position
     * \param    velocity        Create an object with the given velocity
     */
    ObjectMotion(Position position, velocity_t velocity);

    /**
     * modelStatus determines the status of the model of the motion. Is the model valid? Is it still up-in-the-air as
     * to whether or not this is a valid estimate? Is the model flat-out wrong?
     */
    virtual ObjectMotionStatus modelStatus(void) const = 0;

    /**
     * updateMotionEstimate updates the estimated position and the velocity by incorporating a new measurement.
     *
     * \param    object          Object detected in current scan
     * \return   Updated estimate of position.
     */
    virtual object_motion_state_t updateMotionEstimate(const LaserObject& object) = 0;

    /**
     * estimateFuturePosition estimates the position of the object some time in the future. The time is relative to the
     * current position of the object.
     *
     * \param    deltaTimeMs         Time in the future to estimate the position
     * \return   Position of the object in the future.
     */
    virtual Position estimateFuturePosition(int deltaTimeMs) const = 0;

    /**
     * estimateFutureTrajectory estimates the future trajectory of the object for some duration of time. The offset time
     * determines when the trajectory starts.
     *
     * The default implementation calls estimateFuturePosition for each time step. If more sophisticated behavior is
     * needed, then the method can be overwritten.
     *
     * \param    numSteps            Number of time steps in the trajectory
     * \param    stepMs              Milliseconds per timestep
     * \param    startOffsetMs       Milliseconds in the future to start the trajectory estimate
     * \return   Estimated trajectory of the object represented as a sequence of positions.
     */
    virtual std::vector<Position> estimateFutureTrajectory(int numSteps, int stepMs, int startOffsetMs) const;

private:
    int64_t timestamp_ = 0;
    object_motion_state_t motion_;
    std::deque<Position> trajectory_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(timestamp_, motion_, trajectory_);
    }
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_OBJECT_MOTION_H

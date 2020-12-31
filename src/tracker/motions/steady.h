/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     steady.h
 * \author   Collin Johnson
 *
 * Declaration of SteadyMotion.
 */

#ifndef TRACKER_MOTIONS_STEADY_H
#define TRACKER_MOTIONS_STEADY_H

#include "core/multivariate_gaussian.h"
#include "tracker/motions/motion_tracker.h"
#include "tracker/object_motion.h"

namespace vulcan
{
namespace tracker
{

/**
 * SteadyMotion represents an object that moves with a steady, but potentially changing velocity. The position of the
 * object is tracked using a Kalman filter. Motion is assumed to be independent on the x- and y-axes. The velocity of
 * the object is estimated using a regression that takes into account the position and time at which a measurement was
 * made.
 */
class SteadyMotion : public ObjectMotion
{
public:
    /**
     * Default constructor for SteadyMotion.
     */
    SteadyMotion(void);

    /**
     * Constructor for SteadyMotion.
     *
     * \param    timestamp               Timestamp of the initial position
     * \param    initialPosition         Initial position of the object
     * \param    filterParams            Parameters for the Kalman filter for tracking the velocity
     */
    SteadyMotion(int64_t timestamp, Position initialPosition, const tracking_filter_params_t& filterParams);

    /**
     * Destructor for SteadyMotion.
     */
    virtual ~SteadyMotion(void);

    /**
     * slowMotionState retrieves the full distribution of the object state trusting the process model.
     */
    MultivariateGaussian slowMotionState(void) const { return motionTracker_.slowStateWithUncertainty(); }

    /**
     * fastMotionState retrieves the full distribution of the object state with high process uncertainty.
     */
    MultivariateGaussian fastMotionState(void) const { return motionTracker_.fastStateWithUncertainty(); }

    // ObjectMotion interface
    void accept(ObjectMotionVisitor& visitor) const override;
    std::unique_ptr<ObjectMotion> clone(void) const override;

private:
    LinearMotionTracker motionTracker_;

    // ObjectMotion interface
    ObjectMotionStatus modelStatus(void) const override;
    object_motion_state_t updateMotionEstimate(const LaserObject& object) override;
    Position estimateFuturePosition(int deltaTimeMs) const override;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::base_class<ObjectMotion>(this), motionTracker_);
    }
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_MOTIONS_STEADY_H

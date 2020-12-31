/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     fixed_endpoint.h
 * \author   Collin Johnson
 *
 * Declaration of FixedEndpointMotion.
 */

#ifndef TRACKER_MOTIONS_FIXED_ENDPOINT_MOTION_H
#define TRACKER_MOTIONS_FIXED_ENDPOINT_MOTION_H

#include "tracker/object_motion.h"
#include "tracker/objects/fixed_object_model.h"

namespace vulcan
{
namespace tracker
{

/**
 * FixedEndpointMotion
 */
class FixedEndpointMotion : public ObjectMotion
{
public:
    /**
     * Constructor for FixedEndpointMotion.
     */
    FixedEndpointMotion(void);

    /**
     * model retrieves the model of the fixed object used to determine its motion.
     */
    const FixedObjectModel& model(void) const { return model_; }

    /**
     * isPivoting checks if the motion of the object is pivoting.
     */
    bool isPivoting(void) const;

    /**
     * isSliding checks if the motion of the object is sliding.
     */
    bool isSliding(void) const;

    // ObjectMotion interface
    void accept(ObjectMotionVisitor& visitor) const override;
    std::unique_ptr<ObjectMotion> clone(void) const override;

private:
    FixedObjectModel model_;

    // ObjectMotion interface
    ObjectMotionStatus modelStatus(void) const override;
    object_motion_state_t updateMotionEstimate(const LaserObject& object) override;
    Position estimateFuturePosition(int deltaTimeMs) const override;
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_MOTIONS_FIXED_ENDPOINT_MOTION_H

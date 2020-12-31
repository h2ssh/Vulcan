/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "mpepc/metric_planner/progress_checker.h"

namespace vulcan
{
namespace mpepc
{

void ProgressChecker::reset(void)
{
    progressCounter_ = 0;
    robotIsStuckCounter_ = 0;
}


void ProgressChecker::run(const motion_state_t& state)
{
    const float kSlowDecayRate = 0.9;
    const float kFastDecayRate = 0.5;

    const int32_t kProgressCounterThreshold = 10;

    const float kLinVelThreshold = 0.1;
    const float kAngVelThreshold = 0.2;

    if(progressCounter_ == 0)
    {
        slowAverage_ = state.velocity;
        fastAverage_ = state.velocity;
    }
    else
    {
        slowAverage_.linear  = kSlowDecayRate*slowAverage_.linear  + (1 - kSlowDecayRate)*state.velocity.linear;
        slowAverage_.angular = kSlowDecayRate*slowAverage_.angular + (1 - kSlowDecayRate)*state.velocity.angular;

        fastAverage_.linear  = kFastDecayRate*fastAverage_.linear  + (1 - kFastDecayRate)*state.velocity.linear;
        fastAverage_.angular = kFastDecayRate*fastAverage_.angular + (1 - kFastDecayRate)*state.velocity.angular;
    }

    //is robot not moving (forward/backward)?
    bool robotIsNotMoving = fabs(slowAverage_.linear)   < kLinVelThreshold &&
                            fabs(fastAverage_.linear)   < kLinVelThreshold &&
                            fabs(state.velocity.linear) < kLinVelThreshold;

    if(robotIsNotMoving)
    {
        // is robot not rotating?
        bool robotIsNotRotating = fabs(slowAverage_.angular)   < kAngVelThreshold &&
                                  fabs(fastAverage_.angular)   < kAngVelThreshold &&
                                  fabs(state.velocity.angular) < kAngVelThreshold;

        // robot may be chattering (not being able to decide to turn right or left, if the long-term
        // average angular velocity is small, short-term angular delta is not small,
        // but the robot is not moving forward or backward.
        bool robotIsChattering = robotIsNotMoving &&
                                 fabs(slowAverage_.angular) < kAngVelThreshold &&
                                 fabs(fastAverage_.angular - state.velocity.angular) > kAngVelThreshold;

        if(robotIsNotRotating || robotIsChattering)
        {
            // if the robot is not moving, and is not rotating or is chattering around an angle,
            // then declare the robot is stuck somewhere.
            robotIsStuckCounter_++;
        }
    }
    else
    {
        // if robot is moving update counters
        progressCounter_++;

        if(progressCounter_ > kProgressCounterThreshold)
        {
            progressCounter_     = kProgressCounterThreshold;
            robotIsStuckCounter_ = 0;
        }
    }

}

} // namespace mpepc
} // namespace vulcan

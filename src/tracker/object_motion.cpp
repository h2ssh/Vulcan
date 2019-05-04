/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object_motion.cpp
* \author   Collin Johnson
*
* Definition of ObjectMotion.
*/

#include <tracker/object_motion.h>
#include <tracker/laser_object.h>
#include <iostream>
#include <cassert>

namespace vulcan
{
namespace tracker
{

std::ostream& operator<<(std::ostream& out, ObjectMotionStatus status)
{
    switch(status)
    {
    case ObjectMotionStatus::invalid:
        out << "invalid";
        break;

    case ObjectMotionStatus::valid:
        out << "valid";
        break;

    case ObjectMotionStatus::undetermined:
        out << "undetermined";
        break;
    }

    return out;
}


ObjectMotion::ObjectMotion(void)
{
}


ObjectMotion::ObjectMotion(Position position, velocity_t velocity)
: motion_(position.x, position.y, velocity.x, velocity.y)
{
}

    
Position ObjectMotion::estimatePositionAt(int deltaTimeMs) const
{
    // Enforce the preconditions
    // Calculate the position at the appropriate time
    assert(deltaTimeMs >= 0);
    
    return estimateFuturePosition(deltaTimeMs);
}


std::vector<Position> ObjectMotion::estimateTrajectoryFor(int durationMs, 
                                                            int stepMs, 
                                                            int startOffsetMs) const
{
    // Enforce the preconditions
    // Calculate the number of number of steps 
    // Calculate the position for each step
    assert(durationMs >= stepMs);
    assert(stepMs > 0);
    assert(startOffsetMs >= 0);
    assert(stepMs <= durationMs);
    
    int numSteps = std::max(durationMs / stepMs, 1);
    return estimateFutureTrajectory(numSteps, stepMs, startOffsetMs);
}


ObjectMotionStatus ObjectMotion::updateModel(const LaserObject& detectedObject)
{
    // Update the position and velocity estimate
    timestamp_ = detectedObject.timestamp();
    motion_ = updateMotionEstimate(detectedObject);
    /*
    // Save the new position into the recent trajectory
    trajectory_.push_back(position());

    if(trajectory_.size() > 500)
    {
        trajectory_.pop_front();
    }*/
    
    return modelStatus();
}


std::vector<Position> ObjectMotion::estimateFutureTrajectory(int numSteps, 
                                                             int stepMs,
                                                             int startOffsetMs) const
{
    std::vector<Position> trajectory;
    trajectory.reserve(numSteps);
    
    for(int n = 0; n < numSteps; ++n)
    {
        trajectory.emplace_back(estimateFuturePosition(startOffsetMs + stepMs*n));
    }
    
    return trajectory;
}

} // namespace tracker
} // namespace vulcan

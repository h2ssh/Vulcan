/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     pivoting_object.cpp
 * \author   Collin Johnson
 *
 * Definition of PivotingObject.
 */

#include "tracker/objects/pivoting_object.h"
#include "tracker/dynamic_object_visitor.h"
#include "tracker/object_state.h"
#include "utils/stub.h"
#include <boost/range/iterator_range.hpp>

namespace vulcan
{
namespace tracker
{

PivotingObject::PivotingObject(ObjectId id, int64_t timestamp, const FixedObjectModel& model)
: FixedObject(timestamp, model)
, id_(id)
{
}


Arc PivotingObject::arc(void) const
{
    return model().estimatedArc();
}


std::unique_ptr<DynamicObject> PivotingObject::clone(void) const
{
    return std::unique_ptr<DynamicObject>(new PivotingObject(*this));
}


void PivotingObject::accept(DynamicObjectVisitor& visitor) const
{
    visitor.visitPivotingObject(*this);
}


velocity_t PivotingObject::estimateVelocity(int64_t timestamp) const
{
    PRINT_STUB("PivotingObject::estimateVelocity");

    return velocity_t(0.0f, 0.0f);

    //     const int64_t kVelocityTimeWindowUs = 200000;
    //
    //     // Find where the first saved state is within the window for which to estimate the velocity
    //     auto windowBeginIt = std::find_if(beginRecentTrajectory(), endRecentTrajectory(),
    //                                       [timestamp](const object_state_t& state)
    //                                       {
    //                                           return timestamp - kVelocityTimeWindowUs < state.timestamp;
    //                                       });
    //
    //     // If no states exist yet, then no velocity
    //     if(windowBeginIt == endRecentTrajectory())
    //     {
    //         std::cout << "Didn't detect a state before " << (timestamp - kVelocityTimeWindowUs) << '\n';
    //         return velocity_t(0.0f, 0.0f);
    //     }
    //
    //     // Calculate the total angle subtended during the time window
    //     int64_t totalTime  = 0;
    //     double  totalAngle = 0.0;
    //
    //     auto fixedPosition = model().fixedPosition();
    //     auto prevAngle     = angle_to_point(fixedPosition, Position(windowBeginIt->x, windowBeginIt->y));
    //     auto prevTime      = state_.timestamp;
    //
    //     for(auto& state : boost::make_iterator_range(windowBeginIt+1, endRecentTrajectory()))
    //     {
    //         auto angle = angle_to_point(fixedPosition, Position(state.x, state.y));
    //
    //         totalTime  += state.timestamp - prevTime;
    //         totalAngle += angle_diff(angle, prevAngle);
    //
    //         prevAngle = angle;
    //         prevTime  = state.timestamp;
    //     }
    //
    //     std::cout << "Totals: Time:" << totalTime << " Angle:" << totalAngle << " Final angle:" << prevAngle << '\n';
    //
    //     // If no time has progressed, then not moving
    //     if(totalTime == 0)
    //     {
    //         return velocity_t(0.0f, 0.0f);
    //     }
    //
    //     // Otherwise, the total angle moved over time is the magnitude of the velocity.
    //     // The direction of the velocity points tangent to the current angle of the object, which is stored in
    //     prevAngle
    //     // The sign of totalAngle determines the direction of the velocity
    //     // v = angVel * radius
    //     double magnitude = (totalAngle / (totalTime / 1000000.0)) * model().estimatedArc().radius();
    //     double direction = prevAngle + std::copysign(M_PI_2, totalAngle);
    //
    //     return velocity_t(std::cos(direction) * magnitude, std::sin(direction) * magnitude);
}

}   // namespace tracker
}   // namespace vulcan

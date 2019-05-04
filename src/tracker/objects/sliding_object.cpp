/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     sliding_object.cpp
* \author   Collin Johnson
* 
* Definition of SlidingObject.
*/

#include <tracker/objects/sliding_object.h>
#include <tracker/dynamic_object_visitor.h>
#include <tracker/object_state.h>
#include <utils/stub.h>
#include <boost/range/iterator_range.hpp>

namespace vulcan
{
namespace tracker
{

SlidingObject::SlidingObject(ObjectId id, int64_t timestamp, const FixedObjectModel& model)
: FixedObject(timestamp, model)
, id_(id)
{
}


float SlidingObject::length(void) const
{
    std::cout << "STUB! SlidingObject::length\n";
    return 1.0f;
}


float SlidingObject::direction(void) const
{
    std::cout << "STUB! SlidingObject::direction\n";
    return 0.0f;
}


std::unique_ptr<DynamicObject> SlidingObject::clone(void) const 
{
    return std::unique_ptr<DynamicObject>(new SlidingObject(*this));
}


void SlidingObject::accept(DynamicObjectVisitor& visitor) const
{
    visitor.visitSlidingObject(*this);
}


velocity_t SlidingObject::estimateVelocity(int64_t timestamp) const
{
    PRINT_STUB("SlidingObject::estimateVelocity");
    return velocity_t(0.0f, 0.0f);
    
//     const int64_t kVelocityTimeWindowUs = 200000;
//     
//     // Find where the first saved state is within the window for which to estimate the velocity
//     auto windowBeginIt = std::find_if(beginRecentTrajectory(), endRecentTrajectory(),
//                                       [timestamp](const object_state_t& state)
//                                       {
//                                           return timestamp - state.timestamp < kVelocityTimeWindowUs;
//                                       });
//     
//     // If no states exist yet, then no velocity
//     if(windowBeginIt == endRecentTrajectory())
//     {
//         return velocity_t(0.0f, 0.0f);
//     }
//     
//     // Calculate the total distance moved along each axis
//     int64_t totalTime = 0;
//     double  totalX    = 0.0;
//     double  totalY    = 0.0;
//     
//     auto prevX    = windowBeginIt->x;
//     auto prevY    = windowBeginIt->y;
//     auto prevTime = windowBeginIt->timestamp;
//     
//     for(auto& state : boost::make_iterator_range(windowBeginIt+1, endRecentTrajectory()))
//     {
//         totalTime += state.timestamp - prevTime;
//         totalX    += state.x - prevX;
//         totalY    += state.y - prevY;
//         
//         prevTime = state.timestamp;
//         prevX    = state.x;
//         prevY    = state.y;
//     }
//     
//     // If no time has progressed, then not moving
//     if(totalTime == 0)
//     {
//         return velocity_t(0.0f, 0.0f);
//     }
//     
//     // Otherwise, the total distance moved over time is the magnitude of the velocity.
//     // The direction of the velocity points along the estimated line fit to the measured moving points.
//     double magnitude = std::sqrt(totalX*totalX + totalY*totalY) / (totalTime / 1000000.0);
//     double direction = direction(model().estimatedSegment());
//     
//     // If the direction motion of the object, as indicated by the measurements end points is closer in magnitude
//     // to the other possible direction for the line, then use that direction
//     if(angle_diff_abs(direction, std::atan2(totalY, totalX)) > M_PI_2)
//     {
//         direction = angle_sum(direction, M_PI);
//     }
//     
//     return velocity_t(std::cos(direction) * magnitude, std::sin(direction) * magnitude);
}
                                                     
} // namespace tracker
} // namespace vulcan

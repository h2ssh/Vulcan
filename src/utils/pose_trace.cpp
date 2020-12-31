/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pose_trace_.cpp
* \author   Collin Johnsn
*
* Implementation of PoseTrace.
*/

#include "utils/pose_trace.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <cassert>

namespace vulcan
{
namespace utils
{

// Functor that checks if the timestamp of the input pose occurs before the time specified in the constructor: pose.timestamp < time.
class PoseBeforeTime
{
public:

    PoseBeforeTime(int64_t time) : time(time) { }

    bool operator()(const pose_t& lhs) const { return lhs.timestamp < time; }

private:
    int64_t time;
};


PoseTrace::PoseTrace(float distance, std::size_t maxSize)
: minDistance_(distance)
, minDuration_(0)
, maxSize_(maxSize)
{
    assert(minDistance_ >= 0.0f);
    assert(maxSize_     >  0);
}


PoseTrace::PoseTrace(int64_t duration, std::size_t maxSize)
: minDistance_(0.0f)
, minDuration_(duration)
, maxSize_(maxSize)
{
    assert(minDuration_ >= 0);
    assert(maxSize_     >  0);
}


PoseTrace::PoseTrace(const std::string& filename)
{
    std::ifstream in(filename);
    pose_t loadedPose;

    while(true)
    {
        in >> loadedPose.timestamp;

        if(in.eof())
        {
            break;
        }

        in >> loadedPose.x;

        if(in.eof())
        {
            break;
        }

        in >> loadedPose.y;

        if(in.eof())
        {
            break;
        }

        in >> loadedPose.theta;

        trace_.push_back(loadedPose);
    }
}


bool PoseTrace::addPose(const pose_t& pose)
{
    bool shouldAdd = trace_.empty() || (isFarEnough(pose) && isElapsedEnough(pose));

    if(shouldAdd)
    {
        trace_.push_back(pose);
    }

    if((trace_.size() > maxSize_) && (maxSize_ > 0))
    {
        assert(trace_.size() == maxSize_ + 1);  // never should be more than one above the maximum size
        trace_.pop_front();
    }

    return shouldAdd;
}


pose_t PoseTrace::poseAt(int64_t time) const
{
    pose_t interpolatedPose;

    if(empty())
    {
        interpolatedPose.timestamp = time;
        return interpolatedPose;
    }

    if(front().timestamp >= time)
    {
        interpolatedPose = front();
    }
    else if(back().timestamp <= time)
    {
        interpolatedPose = back();
    }
    else
    {
        auto poseAfterIt = std::find_if_not(trace_.begin(), trace_.end(), PoseBeforeTime(time));

        assert(poseAfterIt != trace_.end());
        assert(poseAfterIt != trace_.begin());

        interpolatedPose = interpolate_pose(*(poseAfterIt-1), *poseAfterIt, time);
    }

    interpolatedPose.timestamp = time;
    return interpolatedPose;
}


bool PoseTrace::hasPoseBeforeTime(int64_t time) const
{
    return !trace_.empty() && (trace_.front().timestamp <= time);
}


bool PoseTrace::hasPoseAfterTime(int64_t time) const
{
    return !trace_.empty() && (trace_.back().timestamp >= time);
}


void PoseTrace::changeReferenceFrame(const pose_t& transform)
{
    for(auto& pose : trace_)
    {
        pose = pose.transformToNewFrame(transform);
    }
}


void PoseTrace::clearAll(void)
{
    trace_.clear();
}


std::size_t PoseTrace::clearBefore(int64_t time)
{
    auto poseIt = std::find_if_not(trace_.begin(), trace_.end(), PoseBeforeTime(time));

    std::size_t numRemoved = poseIt - trace_.begin();
    trace_.erase(trace_.begin(), poseIt);
    return numRemoved;
}


std::size_t PoseTrace::clearAfter(int64_t time)
{
    auto poseIt = std::find_if_not(trace_.begin(), trace_.end(), PoseBeforeTime(time));

    std::size_t numRemoved = trace_.end() - poseIt;

    if(poseIt != trace_.end()) // only erase if not the end, otherwise, not a valid iterator
    {
        trace_.erase(poseIt, trace_.end());
    }

    return numRemoved;
}


bool PoseTrace::saveToFile(const std::string& filename) const
{
    std::ofstream out(filename);

    if(!out.is_open())
    {
        std::cerr << "WARNING: PoseTrace: Failed to open file for saving the PoseTrace: " << filename << '\n';
        return false;
    }

    for(auto p : trace_)
    {
        out << p.timestamp << ' ' << p.x << ' ' << p.y << ' ' << p.theta << '\n';
    }

    return out.good();
}


bool PoseTrace::isFarEnough(const pose_t& pose) const
{
    assert(!trace_.empty());

    return distance_between_points(pose.toPoint(), trace_.back().toPoint()) >= minDistance_;
}


bool PoseTrace::isElapsedEnough(const pose_t& pose) const
{
    assert(!trace_.empty());

    return pose.timestamp - trace_.back().timestamp >= minDuration_;
}

} // namespace utils
} // namespace vulcan

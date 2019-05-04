/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \author   Collin Johnson
* \file     pose_trace.h
* 
* Definition of PoseTrace, a simple class for managing a sequence of poses.
*/

#ifndef UTILS_POSE_TRACE_H
#define UTILS_POSE_TRACE_H

#include <core/pose.h>
#include <deque>

namespace vulcan
{
namespace utils
{

/**
* PoseTrace represents a sequence of poses separated based on distance or time criteria. The addPose method will
* add new poses to the trace if they are far enough from the previous end pose. When the distance criterion is set,
* poses are added when:
*   
*   dist(newPose, lastPose) > distance
* 
* When the time criterion is set, poses are added when:
* 
*   newPose.timestamp - lastPose.timestamp > duration
* 
* In all cases, a maximum number of poses can be set for the trace_. When the maximum number of poses is hit, each addition
* will erase the pose at the start of the trace. Poses can also be removed via the clearAll(), clearBefore(), or clearAfter()
* methods that erase all poses occurring before or after a certain time.
*
* Poses are stored in ascending order of timestamp.
* 
* The iterators returned from begin()/end() are guaranteed to be RandomAccessIterators.
*/
class PoseTrace
{
public:
    
    using ConstIter  = std::deque<pose_t>::const_iterator;
    using ConstRIter = std::deque<pose_t>::const_reverse_iterator;
    
    /**
    * Constructor for PoseTrace.
    * 
    * Create a pose trace where poses will be separated by at least distance. There is no minimum required time
    * between poses.
    * 
    * \param    distance        Minimum distance between consecutive poses in the trace (optional) (default = 0.05m)
    * \param    maxSize         Maximum number of poses in the trace (optional, default = 5000) (0 = no limit)
    * 
    * \pre  distance >= 0.0f
    * \pre  maxSize > 0
    */
    explicit PoseTrace(float distance = 0.05f, std::size_t maxSize = 5000);
    
    /**
    * Constructor for PoseTrace.
    * 
    * Create a pose trace where the time elapsed is at least duration. There is no minimum required distance between
    * poses.
    * 
    * \param    duration        Minimum duration between consecutive poses in the trace
    * \param    maxSize         Maximum number of poses in the trace (optional, default = 5000) (0 = no limit)
    * 
    * \pre  duration >= 0
    * \pre  maxSize > 0
    */
    explicit PoseTrace(int64_t duration, std::size_t maxSize = 5000);

    /**
    * Constructor for PoseTrace.
    *
    * Load a PoseTrace from a file.
    *
    * A loaded PoseTrace is not intended to be modified. You can if you want and the default constructor will be used
    * to set the parameters pertaining to modification of the trace.
    *
    * \param    filename        File containing the PoseTrace
    */
    explicit PoseTrace(const std::string& filename);

    /**
    * addPose adds a new pose to the end of the trace if the addition criterion specified on construction is satisfied.
    * 
    * \param    pose            Pose to be added
    * \return   True if the pose was added.
    */
    bool addPose(const pose_t& pose);
    
    /**
    * poseAt finds the pose at the specified time. The pose at the specified time is determined via linear interpolation
    * between the time immediately before and immediately after the requested pose time.
    * 
    * \param    time        Time at which to find the pose
    * \return   time < front().time -> front()
    *           time > back().time  -> back()
    *           else interpolate between pose_time-delta and pose_time+delta
    */
    pose_t poseAt(int64_t time) const;
    
    /**
    * hasPoseBeforeTime checks if the trace has a pose that occurs before the specified time. 
    * 
    * \param    time        Time at which to check for poses
    * \return   True if there is a pose before or equal to the specified time. False if there are no poses or no poses
    *           with a time before the specified time.
    */
    bool hasPoseBeforeTime(int64_t time) const;
    
    /**
    * hasPoseAfterTime checks if the trace has a pose that occurs after the specified time.
    * 
    * \param    time        Time at which to check for poses
    * \return   True if there is a pose after or equal to the specified time. False if there are no poses or no poses
    *           with a time after the specified time.
    */
    bool hasPoseAfterTime(int64_t time) const;
    
    /**
    * changeReferenceFrame updates the reference frame for all the poses in the trace_.
    * 
    * \param    transform       Transform to apply to get to the new frame
    */
    void changeReferenceFrame(const pose_t& transform);
    
    /**
    * clearAll removes all poses from the trace.
    */
    void clearAll(void);
    
    /**
    * clearBefore removes all poses occurring before the specified time.
    * 
    * \param    time            Time before which all poses will be cleared
    * \return   The number of poses erased.
    */
    std::size_t clearBefore(int64_t time);
    
    /**
    * clearAfter removes all poses occurring after the specified time.
    * 
    * \param    time            Time after which all poses will be cleared
    * \return   The number of poses erased.
    */
    std::size_t clearAfter(int64_t time);

    /**
    * saveToFile saves the PoseTrace to a file that can be loaded in the future. The format is simply:
    *
    *   timestamp x y theta
    *
    * with one pose saved per line of the file.
    *
    * \param    filename        Filename in which to save the poses
    * \return   True if saving the poses was successful.
    */
    bool saveToFile(const std::string& filename) const;
    
    /**
    * begin retrieves a ConstIter to the start of the pose trace.
    */
    ConstIter begin(void) const { return trace_.begin(); }
    
    /**
    * end retrieves a ConstIter to the one-past-the-end of the pose trace.
    */
    ConstIter end(void) const { return trace_.end(); }
    
    /**
    * rbegin retrieves a reverse iterator to the end of the trace.
    */
    ConstRIter rbegin(void) const { return trace_.rbegin(); }
    
    /**
    * rend retrieves a reverse end iterator for the trace.
    */
    ConstRIter rend(void) const { return trace_.rend(); }
    
    /**
    * front retrieves a reference to the front of the trace.
    */
    const pose_t& front(void) const { return trace_.front(); }
    
    /**
    * back retrieves a reference to the end of the trace.
    */
    const pose_t& back(void) const { return trace_.back(); }
    
    /**
    * operator[] to get direct access to a particular pose index.
    * 
    * No check for out of bounds.
    */
    const pose_t& operator[](int index) const { return trace_[index]; }
    
    /**
    * at to get direct access to a particular pose index.
    * 
    * Throws if out of bounds.
    */
    const pose_t& at(int index) const { return trace_.at(index); }
    
    /**
    * size retrieves the size of the pose trace.
    */
    std::size_t size(void) const { return trace_.size(); }
    
    /**
    * empty checks if there are any poses in the trace.
    */
    bool empty(void) const { return trace_.empty(); }
    
    /**
    * pop_front removes the pose at the front of the trace. Does nothing if no poses.
    */
    void pop_front(void) { if(!trace_.empty()) { trace_.pop_front(); } }
    
    /**
     * pop_back removes the pose at the end of the trace. Does nothing if no poses.
    */
    void pop_back(void) { if(!trace_.empty()) { trace_.pop_back(); } }
    
private:
    
    std::deque<pose_t> trace_;
    
    float       minDistance_;
    int64_t     minDuration_;
    std::size_t maxSize_;
    
    bool isFarEnough    (const pose_t& pose) const;
    bool isElapsedEnough(const pose_t& pose) const;
};

}
}

#endif // UTILS_POSE_TRACE_H

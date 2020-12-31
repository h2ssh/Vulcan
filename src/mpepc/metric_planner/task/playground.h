/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     playground.h
* \author   Jong Jin Park
* 
* Declaration of PlaygroundTask, which encapsulates a rectangular boundary of an area.
*/

#ifndef MPEPC_PLAYGROUND_TASK_H
#define MPEPC_PLAYGROUND_TASK_H

#include "mpepc/metric_planner/task/task.h"
#include "mpepc/metric_planner/task/params.h"
#include "math/geometry/rectangle.h"
#include <cereal/access.hpp>
#include <cereal/types/base_class.hpp>

namespace vulcan
{

namespace mpepc
{
    
class PlaygroundTask : public MetricPlannerTask
{
    static const int32_t kPlaygroundTaskID = 3;
    
    /**
    * Constructor for PlaygroundTask.
    * 
    * \param    target          Target to be associated with the task
    */
    PlaygroundTask(const math::Rectangle<float>& target)
    : MetricPlannerTask()
    , target_(target)
    {
    }
    
    /**
    * getTarget retrieves the target associated with this task.
    * 
    * \return   Intial location of the person for the robot to follow or to pace with
    */
    math::Rectangle<float> getTarget(void) const { return target_; }
    
    // MetricPlannerTask interface
    virtual int32_t getId(void) const { return kPlaygroundTaskID; } /* override */
    virtual bool    isSafeToExecute(const ObstacleDistanceGrid& map) { return false;}; /* override */ // TODO: Implement this!
    
protected:
    
    // For serialization support, allow subclasses to default construct
    PlaygroundTask(void) { }
    
private:
    
    math::Rectangle<float> target_;
    
    // Serialization support
    friend class ::cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar & target_;
    }
};

} // mpepc
} // vulcan

// Serialization support via smart pointer
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::mpepc::PlaygroundTask)

#endif // MPEPC_PLAYGROUND_TASK_H
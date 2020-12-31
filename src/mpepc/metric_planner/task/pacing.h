/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pacing.h
* \author   Jong Jin Park
* 
* Declaration of PacingTask, which encapsulates a position of the target person.
*/

#ifndef MPEPC_PACING_TASK_H
#define MPEPC_PACING_TASK_H

#include "mpepc/metric_planner/task/task.h"
#include "mpepc/metric_planner/task/params.h"
#include "core/point.h"
#include <cereal/access.hpp>
#include <cereal/types/base_class.hpp>


namespace vulcan
{

namespace mpepc
{

class PacingTask : public MetricPlannerTask
{
    static const int32_t kPacingTaskID = 2;
    
    /**
    * Constructor for PacingTask.
    * 
    * \param    target          Target to be associated with the task
    */
    PacingTask(const Point<float>& target)
    : MetricPlannerTask()
    , target_(target)
    {
    }
    
    /**
    * getTarget retrieves the target associated with this task.
    * 
    * \return   Intial location of the person for the robot to follow or to pace with
    */
    Point<float> getTarget(void) const { return target_; }
    
    // MetricPlannerTask interface
    virtual int32_t getId(void) const { return kPacingTaskID; } /* override */
    virtual bool    isSafeToExecute(const ObstacleDistanceGrid& map); /* override */
    
protected:
    
    // For serialization support, allow subclasses to default construct
    PacingTask(void) { }
    
private:
    
    Point<float> target_;
    
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

CEREAL_REGISTER_TYPE(vulcan::mpepc::PacingTask)

#endif // MPEPC_PACING_TASK_H

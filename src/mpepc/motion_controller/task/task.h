/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     task.h
* \author   Collin Johnson
* 
* Declaration of MotionControllerTask interface for commanding the motion controller to do something.
*/

#ifndef MPEPC_MOTION_CONTROLLER_TASK_H
#define MPEPC_MOTION_CONTROLLER_TASK_H

#include "utils/timestamp.h"
#include <string>
#include "system/message_traits.h"
#include <cereal/access.hpp>
#include <cereal/types/memory.hpp>

namespace vulcan
{

namespace mpepc
{

/**
* MotionControllerTask is the interface to be implemented for all types of tasks that 
* can be handled by the motion controller. The task only identifies which unique type
* it is, then a task handler is responsible for casting down the hierarchy to the
* actual type of the task to be used internally.
* 
* A task is defined by its id. The interface provides only identification and any further
* usefulness is in the subclasses, thus a cast is unfortunately required.
* 
* The task interface consists of two methods:
* 
*   - getId() retrieves the unique identifier for the task
*   - getDescription() retrieves a text description of the task, to be used only for diagnostics
*/
class MotionControllerTask
{
public:
    
    /**
    * Default constructor for MotionControllerTask.
    * 
    * \param    timestamp           Optional timestamp value to assign, otherwise the current system time will be used
    */
    MotionControllerTask(int64_t timestamp)
    {
        this->timestamp = (timestamp != 0) ? timestamp : utils::system_time_us();
    }
    
    virtual ~MotionControllerTask(void) { }
    
    /**
    * getTimestamp retrieves the timestamp indicating when this task was created.
    */
    int64_t getTimestamp(void) const { return timestamp; }
    
    /**
    * getId retrieves the unique identifier for this type of task.
    * 
    * \pre      The id is unique amongst all subclasses of MotionControllerTask.
    * \return   The identifier associated with the task.
    */
    virtual int32_t getId(void) const = 0;
    
    /**
    * getDescription retrieves a text description of the task. The text description
    * is not guaranteed to be unique and should be used only for diagnostic or debugging
    * purposes.
    * 
    * \return   Text description of the task.
    */
    virtual std::string getDescription(void) const = 0;
    
protected:
    
    // For serialization support, allow subclasses to default construct
    MotionControllerTask(void)
    {
        this->timestamp = 0;
    }
    
private:
    
    int64_t timestamp;
    
    // Serialization support
    friend class ::cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar & timestamp;
    }
};

} // mpepc
} // vulcan

DEFINE_SYSTEM_MESSAGE(std::shared_ptr<vulcan::mpepc::MotionControllerTask>, ("MPEPC_MOTION_CONTROLLER_TASK"))

#endif // MPEPC_MOTION_CONTROLLER_TASK_H

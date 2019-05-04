/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     script_task.h
* \author   Collin Johnson and Jong Jin Park
*
* Declaration of ScriptTask interface, which is an element of the MetricPlannerScript.
*/

#ifndef MPEPC_SCRIPT_TASK_H
#define MPEPC_SCRIPT_TASK_H

#include <core/pose.h>
#include <utils/timestamp.h>
#include <cereal/access.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace mpepc
{

/**
* ScriptTask defines the properties for a task in a script. A task is identified by a sequence of poses and optional
* names for those poses.
*
* Use of a ScriptTask
*
* NOTE: Task may also want to carry around the criteria for determining the success/failure
*       of the task as part of the task specification. Now all tasks of a same type share a single
*       success criteria, which does work most of the time and hence is specified in the parameter
*       structure of the planner for that specific task. Just making a note of that here...
*/
class ScriptTask
{
public:

    /**
     * Default constructor.
     */
    ScriptTask(void) = default;

    /**
    * Constructor for ScriptTask.
    *
    * Create a task to drive to a single target.
    *
    * \param    target              Target pose for the task
    * \param    description         Description of the task
    * \param    timestamp           Timestamp to assign the task (optional, default = current time)
    */
    ScriptTask(pose_t target, std::string description, int64_t timestamp = 0)
    : ScriptTask(std::vector<pose_t>(1, target), std::move(description), timestamp)
    {
    }

    /**
    * Constructor for ScriptTask.
    *
    * Create a task to drive to a single target.
    *
    * \param    target              Target pose for the task
    * \param    name                Name for the target
    * \param    description         Description of the task
    * \param    timestamp           Timestamp to assign the task (optional, default = current time)
    */
    ScriptTask(pose_t target, std::string name, std::string description, int64_t timestamp = 0)
    : ScriptTask(std::vector<pose_t>(1, target), std::vector<std::string>(1, name), std::move(description), timestamp)
    {
    }

    /**
    * Constructor for ScriptTask.
    *
    * Create a task to drive to a sequence of targets. All targets are auto-named 'Pose'.
    *
    * \param    targets         Target poses for the task
    * \param    description     Type of task for the target
    * \param    timestamp       Timestamp to assign the task (optional, default = current time)
    */
    ScriptTask(std::vector<pose_t> targets, std::string description, int64_t timestamp = 0)
    : description_(description)
    , targets_(std::move(targets))
    , names_(targets_.size(), "Pose")
    {
        timestamp_ = (timestamp != 0) ? timestamp : utils::system_time_us();
    }

    /**
    * Constructor for ScriptTask.
    *
    * Create a task to drive to a sequence of named targets.
    *
    * \param    targets         Target poses for the task
    * \param    names           Names for the targets
    * \param    description     Type of task for the target
    * \param    timestamp       Timestamp to assign the task (optional, default = current time)
    */
    ScriptTask(std::vector<pose_t> targets,
               std::vector<std::string> names,
               std::string description,
               int64_t timestamp = 0)
    : ScriptTask(targets, std::move(description), timestamp)
    {
        names_ = std::move(names);
    }

    /**
    * getTimestamp retrieves the timestamp of the task.
    */
    int64_t getTimestamp(void) const { return timestamp_; }

    /**
    * getDescription retrieves a text description of the planner task. The description
    * can be anything and isn't used in processing, just is useful for informing the user
    * of currently executing processes.
    *
    * \return   A text description of the task.
    */
    std::string getDescription(void) const { return description_; }

    /**
    * getTargets will return a vector of all target poses within a task.
    */
    std::vector<pose_t> getTargets(void) const { return targets_; }

    /**
    * Retrieve the descriptions of the targets, if any were provided.
    */
    std::vector<std::string> getTargetNames(void) const { return names_; }

private:

    int64_t timestamp_;
    std::string description_;
    std::vector<pose_t> targets_;
    std::vector<std::string> names_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive &ar)
    {
        ar (timestamp_,
            description_,
            targets_,
            names_
        );
    }
};

} // namespace mpepc
} // namespace vulcan

#endif // MPEPC_SCRIPT_TASK_H

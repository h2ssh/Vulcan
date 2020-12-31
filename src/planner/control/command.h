/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     command.h
 * \author   Collin Johnson
 *
 * Definition of ControlCommand.
 */

#ifndef PLANNER_CONTROL_COMMAND_H
#define PLANNER_CONTROL_COMMAND_H

#include "mpepc/metric_planner/task/task.h"
#include "planner/control/task.h"
#include "system/message_traits.h"
#include <cereal/types/memory.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace planner
{

/**
 * ControlCommand describes a task to be performed by the control planner. The command contains identifying information
 * about the source of the task along with the task to be performed.
 *
 * The id and source are used for all communications of ControlResult to identify the task being performed.
 */
class ControlCommand
{
public:
    /**
     * Default constructor for ControlCommand.
     */
    ControlCommand(void) : timestamp_(-1), source_("default constructor") { }

    /**
     * Constructor for ControlCommand.
     */
    ControlCommand(int64_t timestamp, const std::string& source, const std::shared_ptr<ControlTask>& task)
    : timestamp_(timestamp)
    , source_(source)
    , task_(task)
    {
    }

    /**
     * timestamp retrieves the timestamp for when this command was created.
     */
    int64_t timestamp(void) const { return timestamp_; }

    /**
     * source retrieves the source of the command.
     */
    std::string source(void) const { return source_; }

    /**
     * task retrieves the task to be performed to satisfy the command.
     */
    std::shared_ptr<ControlTask> task(void) const { return task_; }

private:
    int64_t timestamp_;
    std::string source_;
    std::shared_ptr<ControlTask> task_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(timestamp_, source_, task_);
    }
};

}   // namespace planner
}   // namespace vulcan

DEFINE_SYSTEM_MESSAGE(planner::ControlCommand, ("CONTROL_COMMAND"))

#endif   // PLANNER_CONTROL_COMMAND_H

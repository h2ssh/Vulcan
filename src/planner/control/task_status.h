/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     task_status.h
 * \author   Collin Johnson
 *
 * Definition of ControlTaskStatus.
 */

#ifndef PLANNER_CONTROL_TASK_STATUS_H
#define PLANNER_CONTROL_TASK_STATUS_H

#include "system/message_traits.h"
#include <cassert>
#include <cereal/access.hpp>

namespace vulcan
{
namespace planner
{

enum class ControlTaskProgress
{
    waiting,     ///< The task has yet to be started
    executing,   ///< The task has been started, but has not yet finished or failed
    completed,   ///< The task has finished successfully
    failed,      ///< The task has failed to be completed -- see error codes
};

enum class ControlTaskError
{
    none,                 ///< No error has occurred
    target_unreachable,   ///< The target can't be reached due to obstructions in the environment
    target_invalid,       ///< The target is invalid because it falls outside the known map
};

const int32_t kInvalidTaskStatusId = -1;


/**
 * ControlTaskStatus defines the status of the currently executing task.
 */
class ControlTaskStatus
{
public:
    /**
     * Default constructor for ControlTaskStatus.
     */
    ControlTaskStatus(void) : timestamp_(0), progress_(ControlTaskProgress::waiting), error_(ControlTaskError::none) { }

    /**
     * Constructor for ControlTaskStatus.
     *
     * Create a non-error status.
     */
    ControlTaskStatus(int64_t timestamp, int32_t id, ControlTaskProgress progress)
    : timestamp_(timestamp)
    , id_(id)
    , progress_(progress)
    , error_(ControlTaskError::none)
    {
        assert(progress != ControlTaskProgress::failed);
    }

    /**
     * Constructor for ControlTaskStatus.
     *
     * Creates an error status
     */
    ControlTaskStatus(int64_t timestamp, int32_t id, ControlTaskError error)
    : timestamp_(timestamp)
    , id_(id)
    , progress_(ControlTaskProgress::failed)
    , error_(error)
    {
        assert(error != ControlTaskError::none);
    }

    /**
     * timestamp retrieves the time the status was created.
     */
    int64_t timestamp(void) const { return timestamp_; }

    /**
     * id retrieves the id of the task that created the status.
     */
    int32_t id(void) const { return id_; }

    /**
     * progress retrieves the current progress of the task.
     */
    ControlTaskProgress progress(void) const { return progress_; }

    /**
     * error retrieves the error state of the task if it has failed.
     */
    ControlTaskError error(void) const { return error_; }

private:
    int64_t timestamp_;
    int32_t id_;
    ControlTaskProgress progress_;
    ControlTaskError error_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(timestamp_, id_, progress_, error_);
    }
};

}   // namespace planner
}   // namespace vulcan

DEFINE_SYSTEM_MESSAGE(planner::ControlTaskStatus, ("CONTROL_TASK_STATUS"))

#endif   // PLANNER_CONTROL_TASK_STATUS_H

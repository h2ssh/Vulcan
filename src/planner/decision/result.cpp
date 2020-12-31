/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     result.cpp
 * \author   Collin Johnson
 *
 * Definition of DecisionResult.
 */

#include "planner/decision/result.h"
#include "system/system_communicator.h"

namespace vulcan
{
namespace planner
{

DecisionResult::DecisionResult(const boost::optional<DecisionStatus>& status,
                               const boost::optional<DecisionException>& exception,
                               std::unique_ptr<MetricPlannerTask> task)
: status_(status)
, exception_(exception)
, task_(std::move(task))
{
}

// Use default implementation of destructor and move constructors
DecisionResult::~DecisionResult(void) = default;
DecisionResult::DecisionResult(DecisionResult&& rhs) = default;
DecisionResult& DecisionResult::operator=(DecisionResult&& rhs) = default;


void DecisionResult::send(system::SystemCommunicator& communicator) const
{
    // Send whichever results were set on construction
    if (status_) {
        communicator.sendSystem(status_.get());
    }

    if (exception_) {
        communicator.sendSystem(exception_.get());
    }

    if (task_) {
        communicator.sendSystem(task_);
    }
}

}   // namespace planner
}   // namespace vulcan

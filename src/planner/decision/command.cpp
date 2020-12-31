/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     command.cpp
 * \author   Collin Johnson
 *
 * Definition of DecisionCommand.
 */

#include "planner/decision/command.h"
#include "planner/decision/planner.h"
#include "utils/stub.h"

namespace vulcan
{
namespace planner
{

DecisionCommand::DecisionCommand(const std::string& source, IssueMode mode, const Tasks& tasks)
: source_(source)
, mode_(mode)
, tasks_(tasks)
{
}


DecisionCommand::DecisionCommand(const std::string& source) : source_(source), mode_(Clear)
{
}


void DecisionCommand::issue(DecisionPlanner& planner) const
{
    PRINT_STUB("DecisionCommand::issue");
}

}   // namespace planner
}   // namespace vulcan

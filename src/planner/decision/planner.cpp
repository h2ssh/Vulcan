/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     planner.cpp
 * \author   Collin Johnson
 *
 * Definition of DecisionPlanner.
 */

#include "planner/decision/planner.h"
#include "utils/stub.h"

namespace vulcan
{
namespace planner
{

DecisionPlanner::DecisionPlanner(void)
{
}


DecisionPlanner::~DecisionPlanner(void)
{
}


void DecisionPlanner::addTaskToFront(std::unique_ptr<DecisionTask> task)
{
    PRINT_STUB("DecisionPlaner::addTaskToFront");
}


void DecisionPlanner::addTaskToBack(std::unique_ptr<DecisionTask> task)
{
    PRINT_STUB("DecisionPlaner::addTaskToBack");
}


void DecisionPlanner::clearTasks(void)
{
    PRINT_STUB("DecisionPlaner::clearTasks");
}


DecisionResult DecisionPlanner::plan(const DecisionState& state)
{
    PRINT_STUB("DecisionPlaner::plan");

    return DecisionResult{};
}

}   // namespace planner
}   // namespace vulcan

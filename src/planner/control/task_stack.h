/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     task_stack.h
* \author   Collin Johnson
* 
* Declaration of ControlTaskStack.
*/

#ifndef PLANNER_CONTROL_TASK_STACK_H
#define PLANNER_CONTROL_TASK_STACK_H

#include <deque>
#include <memory>

namespace vulcan
{
namespace planner
{
    
class ControlTask;

// A deque currently supplies all functionality needed for the task stack
using ControlTaskStack = std::deque<std::shared_ptr<ControlTask>>;

}
}

#endif // PLANNER_CONTROL_TASK_STACK_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     sequence.cpp
* \author   Collin Johnson
*
* Definition of DecisionTaskSequence.
*/

#include "planner/decision/sequence.h"

namespace vulcan
{
namespace planner
{

DecisionTaskSequence::~DecisionTaskSequence(void)
{
    // for unique_ptr
}


void DecisionTaskSequence::appendTask(std::unique_ptr<DecisionTask> task)
{
    // Push task to back of sequence if it is a valid task
    if(task)
    {
        tasks_.push_back(std::move(task));
    }
}


void DecisionTaskSequence::prependTask(std::unique_ptr<DecisionTask> task)
{
    // Push task on front of sequence if it is a valid task
    if(task)
    {
        tasks_.push_front(std::move(task));
    }
}


std::unique_ptr<DecisionTask> DecisionTaskSequence::nextTask(void)
{
    // If there is a task, then pull it off the front of the sequence
    if(!tasks_.empty())
    {
        // Ensure the pointer is moved, so create explicitly. auto would bind to a reference, which wouldn't give
        // the correct type.
        auto frontTask = std::unique_ptr<DecisionTask>{std::move(tasks_.front())};
        tasks_.pop_front();
        return frontTask;
    }

    // Else return a null task
    return nullptr;
}

}
}

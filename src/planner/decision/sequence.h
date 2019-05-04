/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     sequence.h
* \author   Collin Johnson
*
* Declaration of DecisionTaskSequence.
*/

#ifndef PLANNER_DECISION_SEQUENCE_H
#define PLANNER_DECISION_SEQUENCE_H

#include <deque>
#include <memory>

namespace vulcan
{
namespace planner
{

class DecisionTask;

/**
* DecisionTaskSequence represents the sequence of tasks to be performed by the DecisionPlanner. The sequence ensures
* that no null tasks exist.
*
* Tasks are added to the front or back of the sequence via the appendTask or prependTask methods. The next task to
* execute is grabbed via nextTask. nextTask removes the task from the sequence, so it ownership is transferred to the
* caller.
*/
class DecisionTaskSequence
{
public:

    ~DecisionTaskSequence(void); // for unique_ptr

    /**
    * numTasks retrieves the number of tasks in the sequence.
    */
    std::size_t numTasks(void) const { return tasks_.size(); }

    /**
    * haveTask checks if there are any tasks in the sequence. Equivalent to numTasks() == 0.
    */
    bool haveTask(void) const { return !tasks_.empty(); }

    /**
    * appendTask appends a new task to the end of the sequence if it isn't null. A null task will be ignored.
    */
    void appendTask(std::unique_ptr<DecisionTask> task);

    /**
    * preendTask appends a new task to the front of the sequence if it isn't null. A null task will be ignored.
    */
    void prependTask(std::unique_ptr<DecisionTask> task);

    /**
    * clearTasks clears tasks from the sequence. After clearing, haveTask() == false.
    */
    void clearTasks(void) { tasks_.clear(); }

    /**
    * nextTask removes the next task in the sequence from the front of the sequence and returns it. Subsequent calls to
    * nextTask will not return the same task.
    */
    std::unique_ptr<DecisionTask> nextTask(void);

private:

    std::deque<std::unique_ptr<DecisionTask>> tasks_;
};

}
}

#endif // PLANNER_DECISION_SEQUENCE_H

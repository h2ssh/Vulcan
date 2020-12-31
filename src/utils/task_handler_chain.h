/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     task_handler_chain.h
 * \author   Collin Johnson
 *
 * Definition of TaskHandlerChain.
 */

#ifndef UTILS_TASK_HANDLER_CHAIN_H
#define UTILS_TASK_HANDLER_CHAIN_H

#include <cassert>
#include <memory>
#include <vector>

namespace vulcan
{
namespace utils
{

/**
 * TaskHandlerChain
 *
 * A TaskHandler requires the following interface:
 *
 *   - bool canHandleTask(task)
 *   - void assignTask(task);
 *   - bool isTaskComplete(task_data)
 */
template <class TaskHandler, class Task, class TaskData>
class TaskHandlerChain
{
public:
    TaskHandlerChain(void) : activeHandlerIndex(1) { }

    virtual ~TaskHandlerChain(void) { }

    /**
     * assignTask assigns a new task to be executed by the chain.
     *
     * \param    task            Task to be executed
     * \return   True if the task could be handled. False if no handler exists for it.
     */
    bool assignTask(const std::shared_ptr<Task>& task)
    {
        for (std::size_t index = 0; index < handlers.size(); ++index) {
            if (handlers[index]->canHandleTask(task)) {
                handlers[index]->assignTask(task);
                activeHandlerIndex = index;
                return true;
            }
        }

        activeHandlerIndex = handlers.size() + 1;

        return false;
    }

    /**
     * isTaskComplete checks to see if the current task has finished.
     *
     * \param    data        Data to determine task completion
     * \return   True if the task is finished. False otherwise.
     */
    bool isTaskComplete(const TaskData& data)
    {
        if (haveTask()) {
            return activeHandler().isTaskComplete(data);
        }

        return true;
    }

protected:
    /**
     * addHandler adds a new task handler to the end of the chain.
     *
     * \param    handler         Handler to be added. Won't be added if null.
     */
    void addHandler(std::unique_ptr<TaskHandler>&& handler)
    {
        if (handler) {
            handlers.push_back(std::move(handler));
        }
    }

    /**
     * haveTask checks to see if there's an active task.
     */
    bool haveTask(void) const { return activeHandlerIndex < handlers.size(); }

    /**
     * completedTask informs the handler that the current task has finished, so there isn't an active handler anymore.
     */
    void completedTask(void) { activeHandlerIndex = handlers.size() + 1; }

    /**
     * activeHandler returns the active handler. If no handler is active, the program will crash.
     */
    TaskHandler& activeHandler(void)
    {
        if (activeHandlerIndex >= handlers.size()) {
            assert(activeHandlerIndex < handlers.size());
        }

        return *(handlers[activeHandlerIndex]);
    }

private:
    // INVARIANT: activeHandlerIndex == handlers.size() + 1 if no active handler
    std::size_t activeHandlerIndex;
    std::vector<std::unique_ptr<TaskHandler>> handlers;
};

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_TASK_HANDLER_CHAIN_H

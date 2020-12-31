/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     repeated_task.h
 * \author   Collin Johnson
 *
 * Definition of RepeatedTask.
 */

#ifndef UTILS_REPEATED_TASK_H
#define UTILS_REPEATED_TASK_H

#include <atomic>
#include <functional>
#include <thread>

namespace vulcan
{
namespace utils
{

/**
 * RepeatedTask is a simple task abstraction for executing some function on a separate thread over and over
 * until the function completes its task or the task is killed by some other bit of code.
 *
 * The task being run by the RepeatedTask  must have the following signature and behavior:
 *
 *   bool task(bool, args...)
 *
 * When run, the task is passed a boolean indicating if the task has been requested to end. If true, then
 * the task should exit after this call. If false, the task can keep running.
 *
 * The return value is true if the task has work remaining, false if the task is finished.
 *
 */
class RepeatedTask
{
public:
    /**
     * Constructor for RepeatedTask.
     *
     * \param    f       Function -- f(bool, args...)
     * \param    args    Arguments for the function
     */
    template <class F, class... Args>
    RepeatedTask(F&& f, Args&&... args) : killed_(false)
    {
        auto killAblefunc = std::bind(f, std::placeholders::_1, std::forward<Args>(args)...);
        auto threadFunc = [killAblefunc, this]() {
            bool running = true;
            while (running && !killed_) {
                running = killAblefunc(killed_);
            }
            // If exited because not alive anymore, then run just once more with the flag hard-coded to indicate
            // the function is exiting, in case it hadn't done so.
            if (running) {
                killAblefunc(true);
            }
        };

        t_ = std::thread(threadFunc);
    }

    /**
     * Destructor for RepeatedTask.
     */
    ~RepeatedTask(void)
    {
        // If the task is still running and it is destruction time, then kill the task and wait for it to finish
        if (t_.joinable()) {
            kill();
            t_.join();
        }
    }

    /**
     * kill tells the task to stop. A call to join should be made to ensure the task has completed.
     */
    void kill(void) { killed_ = true; }

    /**
     * join waits for the thread to finish executing. If the task hasn't been killed, then join might never return,
     * depending on the nature of the task being executed.
     */
    void join(void) { t_.join(); }

private:
    std::thread t_;              // Thread on which the task will be running
    std::atomic<bool> killed_;   // Flag indicating if the task should continue running -- false if still running
};

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_REPEATED_TASK_H

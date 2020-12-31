/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "utils/thread.h"
#include <errno.h>
#include <iostream>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

//#define DEBUG_MUTEX
//#define DEBUG_COND
//#define DEBUG_THREAD

namespace vulcan
{
namespace utils
{

// Definitions for Thread

/**
 * Constructor for Thread. Creates a Thread instance that is ready to be run. After
 * creation, a task should be attached to the Thread using attachTask() and then
 * start() should be called.
 *
 * \param    joinable            Flag to indicate if the Thread should be joinable
 * \param    persistant          Flag to keep the thread persistant across multiple tasks
 */
Thread::Thread(bool joinable, bool persistant)
: detached(!joinable)
, persist(persistant)
, running(false)
, executing(false)
, joined(!joinable)
, persistCond(false)
, objTask(0)
{
}


/**
 * Destructor for Thread. Cleans up any lingering effects of the Thread.
 */
Thread::~Thread(void)
{
    /*
     * Things that need to be done:
     *  1) Stop the thread if it is still running
     *  2) Join the thread if it has yet to be joined and is joinable
     */

#ifdef DEBUG_THREAD
    std::cout << "Thread being destructed" << std::endl;
#endif

    flagLock.lock();

    if (!executing && persist)   // give the thread a chance to finish up on its own -- if here, then waiting on cond,
                                 // broadcast and it'll bail
    {
        persist = false;
        objTask = 0;

        flagLock.unlock();
        persistCond.setPredicate(true);
        persistCond.broadcast();
        usleep(50000);   // wait a touch for it to end things
    } else if (running) {
        flagLock.unlock();
        kill();
    }

#ifdef DEBUG_THREAD
    std::cout << "Thread successfully destroyed" << std::endl;
#endif
}


/**
 * detach will take a joinable thread and detach it. If a Thread is detached, it cannot
 * be joined later. It will run until finished and then exit with no way to indicate its
 * completion.
 *
 * \return   0 if the Thread is successfully detached, or was already detached, -1 if an error
 */
int Thread::detach(void)
{
    // If the Thread is not running, then set the detached flag so it will be created
    // to be detached when started. If the thread is already running, then the detach
    // call will be made if the thread is not already detached

    if (running && !detached) {
        // error conditions that arise from this are if already detached or no such thread, but these
        // would both violate invariant maintained by the class
        pthread_detach(thread);
        detached = true;
    } else {
        detached = true;
    }

    return 0;
}


/**
 * join will join a joinable Thread. If the Thread has yet to be started, it will
 * set the Thread to be joinable on creation. If the Thread has started and is
 * joinable then a call to join will block until the Thread has completed execution.
 *
 * \param    retVal                Pointer to void** that will store results of the join operation
 * \return   0 if join was successful, 1 if thread has not started yet, -1 if thread can't be joined
 */
int Thread::join(void** retVal)
{
    /*
     * Handle the three potential cases: thread not running, so initialize to be joinable.
     * Thread detached, nothing you can do, Thread joinable, so make that call.
     */
    if (!running) {
        detached = false;
    } else if (detached) {
        return -1;
    }

#ifdef DEBUG_THREAD
    std::cout << "Beginning call to join" << std::endl;
#endif

    int success = pthread_join(thread, retVal);

    joined = true;

#ifdef DEBUG_THREAD
    std::cout << "Join complete" << std::endl;
#endif

    if (success == EINVAL) {
        return -1;
    }

    return 0;
}


/**
 * kill terminates execution of the Thread. The Thread will be told to exit.
 * If the Thread is joinable, then join should be called after a call to kill.
 *
 * Once a Thread is killed, a new call to start must be issued if it is to start
 * executing code again.
 *
 * Cancellation of the Thread will occur once the current task finishes executing.
 *
 * \return   0 if cancellation is successful, -1 otherwise.
 */
int Thread::kill(void)
{
    // Only cancel if running
    if (running) {
        pthread_cancel(thread);
        // pthread_kill(thread, SIGINT);
        running = false;
    }

    return 0;
}


/**
 * makePersistant makes the Thread persistant. A persistant Thread will continue
 * operating after its current task has finished. It will wait until a new task
 * is assigned to it, at which point it will begin processing again.
 *
 * \return   makePersistant always returns 0.
 */
int Thread::makePersistant(void)
{
    // Acquire the Mutex, change value, release

    flagLock.lock();
    persist = true;
    flagLock.unlock();

    return 0;
}


/**
 * stopPersistant makes a persistant Thread no longer persistant. It will exit
 * once its current task finishes execution.
 *
 * \return   stopPersistant always returns 0.
 */
int Thread::stopPersistant(void)
{
    /*
     * Stopping a persistant thread is a bit trickier. If the thread is currently
     * executing or not running, then it is sufficient to simply change the persist
     * flag. If, on the other hand, the thread is not currently executing a task,
     * then it is waiting for a new task to be assigned. In this case, the execute
     * flag should still be changed, but the persist condition variable should
     * also be signalled, which will break the task execution loop.
     */

    flagLock.lock();

    persist = false;

    if (running && !executing) {
        persistCond.broadcast();
    }

    flagLock.unlock();

    return 0;
}


/**
 * attachTask attaches a new task to be executed by the Thread. A new task can
 * be assigned to the Thread only if it is not currently executing a task. If the
 * Thread is not persistant, then a call to start() should be made after the
 * task is assigned, otherwise the task will begin execution immediately.
 *
 * If a task is attached to a Thread before it begins running and then another task
 * is attached to the Thread before execution begins, the second task will override
 * the first task, thus the second task will be executed.
 *
 * \param    task                New task to be executed
 * \return   0 if the task will be executed, -1 if the Thread is already executing
 *           a task.
 */
int Thread::attachTask(Threadable* task)
{
    /*
     * Two checks need to happen: first, check if the thread is running, if so,
     * then see if it is currently executing, if this is the case, then don't change
     * the task. If, however, it is not executing, then set the new task and signal
     * the condition variable to wake the thread up so it begins processing the new
     * task.
     * If the thread is not running, things are real simple, just change the task.
     */

    flagLock.lock();

    if (!executing) {
#ifdef DEBUG_THREAD
        std::cout << "New task assigned to the thread." << std::endl;
#endif

        objTask = task;

        if (running)   // send the signal that a task has been assigned, so start running again
        {
            persistCond.setPredicate(true);
            persistCond.broadcast();
        }
    }

    int retVal = executing ? -1 : 0;

    flagLock.unlock();

    if (retVal) {
        std::cerr << "Thread already executing a task wait until the task completes before attaching a new task."
                  << std::endl;
    }

    return retVal;
}


/**
 * start launches the Thread if it is not already running. A call to start when
 * the Thread is running does nothing.
 *
 * \return   0 if the Thread launches, -1 if creation fails.
 */
int Thread::start(void)
{
    /*
     * Create a thread that is either joinable or detached. If this fails, then
     * somebody has been going nuts with allocating new threads.
     */

    int success = -1;

    flagLock.lock();

    // Only start if the thread has not already been successfully started, indicated by the running flag
    if (!running) {
        // Create attributes with the appropriate detached value
        pthread_attr_t attributes;
        pthread_attr_init(&attributes);
        pthread_attr_setdetachstate(&attributes, (detached ? PTHREAD_CREATE_DETACHED : PTHREAD_CREATE_JOINABLE));

        success = pthread_create(&thread, &attributes, threadCaller, (void*)this);

        pthread_attr_destroy(&attributes);

        if (success == 0) {
            running = true;
        } else {
            std::cerr << "Unable to create a new thread." << std::endl;
        }
    } else   // running
    {
        success = 0;
    }

    flagLock.unlock();

    return (success ? -1 : 0);
}


/**
 * threadLoop is the loop that handles the execution of Threadable tasks. If
 * a Thread is marked as persistant, then threadLoop will continually run until
 * explicitly cancelled or the Thread is made unpersistant. threadLoop is called by
 * the threadStarter function, which is the actual function initially executed by
 * the call to pthread_create, but because that won't accept member functions, this
 * is my personal workaround.
 */
int Thread::threadLoop(void)
{
    /*
     * threadLoop operates by running in a continual loop and handling tasks as they
     * arrive. This is a slightly different way of doing things that the usual derived
     * class way, but it has the benefit that creating static threads is extremely
     * simple.
     *
     * The way this works is: Thread has a condition variable that it maintains internally
     * which allows the thread to more or less be turned on and off at will (well after
     * every execution of a task). If a Thread is set to be persistant, then after a
     * task finishes, the Thread will block on the condition variable waiting for a
     * new task. Once the new task arrives, signal the condition variable and bam,
     * the new task is flying. If not persistant, don't block on the condition variable
     * and just jump right out of the loop. Done, voila!
     */

#ifdef DEBUG_THREAD
    std::cout << "Going into main thread loop." << std::endl;
#endif

    while (true) {
        // Set executing to true, this will indirectly protect the current task
        // (yes this behavior is a bit shady, but such is life right?)
        flagLock.lock();
        executing = true;
        flagLock.unlock();

        // make sure there is a task to run, then just let it go until completion, however long that may end up being
        if (objTask != 0) {
#ifdef DEBUG_THREAD
            std::cout << "Beginning new task" << std::endl;
#endif

            objTask->run();

            // after returning there is no task, so reset the task variable to 0
            objTask = 0;
        }

        // now check to see if the Thread is persistant and should wait for new tasks,
        // or if it should just die a nice peaceful death
        flagLock.lock();
        executing = false;

        if (persist) {
#ifdef DEBUG_THREAD
            std::cout << "I am a persistant thread. I'm going to wait for a new task." << std::endl;
#endif

            flagLock.unlock();
            persistCond.setPredicate(false);
            persistCond.wait();

#ifdef DEBUG_THREAD
            std::cout << "I am a persistant thread. I was woken. Time to execute a new task." << std::endl;
#endif
        } else {
            running = false;
            flagLock.unlock();
            break;
        }
    }

#ifdef DEBUG_THREAD
    std::cout << "Thread execution complete!" << std::endl;
#endif

    return 0;
}


/**
 * threadCaller is a helper function for Thread that calls the main Thread processing
 * loop.
 *
 * \param    args            The arg is a Thread instance
 * \return   threadCaller always returns 0.
 */
void* threadCaller(void* args)
{
    // Just check that it is a valid pointer, then make the cast to Thread*

    if (args) {
#ifdef DEBUG_THREAD
        std::cout << "Thread Caller: Launching the thread" << std::endl;
#endif

        ((Thread*)args)->threadLoop();

#ifdef DEBUG_THREAD
        std::cout << "Thread finished" << std::endl;
#endif
    }

    return 0;
}

}   // namespace utils
}   // namespace vulcan

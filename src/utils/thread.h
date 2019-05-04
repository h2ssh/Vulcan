/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef THREAD_H
#define THREAD_H

#include <pthread.h>
#include <utils/mutex.h>
#include <utils/condition_variable.h>

namespace vulcan
{
namespace utils
{

/**
* thread.h provides a wrapper around the major functionality of the pthreads library.
* In particular, it provides a Thread class that allows for executing a function or
* member function in a thread, a Mutex class that provides a locking mechanism,
* a ConditionVariable class that handles the details of synchronizing via conditions,
* and a Threadable interface for classes to implement that provides a means for
* running the member functions of a class in a separate thread.
*/


/**
* Threadable is an interface that allows a class to be executed within a Thread.
* This is accomplished by an override of the run() member. This method is called
* when a Threadable object is passed to a Thread for execution.
*/
class Threadable
{
public:

    virtual ~Threadable() { }

protected:

    friend class Thread;

    /**
    * run is called by Thread when a Threadable is executed within a Thread. run
    * is a pure virtual function so it must be overriden in any derived classes.
    */
    virtual int run(void) = 0;
};


/**
* Thread is a wrapper around the pthreads library. It provides the major functionality
* needed to run a member function in a separate thread.
*
* Threads come in two varieties: detached and joinable. A detached thread is spawned
* and never returns again. The only communication between the parent thread and
* the new thread is through shared variables. A joinable thread, on the other hand,
* can be joined, which means that the parent thread can wait until the joinable thread
* finished execution before continuing on with further processing.
*
* Threads can also be marked to be persistant. A persistant thread does not die after
* the function that it was executing returns. Instead, once this happens the Thread
* goes into a suspended state, where it will remain until a new task is attached to it.
* Or until it is told to die. This is handy for use in thread pools or in parallel
* algorithms where having threads around to do some processing is handy.
*
* A class can be run within a thread by inheriting Threadable and overriding the
* run method. After this, an instance of the class can be passed to attachTask(),
* which will run the class once the thread is started.
*/
class Thread
{
public:

    /** Constructor for Thread. */
    Thread(bool joinable = false, bool persistant = false);

    /** Destructor for Thread. */
    ~Thread(void);

    // Accessors

    /**
    * isJoinable checks if the Thread was created to be joinable.
    *
    * \return   True if the Thread can be joined.
    */
    bool isJoinable(void) const { return !detached; }

    /**
    * isDetached checks if the Thread is detached.
    *
    * \return   True if the Thread is detached.
    */
    bool isDetached(void) const { return detached; }

    /**
    * isPersistant checks if the Thread has been flagged to be persistant.
    *
    * \return   True if the thread is persistant.
    */
    bool isPersistant(void) const
    {
        bool retVal = false;  // Create a copy of the value and return that
        flagLock.lock();
        retVal =  persist;
        flagLock.unlock();
        return retVal;
    }

    /**
    * isRunning checks if the Thread is currently executing, i.e. start() has been
    * called.
    *
    * \return   True if the Thread is currently running.
    */
    bool isRunning(void) const { return running; }

    /**
    * isExecuting checks to see if the Thread is currently executing a task.
    *
    * \return   True if a task is currently being executed.
    */
    bool isExecuting(void) const
    {
        bool retVal = false;  // Create a copy of the value and return that
        flagLock.lock();
        retVal =  executing;
        flagLock.unlock();
        return retVal;
    }

    // Meat-and-bones

    /** start begins execution of the Thread. */
    int start(void);

    /** detach takes a Thread that was potentially joinable and detaches it. */
    int detach(void);

    /** join executes a join operation on the Thread. */
    int join(void** retVal = 0);

    /** kill stops execution of the Thread. */
    int kill(void);

    /** makePersistant makes a Thread become persistant. */
    int makePersistant(void);

    /** stopPersistant stops a Thread from being persistant. */
    int stopPersistant(void);

    /** attachTask sets a new task to be executed by the Thread. */
    int attachTask(Threadable* task);

private:

    // A Thread should have no copies of itself exist, that would be a very bad thing
    Thread(const Thread& t) {}                     ///< Disable the copy constructor
    void operator=(const Thread& t) {}      ///< Disable the assignment operator

    int threadLoop(void);                     ///< Loop where all the action is happening
    friend void* threadCaller(void* args);  ///< Cheap hack to let me run threadLoop in a loop

    bool detached;                  ///< Flag to store whether this thread is joinable or has already been detached
    bool persist;                   ///< Flag indicating if this is to be a persistant thread, i.e. to be used in a static pool
    bool running;                   ///< Flag that indicates if the thread has been started
    bool executing;                 ///< Flag set to true when the Thread is executing a task
    bool joined;                    ///< Flag to indicate if join was called for the Thread
    mutable Mutex flagLock;                 ///< Mutex to keep all the flags from being corrupted

    // Pieces needed for persistancy
    ConditionVariable persistCond;  ///< Condition to keep the thread in persistant mode if requested

    Threadable* objTask;            ///< Current threadable task being executed

    pthread_t thread;               ///< The actual thread that is being controlled
};

void* threadCaller(void* args);


} // namespace utils
} // namespace vulcan

#endif //THREAD_H

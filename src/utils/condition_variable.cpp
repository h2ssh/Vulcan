/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <utils/condition_variable.h>
#include <errno.h>
#include <time.h>
#include <cstdint>

namespace vulcan
{
namespace utils
{

/**
* Constructor for ConditionVariable. The initial state of the predicate is controlled
* by the predicate parameter.
*
* \param    predicate           Value to initially assign to the predicate
*/
ConditionVariable::ConditionVariable(bool predicate) : predicateLock(true), refLock(true)
{
    // Both mutexes are created to be fast mutexes since their operation is very constrained
    // Otherwise, the predicate needs to be initialized and the refCount, then we're done

    this->predicate    = new bool;
    *(this->predicate) = predicate;

    refCount  = new int;
    *refCount = 1;

    // initialize the condition variable, the second parameter is ignored by linux, so don't worry about it
    condition = new pthread_cond_t;
    pthread_cond_init(condition, NULL);
}


/**
* Copy constructor for ConditionVariable. Creates a copy of ConditionVariable that represents
* the same condition and predicate. This has the nice effect of allowing multiple instances
* of ConditionVariable to be used in different threads with the same effect. Thus,
* a single, vulnerable pointer does not have to be shared all over the place which is
* a dangerous thing.
*
* \param    cv              ConditionVariable to be copied
*/
ConditionVariable::ConditionVariable(const ConditionVariable& cv) : predicateLock(cv.predicateLock), refLock(cv.refLock),
                                                                    condition(cv.condition), predicate(cv.predicate), refCount(cv.refCount)
{
    // Just increment the reference count
    refLock.lock();
    ++(*refCount);
    refLock.unlock();
}


/**
* Destructor for ConditionVariable. Since ConditionVariable is reference counted,
* the destructor only does something non-trivial, i.e. anything beside decrementing
* the reference count, when the reference count hits 0. At this point, all resources
* used by the ConditionVariable are freed.
*/
ConditionVariable::~ConditionVariable()
{
    // check the reference count -- free resources if it hits 0
    refLock.lock();
    --(*refCount);

    if(refCount && (*refCount == 0))
    {
        refLock.unlock();
        pthread_cond_destroy(condition);

        // don't forget to free these allocated resources, else we're fvcked for a clean memory space
        if(predicate)
        {
            delete predicate;
        }

        if(refCount)
        {
            delete refCount;
        }

        if(condition)
        {
            delete condition;
        }

        predicate = 0;
        refCount  = 0;
        condition = 0;
    }
    else  // still have references out there, so let them do their thing
    {
        refLock.unlock();
    }
}


/**
* broadcast sends a message to all threads currently waiting on the condition
* to wakeup and begin processing again.
*
* \return   Broadcast always returns 0.
*/
int ConditionVariable::broadcast()
{
    // Lock, signal, continue with life

    predicateLock.lock();
    pthread_cond_broadcast(condition);
    predicateLock.unlock();

    return 0;
}


/**
* signal wakes up exactly one thread waiting on the current condition. The thread
* that is awoken cannot be specified.
*
* \return   Signal always returns 0.
*/
int ConditionVariable::signal()
{
    // Lock, signal, continue with life

    predicateLock.lock();
    pthread_cond_signal(condition);
    predicateLock.unlock();

    return 0;
}


/**
* wait will cause a thread to block until signalled if the predicate in the ConditionVariable is
* false. If the predicate is set to true, then the call to wait will return immediately.
*
* \return   Wait always returns 0.
*/
int ConditionVariable::wait()
{
    // Lock, check predicate, maybe wait, then continue with life
    predicateLock.lock();

    while(!(*predicate))
    {
        pthread_cond_wait(condition, predicateLock.mutex);  // here's where being the friend comes in handy, get access to the raw mutex
    }

    predicateLock.unlock();

    return 0;
}


/**
* timedWait will cause a thread to block if the predicate in the ConditionVariable is false. The thread
* will block for the specified amount of time before waking up and continuing. The return value of the
* timedWait indicates the cause of the thread re-awakening.
*
* If the wait times out, the state of the predicate will be ignored and timedWait will return immediately.
* If a signal wakes the timedWait, then it will check the predicate like wait() and take a course
* of action based on the predicate.
*
* \param    millis              Number of milliseconds to wait for a signal
* \return   0 if the timedWait ends as the result of a signal, -1 if it times out.
*/
int ConditionVariable::timedWait(time_t millis)
{
    /*
    *  For the timed wait, the pthread_cond_timedwait requires an absolute time at which to finish. This
    * means getting the system time when called, then adding the delta on to that and issuing the wait
    * call. On return, a value of ETIMEDOUT indicates that the call timed out, so return that rather
    * than checking the predicate again.
    */

    predicateLock.lock();

    int result = 0;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    
    int64_t nanoseconds = ts.tv_nsec + (millis * 1000000ll);

    ts.tv_sec  += nanoseconds / 1000000000;
    ts.tv_nsec =  nanoseconds % 1000000000;

    while(!(*predicate) && (result == 0))
    {
        result = pthread_cond_timedwait(condition, predicateLock.mutex, &ts);
    }

    predicateLock.unlock();

    return (result == 0) ? 0 : -1;
}


/**
* setPredicate sets the value of the predicate associated with the ConditionVariable.
*
* \param    pred            New predicate value
*/
void ConditionVariable::setPredicate(bool pred)
{
    // Lock, set, forget
    predicateLock.lock();
    *predicate = pred;
    predicateLock.unlock();
}

}
}

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UTILS_CONDITION_VARIABLE_H
#define UTILS_CONDITION_VARIABLE_H

#include "utils/mutex.h"
#include <pthread.h>

namespace vulcan
{
namespace utils
{

/**
 * ConditionVariable is a wrapper around the condition variable routines contained
 * in the pthreads library. Condition variables are useful in synchronizing the
 * operation of threads. In particular, they allow for a thread to wait until,
 * surprise, some condition is triggered by another thread. The other thread will
 * broadcast out saying for any threads waiting on that connection to wakeup, at which
 * point one of them does, and continues processing, while the others remain blocked.
 *
 * Central to a condition variable working properly is having a predicate to indicate
 * whether or not to wait. This predicate is shared by all threads blocking on a condition.
 * The default value for the predicate is supplied on creation. The value of the predicate
 * can be changed by any thread that is using the ConditionVariable in a safe manner.
 *
 * A predicate value of true means that a thread can continue processing. A predicate value
 * of false indicates that the thread should block.
 *
 * IMPORTANT: In order for the ConditionVariable to work properly, all threads that
 * are to be synchronized must be using the same ConditionVariable. This can be achieved
 * in a few ways. First, all classes/threads share a single instance of ConditionVariable
 * via a pointer. Given that ConditionVariable is thread-safe, this is a decent way
 * of doing things, but what if someone accidentally deletes the pointer, then everyone
 * is screwed. The other, and better, way of accomplishing the task of sharing a condition
 * is to create copies of the ConditionVariable wherever needed by using copy constructor,
 * which will return a ConditionVariable that will be properly shared with the
 * current ConditionVariable.
 */
class ConditionVariable
{
public:
    /** Constructor for ConditionVariable. */
    ConditionVariable(bool predicate = true);

    /** Copy constructor for ConditionVariable. */
    ConditionVariable(const ConditionVariable& cv);

    /** Destructor for ConditionVariable. */
    ~ConditionVariable();


    // Condition variable fun

    /** broadcast sends a general broadcast to all threads waiting on the ConditionVariable. */
    int broadcast();

    /** signal wakes up a single waiting thread. */
    int signal();

    /** wait will cause the thread to block if the predicate is not true. */
    int wait();

    /** timedWait will wait for a condition variable for the specified amount of time. */
    int timedWait(time_t millis);

    /** setPredicate changes the value of the predicate. */
    void setPredicate(bool value);

private:
    void operator=(const ConditionVariable& cv) {
    }   ///< Disable assignment for now, it opens up a weird bag of worms that I'd not like to think of

    Mutex predicateLock;   ///< Mutex lock to be used by the ConditionVariable for signalling/waiting
    Mutex refLock;         ///< Lock for the reference counter, so it doesn't get borked

    pthread_cond_t* condition;   ///< Actually data member that we're happily abstracting

    bool* predicate;   ///< Predicate pointer to be shared amongst all those sharing this ConditionVariable
    int* refCount;   ///< Count of the number of instances of ConditionVariable associate with this particular condition
};


}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_CONDITION_VARIABLE_H

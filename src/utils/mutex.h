/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UTILS_MUTEX_H
#define UTILS_MUTEX_H

#include <pthread.h>

namespace vulcan
{
namespace utils
{

/**
 * Mutex is a wrapper around the mutex functionality provided in the pthreads library.
 * A mutex (MUTual EXclusion) is a thread synchronization device that allows for
 * locking critical sections of code, i.e. code that involves variables shared by
 * multiple threads. A mutex provides this by allowing for an atomic read/write to
 * be performed. This atomicity allows for the mutex to determine if a value is locked
 * or not in a single CPU instruction. This means that it is impossible for two
 * threads to acquire (i.e. lock) the mutex at the same time. The result of this
 * is something nice: a guarantee of read/write safety for sections of code contained
 * within the protected sector, i.e. the portions between the mutex lock and unlock.
 * Of course, all sections of code the access shared variables need to encapsulate
 * critical regions with the same mutex. With mutexes, there is always a danger of
 * deadlock, i.e. mutexes in separate threads block access to portions of code that
 * they need to access in order to unlock from each other. Thus, the program is stuck
 * and can do no more. As they say, with great power comes great responsibility.
 *
 * Mutex provides three operations: lock, unlock, and trylock. Lock will lock the mutex
 * or block until it can acquire the mutex. Unlock releases the mutex so it can be
 * acquired by other threads. Trylock checks the mutex to see if it is locked, if the
 * mutex is unlocked, then trylock will acquire the mutex, otherwise it will not acquire
 * the mutex and return a value saying that the mutex was locked--this is a non-blocking call.
 *
 * For the Mutex to be effective, all functions/classes that are trying to protect
 * critical sections of code need to use the same mutex. There are two ways of accomplishing
 * this. First, all threads can share a single pointer to a Mutex. This is valid, but
 * carries the danger that one of the threads deletes the Mutex and leaves everyone
 * else hanging. The better way is to create copies of the Mutex using the copy
 * constructor, which provides the same Mutex in a new instance so clobbering the mutex
 * will not happen.
 *
 * There are two possible types of Mutexes, fast and recursive. The fast mutex provides
 * for very fast locking by jumping right in and checking the lock and blocking if
 * the need arises. This has a deleterious effect of causing a deadlock if a thread
 * that already holds the lock on a Mutex tries to lock the Mutex again. This can be
 * avoided by creating a recusrive mutex, which allows a single thread to lock a mutex
 * many times as long as it unlocks it an equal number of times (i.e. properly releases
 * the mutex when finished). The recursive mutex is slow, so hedge your bets on which
 * mutex to create based on the expected usage.
 */
class Mutex
{
public:
    /** Constructor for Mutex. */
    Mutex(bool fast = true);

    /** Copy constructor for Mutex. */
    Mutex(const Mutex& m);

    /** Assignment operator for Mutex. */
    Mutex& operator=(const Mutex& m);

    /** Destructor for Mutex. */
    ~Mutex();

    // Mutex-y fun

    /** lock the mutex. Or block until the lock is acquired. */
    int lock();

    /** unlock the mutex. */
    int unlock();

    /** trylock on the mutex. */
    int trylock();

private:
    friend class ConditionVariable;   ///< ConditionVariable needs access to the raw mutex

    int* refCount;   ///< Count of currently active references to the mutex

    pthread_mutex_t* mutex;     ///< Nitty-gritty little mutex!
    pthread_mutex_t* refLock;   ///< Lock for the reference counter so it doesn't get clobbered
};

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_MUTEX_H

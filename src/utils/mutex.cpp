/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <errno.h>
#include "utils/mutex.h"


using vulcan::utils::Mutex;


// Definitions for the Mutex class

/**
* Constructor for Mutex. The Mutex is initialized based on the fast parameter.
*
* \param    fast            Flag set to true if the Mutex to be created is a fast mutex
*/
Mutex::Mutex(bool fast)
{
    // A few things need to be created, namely the external and internal mutexes
    // as well as creating the refCounters

    refCount  = new int;
    *refCount = 1;   // initialize to 1 because we're the only thing pointing to it at the moment

    mutex   = new pthread_mutex_t;
    refLock = new pthread_mutex_t;

    // For now, don't allow recursive mutexes
    pthread_mutex_init(mutex, 0);

    // the reference count mutex is a simple fast mutex, so nothing elaborate needs to happen
    pthread_mutex_init(refLock, 0);
}


/**
* Copy constructor for Mutex. Creates a copy of Mutex. The copy created wraps the
* same mutex as the original, thus a lock on the copy will lock the original as well.
* This is a desirable property because multiple Mutex objects used in multiple classes
* can be separate objects rather than just pointers to the same object which is rather
* dangerous.
*
* \param    m           Mutex to copy
*/
Mutex::Mutex(const Mutex& m) : refCount(m.refCount), mutex(m.mutex), refLock(m.refLock)
{
    // All that needs to happen is incrementing the reference counter by 1, everything
    // else is handled in the initializer list
    pthread_mutex_lock(refLock);
    ++(*refCount);
    pthread_mutex_unlock(refLock);
}


/**
* Assignment operator for Mutex. Creates a duplicate of the Mutex on the right-hand side.
* The copy is such that a lock on the lvalue will lock the Mutex in the rvalue.
*
* \param    rhs             Rvalue for the '=' operation
* \return   Reference to mutex copy
*/
Mutex& Mutex::operator=(const Mutex& rhs)
{
    refCount = rhs.refCount;
    mutex    = rhs.mutex;
    refLock  = rhs.refLock;

    pthread_mutex_lock(refLock);
    ++(*refCount);
    pthread_mutex_unlock(refLock);

    return *this;
}


/**
* Destructor for Mutex. Mutex is a reference-counted object, so once the references
* go to 0, then the mutex is freed, until then destroying a mutex object does nothing
* more than decrease the reference count.
*/
Mutex::~Mutex()
{
    // Check to see if the reference count is zero, if so, time for the Mutex to
    // be completely obliterated
    if(refLock != 0)
    {
        pthread_mutex_lock(refLock);

        --(*refCount);

        #ifdef DEBUG_MUTEX
            std::cout<<"Destructing Mutex: Reference count: "<<*refCount<<std::endl;
        #endif

        if(refCount && (*refCount == 0))
        {
            // make sure the mutexes are unlocked before attempting to destroy them
            pthread_mutex_unlock(refLock);
            pthread_mutex_unlock(mutex);
            pthread_mutex_destroy(refLock);
            pthread_mutex_destroy(mutex);

            // cleanup our small bit of memory
            if(refCount)
            {
                delete refCount;
            }

            if(refLock)
            {
                delete refLock;
            }

            if(mutex)
            {
                delete mutex;
            }

            // See all the pointers to zero, just to be safe
            refCount = 0;
            refLock  = 0;
            mutex    = 0;

            #ifdef DEBUG_MUTEX
                std::cout<<"No more references: Mutex Destroyed!"<<std::endl;
            #endif
        }
        else  // if a reference still exists, then just unlock the mutex and do no more
        {
            pthread_mutex_unlock(refLock);
        }
    }
}


/**
* lock attempts to acquire a lock on the Mutex. If the Mutex is currently locked,
* then lock will block until the lock is released and can be acquired by Mutex.
*
* \return   Lock always returns 0.
*/
int Mutex::lock()
{
    pthread_mutex_lock(mutex);

    return 0;
}


/**
* unlock releases the lock the thread has on the Mutex. A call to unlock MUST
* ONLY OCCUR AFTER A CALL TO LOCK. If this is not the case, then the thread may
* be unlocking the mutex prematurely and exposing a critical section to potential
* concurrency issues.
*
* \return   Unlock always returns 0.
*/
int Mutex::unlock()
{
    pthread_mutex_unlock(mutex);

    return 0;
}


/**
* trylock attempts to acquire a lock on the Mutex. If the lock attempt would block,
* then trylock returns, rather than blocking.
*
* \return   1 if the lock is acquired, 0 otherwise.
*/
int Mutex::trylock()
{
    int retVal = pthread_mutex_trylock(mutex);

    return (retVal == EBUSY) ? 0 : 1;
}

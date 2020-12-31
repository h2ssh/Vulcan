/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UTILS_AUTO_MUTEX_H
#define UTILS_AUTO_MUTEX_H

#include "utils/mutex.h"

namespace vulcan
{
namespace utils
{

/**
 * AutoMutex applies the RTTI idiom to a Mutex. The AutoMutex treats the mutex lock
 * as a resource. The lock is acquired when the AutoMutex is created and unlocked
 * when the AutoMutex is destroyed. Using the AutoMutex helps ensure that a lock
 * is never accidentally left locked, as it will only remain locked within the
 * scope in which it is declared.
 */
class AutoMutex
{
public:
    AutoMutex(Mutex& mutexToLock) : mutex(mutexToLock) { mutex.lock(); }

    ~AutoMutex(void) { mutex.unlock(); }

private:
    Mutex& mutex;
};

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_AUTO_MUTEX_H

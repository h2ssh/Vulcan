/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     locked_queue.h
* \author   Collin Johnson
* 
* Definition of LockedQueue template.
*/

#ifndef UTILS_LOCKED_QUEUE_H
#define UTILS_LOCKED_QUEUE_H

#include "utils/mutex.h"
#include "utils/auto_mutex.h"
#include <atomic>
#include <cassert>
#include <deque>

namespace vulcan
{
namespace utils
{
    
/**
* LockedQueue is a simple locked queue. All access to the underlying queue is locked. Every
* member function that reads or modifies the queue acquires an internally held lock. The LockedQueue
* can be used in simple producer-consumer scenarios with one producer and one consumer, or in more
* complex multi-threaded solutions. It isn't optimized for any particular scenario. If more than two
* threads are using the queue, beware because calls to front() and pop() could get interleaved such that
* a front() is called on an empty queue, which leads to undefined behavior.
* 
* Unlike std::queue, the LockedQueue front() method assigns a parameter value to be the front. This is used
* because a sequence of empty(), front(), and pop() with the normal queue interface could result in attempting
* to retrieve the front() an empty queue.
*/
template <typename T>
class LockedQueue
{
public:
    
    /**
    * Default constructor for LockedQueue.
    */
    LockedQueue(void)
    : queueSize(0)
    , isEmpty(true)
    {
        // should I do something to queue?
    }
    
    /**
    * Constructor for LockedQueue.
    * 
    * Place the value as the initial member of the queue.
    */
    LockedQueue(const T& initial)
    : queueSize(0)
    , isEmpty(false)
    {
        queue.push_back(initial);
    }
    
    /**
    * Copy constructor for LockedQueue.
    */
    LockedQueue(const LockedQueue& rhs)
    {
        utils::AutoMutex rhsLock(rhs.lock);        
        queue     = rhs.queue;
        queueSize = rhs.queueSize;
        isEmpty   = rhs.isEmpty;
    }
    
    /**
    * Move constructor for LockedQueue.
    */
    LockedQueue(LockedQueue&& rhs)
    {
        utils::AutoMutex rhsLock(rhs.lock);        
        queue.swap(rhs.queue);
        queueSize = rhs.queueSize;
        isEmpty   = rhs.isEmpty;
    }
    
    /**
    * size retrieves the current size of the queue.
    */
    std::size_t size(void) const { return queueSize; }
    
    /**
    * empty checks if the queue is empty.
    */
    bool empty(void) const { return isEmpty; }
    
    /**
    * clear erases all values in the queue.
    */
    void clear(void)
    {
        utils::AutoMutex queueLock(lock);
        queue.clear();
        queueSize = 0;
        isEmpty = true;
    }
    
    /**
    * front retrieves the value at the front of the queue.
    * 
    * \pre !empty()
    * \return   Value at the front of the queue.
    */
    T front(void) const
    {
        utils::AutoMutex queueLock(lock);
        assert(!queue.empty());
        T front = std::move(queue.front());
        return front;
    }
    
    /**
    * back retrieves the value at the back of the queue. If the queue is empty, then value will remain unassigned.
    * 
    * \param[out]   value   Value at the back of the queue, if there was one.
    * \return   True if the queue wasn't empty, so there was a value at the back.
    */
    bool back(T& value) const
    {
        utils::AutoMutex queueLock(lock);
        if(!queue.empty())
        {
            value = queue.back();
            return true;
        }
        
        return false;
    }
    
    /**
    * push pushes a new value onto the back of the queue.
    */
    void push(const T& value)
    {
        utils::AutoMutex queueLock(lock);
        queue.push_back(value);
        ++queueSize;
        isEmpty = false;
    }
    
    void push(T&& value)
    {
        utils::AutoMutex queueLock(lock);
        queue.push_back(value);
        ++queueSize;
        isEmpty = false;
    }
    
    /**
    * pop removes the value from the front of the queue if the queue isn't empty. Otherwise, it has no effect.
    */
    void pop(void)
    {
        utils::AutoMutex queueLock(lock);
        if(!queue.empty())
        {
            queue.pop_front();
            --queueSize;
        }
        isEmpty = queue.empty();
    }
    
private:
    
    std::deque<T>     queue;
    std::atomic<int>  queueSize;
    std::atomic<bool> isEmpty;
    
    mutable utils::Mutex lock;
};
    
} // namespace utils
} // namespace vulcan

#endif // UTILS_LOCKED_QUEUE_H

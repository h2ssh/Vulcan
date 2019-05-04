/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     locked_double_buffer.h
* \author   Collin Johnson
*
* Definition of LockedDoubleBuffer, which allows easy double buffering of
* data incoming to a module along with a simple RAII object that will swap the buffer on destruction, thereby allowing
* a buffered value to be directly returned from a function without copying it first.
*/

#ifndef UTILS_LOCKED_DOUBLE_BUFFER_H
#define UTILS_LOCKED_DOUBLE_BUFFER_H

#include <utils/mutex.h>
#include <utils/auto_mutex.h>
#include <algorithm>
#include <atomic>

namespace vulcan
{
namespace utils
{

/**
* LockedDoubleBuffer is a thread-safe double buffer for storing incoming data in one buffer
* while reading data from the other buffer. The general usage pattern is this:
*
*   1) Wait until hasData is true, then swap the buffers.
*   2) Read data from one thread
*   3) Write data from a different thread
*   4) Go back to 1.
*
* The locking mechanism, as currently implemented, expects read() and swapBuffers() calls to be
* synchronous and occurring from the same thread. This assumption allows reading and writing to
* occur safely on separate threads without blocking each other.
*/
template <typename T>
class LockedDoubleBuffer
{
public:

    /**
    * Constructor for LockedDoubleBuffer.
    */
    LockedDoubleBuffer(void)
    : readIndex_(0)
    , writeIndex_(1)
    , haveData_(false)
    {
    }

    /**
    * write writes new data into the buffer.
    *
    * \param    data        Data to be added
    */
    void write(const T& data)
    {
        utils::AutoMutex autoLock(indexLock);
        buffer_[writeIndex_] = data;
        haveData_ = true;
    }

    /**
    * read reads the latest data from the buffer. If no data has been written yet,
    * the returned data might be stale. Check for new data via the hasData() method.
    *
    * \return   Data in the read buffer.
    */
    T& read(void) { return buffer_[readIndex_]; }

    /**
    * read reads the latest data from the buffer. If no data has been written yet,
    * the returned data might be stale. Check for new data via the hasData() method.
    *
    * \return   Data in the read buffer.
    */
    const T& read(void) const { return buffer_[readIndex_]; }

    /**
    * swapBuffers swaps the read and write buffers.
    */
    void swapBuffers(void)
    {
        utils::AutoMutex autoLock(indexLock);
        std::swap(readIndex_, writeIndex_);
        haveData_ = false;
    }

    /**
    * hasData checks to see if the buffer has received new data.
    */
    bool hasData(void) const { return haveData_.load(); }
    
    // Operator overloads to allow a double-buffered type to be used like a regular-ish type
    
    /**
    * Assignment operator overload that writes to the buffer. It does not return a value however because the write
    * result is only accessible after a swap and read.
    */
    void operator=(const T& data)
    {
        write(data);
    }
    
    /**
    * Implicit conversion operator to T. Allow the value to be implicitly converted to a T&.
    */
    operator const T&(void) const
    {
        return read();
    }
    
    /**
    * Pointer-like access to the current read object.
    */
    const T* operator->(void) const
    {
        return &read();
    }
    
    T* operator->(void)
    {
        return &read();
    }

private:

    T                 buffer_[2];
    int               readIndex_;
    int               writeIndex_;
    std::atomic<bool> haveData_;
    utils::Mutex      indexLock;
};

/**
* SwapOnExit is an RAII class for swapping a double buffer on scope-exit. This approach allows for creating this object
* and then returning the result of read() or the implicit conversion operation directly from a function, i.e.
* 
*   T func()
*   {
*       SwapOnExit<T> swap(buffer);  // buffer type = LockedDoubleBuffer<T>
*       return buffer;
*   }
* 
*   as opposed to creating a temporary:
* 
*   T func()
*   {
*       T temp = buffer;
*       buffer.swapBuffer();
*       return temp;
*    }
* 
* Note that a return-by-value is the only way that this operation is safe.
*/
template<typename T>
class SwapOnExit
{
public:
    
    SwapOnExit(LockedDoubleBuffer<T>& buffer)
    : buf_(buffer)
    {
    }
    
    ~SwapOnExit(void)
    {
        buf_.swapBuffers();
    }
    
private:
    
    LockedDoubleBuffer<T>& buf_;
};


} // namespace utils
} // namespace vuclan

#endif // UTILS_LOCKED_DOUBLE_BUFFER_H

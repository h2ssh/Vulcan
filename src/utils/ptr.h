/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     ptr.h
* \author   Collin Johnson
* 
* Definition of some utilities for dealing with pointers and smart pointers:
* 
*   - SharedToRawComp : functor to use with std::find_if() to find the raw pointer associated with a collection of shared_ptr.
*   - UniqueToRawComp : functor to use with std::find_if() to find the raw pointer associated with a collection of unique_ptr.
* 
*   - smart_to_raw_vector   : converts a collection of smart points to a vector of raw pointers
* 
*   - SmartToRawIter : type to allow iteration over a container that returns the result of .get() for the smart pointer
*/

#ifndef UTILS_PTR_H
#define UTILS_PTR_H

#include <boost/iterator/transform_iterator.hpp>
#include <algorithm>
#include <memory>
#include <vector>

namespace vulcan
{
namespace utils
{
    
template <typename T, class SmartPtr>
struct SmartToRawComp
{
    SmartToRawComp(const T* raw)
    : rawPtr(raw)
    {
    }
    
    bool operator()(const SmartPtr& ptr) const
    {
        return ptr.get() == rawPtr;
    }
    
private:
    
    const T* rawPtr;
};

template <typename T>
using SharedToRawComp = SmartToRawComp<T, std::shared_ptr<T>>;

template <typename T>
using UniqueToRawComp = SmartToRawComp<T, std::unique_ptr<T>>;


template <class T, class SmartIter>
std::vector<T*> smart_to_raw_vector(SmartIter begin, SmartIter end)
{
    using Value = typename std::iterator_traits<SmartIter>::value_type;
    
    std::vector<T*> raw(std::distance(begin, end));
    std::transform(begin, end, raw.begin(), [](const Value& ptr) { return ptr.get(); });
    return raw;
}

template <typename T,
          class SmartPtr = std::unique_ptr<T>>
struct SmartGetFunc
{
    T* operator()(const SmartPtr& ptr) const
    {
        return ptr.get();
    }
    
    typedef T* result_type;
};

template <typename T,
          class SmartPtr = std::unique_ptr<T>>
struct SmartGetConstFunc
{
    const T* operator()(const SmartPtr& ptr) const
    {
        return ptr.get();
    }
    
    typedef const T* result_type;
};

/**
* SmartToRawIter is a transform iterator that converts a Containter<SmartPtr>::iterator to the associated
* raw pointer value. For example:
* 
*   auto it = SmartToRawIter<int>(vec.begin(), SmartGetFunc());
*   
*   Type of *it = int*
*/
template <typename T,
          class SmartPtr = std::unique_ptr<T>,
          class Container = std::vector<SmartPtr>>
using SmartToRawIter = boost::transform_iterator<SmartGetFunc<T, SmartPtr>, typename Container::iterator>;

/**
* SmartToRawIter is a transform iterator that converts a Containter<SmartPtr>::const_iterator to the associated
* raw pointer value. For example:
* 
*   auto it = SmartToRawConstIter<int>(vec.begin(), SmartGetConstFunc());
*   
*   Type of *it = const int*
*/
template <typename T,
          class SmartPtr = std::unique_ptr<T>,
          class Container = std::vector<SmartPtr>>
using SmartToRawConstIter = boost::transform_iterator<SmartGetConstFunc<T, SmartPtr>, typename Container::const_iterator>;

} // namespace utils
} // namespace vulcan

#endif // UTILS_PTR_H

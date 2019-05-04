/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     filters.h
* \author   Collin Johnson
* 
* Definition of filters for filtering values in a sequence.
* 
* The basic VectorFilter type is any Callable entity with signature:
* 
*   double(Iterator begin, Iterator end)
* 
* Defined filters here are:
* 
*   - MeanFilter    : find the mean of the range
*   - MedianFilter  : find the median of the range
*/

#ifndef MATH_FILTERS_H
#define MATH_FILTERS_H

#include <math/statistics.h>
#include <functional>
#include <vector>

namespace vulcan
{
namespace math
{

/**
* VectorFilter defines the signature for any filter used on a range of values.
*/
template <typename T>
using VectorFilter = std::function<T(typename std::vector<T>::const_iterator, typename std::vector<T>::const_iterator)>;


/**
* MeanFilter is a SequenceFilter that returns the mean of the value range.
*/
template <typename T>
struct MeanFilter
{
    T operator()(typename std::vector<T>::const_iterator begin, typename std::vector<T>::const_iterator end)
    {
        return mean(begin, end);
    }
};

/**
* MedianFilter is a SequenceFilter that returns the median of the value range.
*/
template <typename T>
struct MedianFilter
{
    T operator()(typename std::vector<T>::const_iterator begin, typename std::vector<T>::const_iterator end)
    {
        std::vector<T> values(begin, end);
        return median(values.begin(), values.end());
    }
};


}
}

#endif // MATH_FILTERS_H

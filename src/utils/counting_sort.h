/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UTILS_COUNTING_SORT_H
#define UTILS_COUNTING_SORT_H

#include <cstring>

namespace vulcan
{
namespace utils
{

/**
* counting_sort is a non-comparison sort, which sorts a range of integers [0, K]. The idea
* is to count the number of occurrences of each integer. Then a cumulative total of the
* occurrences can be created. The data is looked at again and the index in the count array
* tells exactly where the next element should go.
* 
* Right now, each element being sorted is assumed to have a .value parameter. This condition
* is extremely limiting for certain. Needs to be fixed by allowing a functor to be provided
* for accessing the numeric value associated with the particular values being sorted
* 
* See CLRS Section 8.2 for details and a proof of correctness.
* 
* \param    K               Template parameter specifying the maximum value to be considered
* \param    begin           Starting iterator for the data
* \param    end             End iterator for the data
* \param    outputBegin     Starting iterator for the output
* \pre      outputBegin is the start iterator for a data structure that can hold at least end-begin values.
* \pre      begin, end, and outputBegin are bi-directional, random-access iterators.
*/
template <int K, typename T>
void counting_sort(T begin, T end, T outputBegin)
{
    /*
    * Simple steps:
    * 
    * 0) Set all counts to 0.
    * 1) Count the number of each value for the data.
    * 2) Get the cumulative values for the counts.
    * 3) Copy the values to the output.
    */
    
    int counts[K];
    
    memset(counts, 0, K*sizeof(int));
    
    for(T countIt = begin; countIt != end; ++countIt)
    {
        ++counts[countIt->value];
    }
    
    for(int n = 1; n < K; ++n)
    {
        counts[n] += counts[n-1];
    }
    
    for(T copyIt = end-1; copyIt >= begin; --copyIt)
    {
        *(outputBegin+counts[copyIt->value]-1) = *copyIt;
        --counts[copyIt->value];
    }
}


}
}

#endif // UTILS_COUNTING_SORT_H

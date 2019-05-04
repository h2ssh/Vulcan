/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     timed_sequence.h
* \author   Collin Johnson
* 
* Definition of TimedSequence class template.
*/

#ifndef UTILS_TIMED_SEQUENCE_H
#define UTILS_TIMED_SEQUENCE_H

#include <utils/algorithm_ext.h>
#include <algorithm>
#include <vector>
#include <cstdint>

namespace vulcan
{
namespace utils
{

/**
* TimedSequence holds a timestamped sequence of data. The data is assumed to have a .timestamp() method for now. In the
* future, SFINAE might be used to allow either a .timestamp member or a .timestamp() method.
* 
* The data is sorted in order of ascending time. Values can be erased based on time values.
* 
* Requirements:
* 
*   - T must be default constructible
*/
template <class T>
class TimedSequence
{
public:
    
    using const_iterator = typename std::vector<T>::const_iterator;
    
    /**
    * nearest_value retrieves the value in the sequence nearest to the provided time.
    */
    T nearest_value(int64_t time) const
    {
        if(empty())
        {
            return T();
        }
        
        int64_t lastDist = std::abs(data_.front().timestamp() - time);
        
        // Data is sorted in order of increasing time, so once the time gap is larger than the previous time gap,
        // the previous value should be returned
        for(std::size_t n = 1; n < data_.size(); ++n)
        {
            int64_t dist = std::abs(data_[n].timestamp() - time);
            if(dist > lastDist)
            {
                return data_[n-1];
            }
            lastDist = dist;
        }
        
        return data_.back();
    }
    
    /**
    * push_back adds new data to the sequence.
    */
    void push_back(const T& t)
    {
        data_.push_back(t);
        if((data_.size() > 1) && (data_[data_.size()-2].timestamp() > data_.back().timestamp()))
        {
            std::sort(data_.begin(), data_.end(), [](const T& lhs, const T& rhs) {
                return lhs.timestamp() < rhs.timestamp();
            });
        }
    }

    /**
    * erase_before erases all values before the given time.
    */
    int erase_before(int64_t time)
    {
        return utils::erase_remove_if(data_, [time](const T& value) {
            return value.timestamp() < time;
        });
    }
    
    /**
    * erase_after erases all values after the given time.
    */
    int erase_after(int64_t time)
    {
        return utils::erase_remove_if(data_, [time](const T& value) {
            return value.timestamp() > time;
        });
    }
    
    // Iterator support
    bool empty(void ) const { return data_.empty(); }
    std::size_t size(void) const { return data_.size(); }
    const_iterator begin(void) const { return data_.begin(); }
    const_iterator end(void) const { return data_.end(); }
    T front(void) const { return data_.front(); }
    T back(void) const { return data_.back(); }
    T operator[](int index) const { return data_[index]; }
    T at(int index) const { return data_.at(index); }
    
private:
    
    // INVARIANT: data_ is always stored in order of increasing time.
    std::vector<T> data_;
};

} // namespace utils
} // namespace vulcan

#endif // UTILS_TIMESTAMPED_SEQUENCE_H

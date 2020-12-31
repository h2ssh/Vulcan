/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     fixed_duration_buffer.h
 * \author   Collin Johnson
 *
 * Definition of class template FixedDurationBuffer that keeps a data with a fixed duration from front to back.
 */

#ifndef UTILS_FIXED_DURATION_BUFFER_H
#define UTILS_FIXED_DURATION_BUFFER_H

#include <algorithm>
#include <deque>
#include <iterator>
#include <utility>

namespace vulcan
{
namespace utils
{

/**
 * FixedDurationBuffer is a buffer of data that stores a fixed duration of data. The buffer maintains a continuous
 * buffer of data holds the following variant:
 *
 *   - The stored values are in order of ascending timestamp
 *   - The amount of data stored is the minimal amount needed such that back().timestamp - front().timestamp >=
 * buffer_duration
 *   - When a value is pushed, 0 or more values are popped off the front with the condition that if another value is
 *       popped, then the duration invariant would no longer hold. Once the stored duration of values goes above the
 *       buffer_duration, then
 *
 * Concept:
 *   T is TimedValue
 *       - T must have a .timestamp member
 */
template <class TimedValue>
class FixedDurationBuffer
{
public:
    // Types needed for STL-compliance
    using value_type = typename std::deque<TimedValue>::value_type;
    using reference = typename std::deque<TimedValue>::reference;
    using const_reference = typename std::deque<TimedValue>::const_reference;
    using iterator = typename std::deque<TimedValue>::iterator;
    using const_iterator = typename std::deque<TimedValue>::const_iterator;
    using difference_type = typename std::deque<TimedValue>::difference_type;
    using size_type = typename std::deque<TimedValue>::size_type;

    /**
     * Constructor for FixedDurationBuffer.
     *
     * \param    durationUs          Duration in microseconds of data to maintain in the buffer (optional, default =
     * 1000000us)
     */
    explicit FixedDurationBuffer(int64_t durationUs = 1000000) : duration_(durationUs) { }

    // Use default constructors
    FixedDurationBuffer(const FixedDurationBuffer<TimedValue>& rhs) = default;
    FixedDurationBuffer<TimedValue>& operator=(const FixedDurationBuffer<TimedValue>& rhs) = default;

    FixedDurationBuffer(FixedDurationBuffer<TimedValue>&& rhs) = default;
    FixedDurationBuffer<TimedValue>& operator=(FixedDurationBuffer<TimedValue>&& rhs) = default;

    /**
     * bufferDuration retrieves the duration of data the buffer is maintaining in microseconds.
     */
    int64_t bufferDuration(void) const { return duration_; }

    /**
     * storedDuration retrieves the duration of data currently stored in the buffer in microseconds.
     *
     * Equivalent of back().timestamp - front().timestamp.
     */
    int64_t storedDuration(void) const { return data_.empty() ? 0 : back().timestamp - front().timestamp; }

    /**
     * isFull checks if the buffer has filled up and represents the desired fixed duration. Whenever full, a push
     * operation is likely (though not always guaranteed) to throw away the first measurement in the buffer.
     *
     * isFull occurs when: storedDuration >= bufferDuration
     */
    bool isFull(void) const { return storedDuration() >= bufferDuration(); }

    /**
     * timestamp retrieves the timestamp of the most recent measurement in the buffer.
     */
    int64_t timestamp(void) const { return data_.empty() ? 0 : data_.back().timestamp; }

    /**
     * push pushes a new TimedValue into the buffer.
     *
     * If value.timestamp > back().timestamp, the value is guaranteed to be stored. Otherwise, the value may no be
     * added because the buffer maintains only the most recent duration of data.
     */
    template <class Value>
    void push(Value&& value)
    {
        // If the value's timestamp comes after the last stored, then just push it back
        if (data_.empty() || (data_.back().timestamp < value.timestamp)) {
            data_.push_back(value);
        }
        // Otherwise push it and sort by time
        else {
            data_.push_back(value);
            std::sort(data_.begin(), data_.end(), [](const TimedValue& lhs, const TimedValue& rhs) {
                return lhs.timestamp < rhs.timestamp;
            });
        }

        // Search until the first element where the duration is too small is found
        auto startIt = std::find_if(data_.rbegin(), data_.rend(), [this](const TimedValue& rhs) {
            return (data_.back().timestamp - rhs.timestamp) >= duration_;
        });

        // If the duration is anything other than the full buffer, erase all the front elements
        if (startIt != data_.rend()) {
            data_.erase(data_.begin(), std::next(startIt).base());
        }
    }


    // Values stored in the buffer cannot be modified, so only constant iterators are available
    const_iterator begin(void) const { return data_.begin(); }
    const_iterator end(void) const { return data_.end(); }

    const_iterator cbegin(void) const { return data_.cbegin(); }
    const_iterator cend(void) const { return data_.cend(); }

    // Access the front and back
    const_reference front(void) const { return data_.front(); }
    const_reference back(void) const { return data_.back(); }

    size_type size(void) const { return data_.size(); }
    bool empty(void) const { return data_.empty(); }

    const_reference operator[](int index) { return data_[index]; }
    const_reference operator[](int index) const { return data_[index]; }

    void clear(void) { data_.clear(); }

private:
    int64_t duration_;
    std::deque<TimedValue> data_;

    // INVARIANT: data_ sorted in ascending order of timestamp
    // INVARIANT: data_.back().timestamp - data_.front().timestamp > duration_ once enough values have been added
};

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_FIXED_DURATION_BUFFER_H

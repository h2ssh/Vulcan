/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     statistics.h
 * \author   Collin Johnson
 *
 * Definition of functions to calculate simple statistics of a range of values:
 *
 *   - mean
 *   - variance
 */

#ifndef MATH_STATISTICS_H
#define MATH_STATISTICS_H

#include <algorithm>
#include <cassert>
#include <iterator>

namespace vulcan
{
namespace math
{

/**
 * median calculates the median of a range of values. The values must be comparable with operator<.
 *
 * Iterator must be a RandomAccessIterator.
 *
 * NOTE: The elements will be sorted such that the median value will be in the middle of the range: [begin,end)
 *
 * \param    begin           Starting element for the range
 * \param    end             One beyond the end for the range
 * \return   Median of the elements in the range [begin, end).
 */
template <class RandomAccessIterator>
typename std::iterator_traits<RandomAccessIterator>::value_type median(RandomAccessIterator begin,
                                                                       RandomAccessIterator end)
{
    assert(begin < end);
    auto medianIt = begin + (std::distance(begin, end) / 2);
    std::nth_element(begin, medianIt, end);
    return *medianIt;
}

/**
 * third_quartile calculates the third quartile of a range of values. The values must be comparable with operator<.
 *
 * Iterator must be a RandomAccessIterator.
 *
 * \param    begin           Starting element for the range
 * \param    end             One beyond the end for the range
 * \return   Third quartile of the elements in the range [begin, end).
 */
template <class RandomAccessIterator>
typename std::iterator_traits<RandomAccessIterator>::value_type third_quartile(RandomAccessIterator begin,
                                                                               RandomAccessIterator end)
{
    assert(begin < end);
    auto thirdIt = begin + (std::distance(begin, end) * 3 / 4);
    std::nth_element(begin, thirdIt, end);
    return *thirdIt;
}

/**
 * first_quartile calculates the first quartile of a range of values. The values must be comparable with operator<.
 *
 * Iterator must be a RandomAccessIterator.
 *
 * \param    begin           Starting element for the range
 * \param    end             One beyond the end for the range
 * \return   First quartile of the elements in the range [begin, end).
 */
template <class RandomAccessIterator>
typename std::iterator_traits<RandomAccessIterator>::value_type first_quartile(RandomAccessIterator begin,
                                                                               RandomAccessIterator end)
{
    assert(begin < end);
    auto firstIt = begin + (std::distance(begin, end) / 4);
    std::nth_element(begin, firstIt, end);
    return *firstIt;
}

/**
 * mean calculates the mean of a range of values. The iterator must point to a type that supports:
 *   - operator+(value)
 *   - operator/(int)
 *
 * The mean of the range [begin, end) is calculated.
 *
 * \param    begin       Start of the range for which to calculate the mean
 * \param    end         End of the range
 * \return   Mean of the range. If the range is empty, then the default value.
 */
template <class Iterator>
typename std::iterator_traits<Iterator>::value_type mean(Iterator begin, Iterator end)   //-> decltype(*begin)
{
    typename std::iterator_traits<Iterator>::value_type sum = 0;
    int n = 0;

    while (begin != end) {
        sum = sum + *begin;
        ++n;
        ++begin;
    }

    if (n == 0) {
        return sum;
    } else {
        return sum / n;
    }
}


/**
 * variance calculates the variance of a range of values. The iterator must point to a type that supports:
 *   - operator+(value)
 *   - operator-(value)
 *   - operator/(int)
 *
 * The variance of values of the range [begin, end) is calculated.
 *
 * \param    begin       Start of the range for which to calculate the mean
 * \param    end         End of the range
 * \return   Variance of the range. If the range is empty, then the default value.
 */
template <class Iterator>
typename std::iterator_traits<Iterator>::value_type variance(Iterator begin, Iterator end)   // -> decltype(*begin)
{
    auto avg = mean(begin, end);

    decltype(avg) sum = 0;
    decltype(avg) error = 0;

    int n = 0;

    while (begin != end) {
        sum = sum + (*begin - avg) * (*begin - avg);
        error = error + (*begin - avg);
        ++n;
        ++begin;
    }

    if (n < 2) {
        return sum;
    } else {
        return (sum - error * error / n) / (n - 1);
    }
}


/**
 * variance calculates the variance of a range of values. The iterator must point to a type that supports:
 *   - operator+(value)
 *   - operator-(value)
 *   - operator/(int)
 *
 * The variance of values of the range [begin, end) is calculated.
 *
 * \param    begin       Start of the range for which to calculate the mean
 * \param    end         End of the range
 * \return   Variance of the range. If the range is empty, then the default value.
 */
template <class Iterator, class T>
typename std::iterator_traits<Iterator>::value_type
  variance(Iterator begin, Iterator end, T mean)   // -> decltype(*begin)
{
    T sum = 0;
    T error = 0;

    int n = 0;

    while (begin != end) {
        sum = sum + (*begin - mean) * (*begin - mean);
        error = error + (*begin - mean);
        ++n;
        ++begin;
    }

    if (n < 2) {
        return sum;
    } else {
        return (sum - error * error / n) / (n - 1);
    }
}

}   // namespace math
}   // namespace vulcan

#endif   // MATH_STATISTICS_H

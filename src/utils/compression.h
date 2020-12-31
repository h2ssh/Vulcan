/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     compression.h
 * \author   Paul Foster and Collin Johnson
 *
 * Definition of a simple run-length encoding compression algorithm for a sequence of values.
 */

#ifndef UTILS_ENCODING_H
#define UTILS_ENCODING_H

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iterator>
#include <limits>
#include <type_traits>

namespace vulcan
{
namespace utils
{

/**
 * compress
 * invariants in outer function:
 * 1.out points at next slot to write to
 * 2.in points to the next literal to be added to the stream
 * 3.reads from current symbol shall not occur when in points to end (assert before current_symbol)
 * assert(current_point!=end); 4.reads frome current symbol are equivalent to memory access
 *
 * \pre  OutIter is an OutputIterator
 * \pre  InIter is a ForwardIterator
 * \pre  InIter::value_type == OutIter::value_type
 */
template <class InIter, class OutIter>
OutIter compress(InIter begin, InIter end, OutIter out)
{
    if (begin >= end) {
        return out;
    }

    const std::size_t kMaxCount = std::numeric_limits<typename std::iterator_traits<InIter>::value_type>::max();

    while (begin < end) {
        InIter chunkStartIt = begin;
        std::size_t count = 0;
        int isRepeat = (begin < std::prev(end)) && (*begin == *std::next(begin)) ? 1 : 0;

        // If the values were repeating, count the number of repeating values and store the repeated value in the
        // out buffer
        if (isRepeat) {
            auto repeatedValue = *begin;
            while ((count < kMaxCount) && (begin < end) && (*begin == repeatedValue)) {
                ++begin;
                ++count;
            }
        }
        // Count the number of non-repeating values starting from the current position and copy those values
        // into the out buffer
        else {
            while ((count < kMaxCount)
                   && (((begin < std::prev(end)) && (*begin != *std::next(begin))) || (begin == std::prev(end)))) {
                ++begin;
                ++count;
            }
        }

        // Store the header
        *out++ = isRepeat;
        *out++ = count;

        if (isRepeat) {
            *out++ = *chunkStartIt;
        } else {
            out = std::copy(chunkStartIt, begin, out);
        }
    }

    return out;
}


/**
 * decompress
 *
 * \pre  OutIter is an OutputIterator
 * \pre  InIter is an InputIterator
 */
template <class InIter, class OutIter>
OutIter decompress(InIter begin, InIter end, OutIter out)
{
    while (begin < end) {
        auto isRepeat = *begin++;
        auto count = *begin++;

        assert((isRepeat == 0) || (isRepeat == 1));

        if (isRepeat) {
            out = std::fill_n(out, count, *begin);
            begin++;
        } else {
            out = std::copy_n(begin, count, out);
            std::advance(begin, count);
        }
    }

    return out;
}

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_ENCODING_H

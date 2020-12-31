/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     algorithm_ext.h
 * \author   Collin Johnson
 *
 * Definition of some common idioms for dealing with STL containers:
 *
 *   - erase_remove/erase_remove_if : call .erase on the container with the result of std::remove/remove_if
 *   - contains/contains_if : check if the value is inside the provided container using std::find
 */

#ifndef UTILS_ALGORITHM_EXT_H
#define UTILS_ALGORITHM_EXT_H

#include <algorithm>
#include <type_traits>

namespace vulcan
{
namespace utils
{

/**
 * erase_remove uses the erase-remove idiom to efficiently erase elements from a container. The provided container
 * must have MoveAssignable iterators. Associative containers don't meet this criteria.
 *
 * \param    c           Container to remove value from
 * \param    value       Value to be removed
 */
template <class Container, class T>
int erase_remove(Container& c, T&& value)
{
    static_assert(std::is_move_assignable<typename Container::iterator>::value,
                  "erase_remove must have MoveAssignable iterators\n");
    auto endIt = std::remove(c.begin(), c.end(), value);
    int numErased = std::distance(endIt, c.end());
    c.erase(endIt, c.end());
    return numErased;
}


/**
 * erase_remove_if uses the erase-remove idiom to efficiently erase elements from a container. The provided container
 * must have MoveAssignable iterators. Associative containers don't meet this criteria.
 *
 * \param    c           Container to remove value from
 * \param    unary       Unary operation to use for deciding if the value should be removed
 */
template <class Container, class UnaryOp>
int erase_remove_if(Container& c, UnaryOp&& op)
{
    static_assert(std::is_move_assignable<typename Container::iterator>::value,
                  "erase_remove_if must have MoveAssignable iterators\n");
    auto endIt = std::remove_if(c.begin(), c.end(), op);
    int numErased = std::distance(endIt, c.end());
    c.erase(endIt, c.end());
    return numErased;
}


/**
 * erase_unique uses the erase-remove idiom to efficiently erase adjacent elements that are the same from a container.
 * The provided container must have MoveAssignable iterators. Associative containers don't meet this criteria.
 *
 * \param    c           Container to remove non-unique values from
 * \return   Number of elements erased.
 */
template <class Container>
int erase_unique(Container& c)
{
    static_assert(std::is_move_assignable<typename Container::iterator>::value,
                  "erase_remove must have MoveAssignable iterators\n");
    auto endIt = std::unique(c.begin(), c.end());
    int numErased = std::distance(endIt, c.end());
    c.erase(endIt, c.end());
    return numErased;
}


/**
 * contains uses std::find to determine if a container contains the provided value. The equivalent operation is:
 *
 *   std::find(container.begin(), container.end(), value) != container.end().
 *
 * \param    c           Container to search
 * \param    value       Value to search for
 * \return   True if the value is contained in the provided container.
 */
template <class Container, class T>
bool contains(const Container& c, T&& value)
{
    return std::find(c.begin(), c.end(), value) != c.end();
}


/**
 * contains_if uses std::find_if to determine if a container contains the provided value. The equivalent operation is:
 *
 *   std::find_if(container.begin(), container.end(), op) != container.end().
 *
 * \param    c           Container to search
 * \param    op          Unary operation to use for deciding if the value is in the container
 * \return   True if the unary condition is satisfied by a value in the container.
 */
template <class Container, class UnaryOp>
bool contains_if(const Container& c, UnaryOp&& op)
{
    return std::find_if(c.begin(), c.end(), op) != c.end();
}


/**
 * contains_any checks to see if any of the values in c are contained within the container values. The operation is
 * equivalent to seeing if c contains any value in values.
 *
 * \param    c           Container to search
 * \param    values      Values to search for
 * \return   True if the value is contained in the provided container.
 */
template <class Container, class Values>
bool contains_any(const Container& c, Values&& value)
{
    for (auto& v : value) {
        // As sson as a value is found, then finished
        if (contains(c, v)) {
            return true;
        }
    }

    return false;
}

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_ALGORITHM_EXT_H

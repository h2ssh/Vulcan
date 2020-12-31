/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     enum_operators.h
 * \author   Collin Johnson
 *
 * Definition of templates for implementing operator overloads for enum classes.
 * These templates allow for doing bit operations on the enum class values, performing
 * the necessary casting internally.
 *
 *   - enum_bitwise_and
 *   - enum_bitwise_or
 */

#ifndef UTILS_ENUM_OPERATORS_H
#define UTILS_ENUM_OPERATORS_H

#include <type_traits>

namespace vulcan
{
namespace utils
{

/**
 * enum_bitwise_or
 */
template <class T>
inline constexpr T enum_bitwise_or(T lhs, T rhs)
{
    using Size = typename std::underlying_type<T>::type;
    return static_cast<T>(static_cast<Size>(lhs) | static_cast<Size>(rhs));
}

/**
 * enum_bitwise_and
 */
template <class T>
inline constexpr T enum_bitwise_and(T lhs, T rhs)
{
    using Size = typename std::underlying_type<T>::type;
    return static_cast<T>(static_cast<Size>(lhs) & static_cast<Size>(rhs));
}

/**
 * enum_to_bool
 */
template <class T>
inline constexpr bool enum_to_bool(T e)
{
    using Size = typename std::underlying_type<T>::type;
    return static_cast<Size>(e) == 0 ? false : true;
}

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_ENUM_OPERATORS_H

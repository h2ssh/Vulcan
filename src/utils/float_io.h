/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     float_io.h
* \author   Collin Johnson
*
* Helper functions for dealing with saving/loading floating point values to/from a stream.
*/

#ifndef UTILS_FLOAT_IO_H
#define UTILS_FLOAT_IO_H

#include <iomanip>
#include <istream>
#include <limits>
#include <ostream>
#include <string>
#include <type_traits>
#include <cstdlib>

namespace vulcan
{
namespace utils
{

/**
* save_floating_point saves a floating point number to the provided stream. A space is added after the number.
*/
template <typename T>
void save_floating_point(std::ostream& out, T val)
{
    static_assert(std::is_floating_point<T>::value, "save_floating_point type must be floating point");
    out << std::setprecision(std::numeric_limits<double>::max_digits10) << val << ' ';
}

/**
* load_floating_point loads a floating point number from a stream.
*/
inline double load_floating_point(std::istream& in)
{
    // The best way to read a floating point is to read the full string and then use strtod because it correctly handles
    // NaN or INF values written in the text.
    std::string str;
    in >> str;
    return std::strtod(str.c_str(), nullptr);
}

}
}

#endif // UTILS_FLOAT_IO_H

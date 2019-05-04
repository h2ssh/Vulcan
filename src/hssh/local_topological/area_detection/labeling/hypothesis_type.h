/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_type.h
* \author   Collin Johnson
*
* Definition of HypothesisType enum class.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_TYPE_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_TYPE_H

#include <utils/enum_operators.h>
#include <iosfwd>

namespace vulcan
{
namespace hssh
{

/**
* HypothesisType defines the possible classifications for a hypothesis.
*/
enum class HypothesisType : unsigned char
{
    kNone         = 0x00,
    kDecision     = 0x02,
    kDest         = 0x04,
    kPlace        = kDest | kDecision,
    kPathEndpoint = 0x10,
    kPathDest     = 0x20,
    kPath         = kPathEndpoint | kPathDest,
    kArea         = kPath | kDecision | kDest
};

// Overloads for easier processing of HypothesisType
inline constexpr HypothesisType operator&(HypothesisType lhs, HypothesisType rhs)
{
    return utils::enum_bitwise_and(lhs, rhs);
}

inline constexpr HypothesisType operator|(HypothesisType lhs, HypothesisType rhs)
{
    return utils::enum_bitwise_or(lhs, rhs);
}

inline bool is_hypothesis_type(HypothesisType value, HypothesisType type)
{
    return utils::enum_to_bool(value & type);
}

std::ostream& operator<<(std::ostream& out, HypothesisType type);
std::istream& operator>>(std::istream& in, HypothesisType& type);

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_TYPE_H


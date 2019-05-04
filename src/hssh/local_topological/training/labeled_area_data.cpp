/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     labeled_area_data.cpp
* \author   Collin Johnson
* 
* Definition of LabeledAreaData.
*/

#include <hssh/local_topological/training/labeled_area_data.h>
#include <istream>
#include <map>
#include <ostream>
#include <cassert>

namespace vulcan
{
namespace hssh
{

//////////////////// Operators ////////////////////////

/*
* The format for LabeledFeatures is:
* 
*   type features (using HypothesisFeatures format)
*/
std::ostream& operator<<(std::ostream& out, const LabeledFeatures& example)
{
    out << static_cast<int>(example.type) << ' ' << example.features;
    return out;
}


std::istream& operator>>(std::istream& in, LabeledFeatures& example)
{
    int type;
    in >> type >> example.features;

    example.type = static_cast<HypothesisType>(type);
    return in;
}

bool operator==(const LabeledFeatures& lhs, const LabeledFeatures& rhs)
{
    return (lhs.type == rhs.type) && (lhs.features == rhs.features);
}


bool operator!=(const LabeledFeatures& lhs, const LabeledFeatures& rhs)
{
    return !(lhs == rhs);
}

} // namespace hssh
} // namespace vulcan

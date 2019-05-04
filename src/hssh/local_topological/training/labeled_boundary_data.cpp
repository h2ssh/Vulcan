/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     labeled_boundary_data.cpp
* \author   Collin Johnson
*
* Definition of required operators for LabeledBoundaryFeatures.
*/

#include <hssh/local_topological/training/labeled_boundary_data.h>

namespace vulcan
{
namespace hssh
{

bool operator==(const LabeledBoundaryFeatures& lhs, const LabeledBoundaryFeatures& rhs)
{
    return (lhs.types[0] == rhs.types[0]) && (lhs.types[1] == rhs.types[1]) && (lhs.isOn == rhs.isOn);
}


bool operator!=(const LabeledBoundaryFeatures& lhs, const LabeledBoundaryFeatures& rhs)
{
    return !(lhs == rhs);
}


std::ostream& operator<<(std::ostream& out, const LabeledBoundaryFeatures& example)
{
    out << example.types[0] << ' ' << example.types[1] << ' ' << example.isOn;
    return out;
}


std::istream& operator>>(std::istream& in, LabeledBoundaryFeatures& example)
{
    in >> example.types[0] >> example.types[1] >> example.isOn;
    return in;
}

} // namespace hssh
} // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     labeled_gateway_data.cpp
 * \author   Collin Johnson
 *
 * Definition of LabeledGatewayFeatures.
 */

#include "hssh/local_topological/training/labeled_gateway_data.h"

namespace vulcan
{
namespace hssh
{

bool operator==(const LabeledGatewayFeatures& lhs, const LabeledGatewayFeatures& rhs)
{
    return (lhs.isGateway == rhs.isGateway) && (lhs.features == rhs.features);
}


bool operator!=(const LabeledGatewayFeatures& lhs, const LabeledGatewayFeatures& rhs)
{
    return !(lhs == rhs);
}

/*
 * The format for LabeledGatewayFeatures is:
 *
 *   is_gateway cell features
 */
std::ostream& operator<<(std::ostream& out, const LabeledGatewayFeatures& example)
{
    out << example.isGateway << ' ' << example.cell.x << ' ' << example.cell.y << ' ' << example.features;
    return out;
}


std::istream& operator>>(std::istream& in, LabeledGatewayFeatures& example)
{
    in >> example.isGateway >> example.cell.x >> example.cell.y >> example.features;
    return in;
}

}   // namespace hssh
}   // namespace vulcan

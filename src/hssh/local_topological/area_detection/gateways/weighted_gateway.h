/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     weighted_gateway.h
* \author   Collin Johnson
* 
* Declaration of WeightedGateway and associated operators.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_WEIGHTED_GATEWAY_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_WEIGHTED_GATEWAY_H

#include <hssh/local_topological/gateway.h>

namespace vulcan
{
namespace hssh 
{

/**
* WeightedGateway is a gateway with an associated weight and flag indicating if it was part of a
* transition in the environment.
*/
struct WeightedGateway
{
    Gateway gateway;
    double weight;
    bool isTransition;
};

// Comparison operators compare the weight
inline bool operator<(const WeightedGateway& lhs, const WeightedGateway& rhs)
{
    return lhs.weight < rhs.weight;
}

inline bool operator>(const WeightedGateway& lhs, const WeightedGateway& rhs)
{
    return lhs.weight > rhs.weight;
}

// Equality operators check if the underlying gateways are the same
inline bool operator==(const WeightedGateway& lhs, const WeightedGateway& rhs)
{
    return lhs.gateway == rhs.gateway;
}

inline bool operator!=(const WeightedGateway& lhs, const WeightedGateway& rhs)
{
    return !(lhs == rhs);
}

// Also allow equality comparison against a raw gateway
inline bool operator==(const Gateway& lhs, const WeightedGateway& rhs)
{
    return lhs == rhs.gateway;
}

inline bool operator==(const WeightedGateway& lhs, const Gateway& rhs)
{
    return lhs.gateway == rhs;
}

} // namespace vulcan
}

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_WEIGHTED_GATEWAY_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lazy_generator.cpp
* \author   Collin Johnson
* 
* Definition of LazyGenerator.
*/

#include "hssh/global_topological/mapping/lazy_generator.h"
#include "utils/stub.h"

namespace vulcan
{
namespace hssh 
{

double LazyGenerator::computeChildLogLikelihood(const TopologicalState& parent,
                                                const TopologicalState& child, 
                                                const TopologicalVisit& visit)
{
    PRINT_PRETTY_STUB();
    return 0.0;
}
    
} // namespace hssh
} // namespace vulcan

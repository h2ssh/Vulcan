/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     director.cpp
* \author   Collin Johnson
*
* Definition of TopoMapDirector.
*/

#include "simulator/topo/director.h"
#include "simulator/topo/params.h"

namespace vulcan
{
namespace simulator
{

TopoMapDirector::TopoMapDirector(const topo_map_simulator_params_t& params)
                            : simulator(params.simParams)
{
}


void TopoMapDirector::handleData(const hssh::TopologicalMap& map, const std::string& channel)
{

}


void TopoMapDirector::handleData(const hssh::GlobalLocation& state, const std::string& channel)
{

}


void TopoMapDirector::handleData(const planner::GoalRoute& plan, const std::string& channel)
{

}


void TopoMapDirector::handleData(const planner::DecisionTargetSequence& sequence, const std::string& channel)
{

}


void TopoMapDirector::waitForData(void)
{

}


void TopoMapDirector::processAvailableData(void)
{

}


void TopoMapDirector::transmitCalculatedOutput(void)
{

}

} // namespace simulator
} // namespace vulcan

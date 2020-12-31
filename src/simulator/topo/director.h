/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     director.h
* \author   Collin Johnson
*
* Declaration of TopoMapDirector.
*/

#ifndef SIMULATOR_TOPO_DIRECTOR_H
#define SIMULATOR_TOPO_DIRECTOR_H

#include "system/director.h"
#include "simulator/topo/consumers.h"
#include "utils/mutex.h"
#include "utils/condition_variable.h"
#include "simulator/topo/topological_map_simulator.h"

namespace vulcan
{
namespace simulator
{

struct topo_map_simulator_params_t;

/**
* TopoMapDirector organizes processing of data for the topo_map_simulator. Incoming data is passed to
* the TopologicalMapSimulator and the output of the simulator is sent of to the subscribed output consumers.
*/
class TopoMapDirector : public system::Director
                      , public TopoMapInputConsumer
{
public:

    /**
    * Constructor for TopoMapDirector.
    *
    * \param    params          Parameters for the module
    */
    TopoMapDirector(const topo_map_simulator_params_t& params);

    // TopoMapInputConsumer interface
    void handleData(const hssh::TopologicalMap&             map,      const std::string& channel);
    void handleData(const hssh::GlobalLocation&          state,    const std::string& channel);
    void handleData(const planner::GoalRoute&         plan,     const std::string& channel);
    void handleData(const planner::DecisionTargetSequence& sequence, const std::string& channel);

private:

    // system::Director interface
    void waitForData(void);
    void processAvailableData(void);
    void transmitCalculatedOutput(void);

    TopologicalMapSimulator simulator;

    utils::Mutex             dataLock;
    utils::ConditionVariable dataTrigger;
};

}
}

#endif // SIMULATOR_TOPO_DIRECTOR_H

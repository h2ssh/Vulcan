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
* Declaration of GlobalTopoDirector.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_DIRECTOR_H
#define HSSH_GLOBAL_TOPOLOGICAL_DIRECTOR_H

#include "system/director.h"
#include "hssh/global_topological/utils/global_topo_data_queue.h"
#include "hssh/global_topological/params.h"
#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh
{

class TopologicalSLAM;

/**
* GlobalTopoDirector is responsible for orchestrating the flow of computation for
* the global_topo module. Data arrives, computation occurs, data is sent to output consumers.
*/
class GlobalTopoDirector : public system::Director
{
public:

    /**
    * Constructor for GlobalTopoDirector.
    */
    GlobalTopoDirector(std::unique_ptr<TopologicalSLAM> slam,
                       const utils::CommandLine& commandLine,
                       const utils::ConfigFile& config);

    /**
    * Destructor for GlobalTopoDirector.
    */
    virtual ~GlobalTopoDirector(void);

    // system::Director interface
    void subscribeToData(system::ModuleCommunicator& communicator) override;
    system::TriggerStatus waitForTrigger(void) override;
    system::UpdateStatus runUpdate(system::ModuleCommunicator& communicator) override;
    void shutdown(system::ModuleCommunicator& communicator) override;

    // Data handlers
    void handleData(const LocalAreaEventVec& events, const std::string& channel);
    void handleData(const GlobalTopoCommandPtr& command, const std::string& channel);
    void handleData(const LocalPose& pose, const std::string& channel);
    void handleData(const LocalTopoMap& localMap, const std::string& channel);

private:

    global_topo_params_t params_;
    std::unique_ptr<TopologicalSLAM> slam_;
//     MetricMapCache placeManager;
    GlobalTopoDataQueue queue_;
    bool haveDataToTransmit_ = false;

    void processCommand(const GlobalTopoCommand& command);
    void processEvent(const LocalAreaEvent& event, const LocalTopoMap& localMap);
    void transmitCalculatedOutput(system::ModuleCommunicator& communicator);
};

}
}

#endif // HSSH_GLOBAL_TOPOLOGICAL_DIRECTOR_H

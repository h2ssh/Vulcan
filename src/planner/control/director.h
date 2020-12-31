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
* Declaration of ControlPlannerDirector.
*/

#ifndef PLANNER_CONTROL_DIRECTOR_H
#define PLANNER_CONTROL_DIRECTOR_H

#include "planner/control/command.h"
#include "planner/control/state.h"
#include "system/director.h"
#include "utils/condition_variable.h"
#include "utils/locked_double_buffer.h"
#include "lcmtypes/commands/direct_control_command.h"

namespace vulcan
{
namespace planner
{
    
class ControlPlanner;

/**
* ControlPlannerDirector
*/
class ControlPlannerDirector : public system::Director
{
public:
    
    /**
    * Constructor for ControlPlannerDirector.
    */
    ControlPlannerDirector(std::unique_ptr<ControlPlanner> planner);
    
    /**
    * Destructor for ControlPlannerDirector.
    */
    virtual ~ControlPlannerDirector(void);
    
    // Data handlers
    void handleData(const ControlCommand& command, const std::string& channel);
    void handleData(const hssh::LocalPose& pose, const std::string& channel);
    void handleData(const hssh::LocalPerceptualMap& lpm, const std::string& channel);
    void handleData(const mpepc::metric_planner_status_message_t& status, const std::string& channel);
    void handleData(const vulcan_lcm::direct_control_command& command, const std::string& channel);
    
    // system::Director interface
    void subscribeToData(system::ModuleCommunicator& communicator) override;
    void shutdown(system::ModuleCommunicator& communicator) override;
    system::TriggerStatus waitForTrigger(void) override;
    system::UpdateStatus runUpdate(system::ModuleCommunicator& communicator) override;

private:
    
    template <class T>
    using Buffer = utils::LockedDoubleBuffer<T>;
    
    Buffer<ControlCommand> command_;
    Buffer<hssh::LocalPose> pose_;
    Buffer<hssh::LocalPerceptualMap> map_;
    Buffer<mpepc::metric_planner_status_message_t> mpepcStatus_;
    
    bool haveLPM_;                  ///< True if an LPM has ever been received
    bool executingCommand_;         ///< True if a command is currently executing
    
    std::unique_ptr<ControlPlanner> planner_;
    
    utils::ConditionVariable dataTrigger_;

    ControlState loadState(void);
    void issueNewCommandIfNeeded(const ControlState& state);
    ControlTaskResult executeTask(const ControlState& state);
};

}
}

#endif // PLANNER_CONTROL_DIRECTOR_H

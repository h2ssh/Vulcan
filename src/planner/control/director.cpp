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
* Definition of ControlPlannerDirector.
*/

#include "planner/control/director.h"
#include "planner/control/planner.h"
#include "planner/control/tasks/absolute_motion.h"
#include "mpepc/metric_planner/task/task.h"
#include "system/module_communicator.h"

namespace vulcan
{
namespace planner
{
    
ControlPlannerDirector::ControlPlannerDirector(std::unique_ptr<ControlPlanner> planner)
: haveLPM_(false)
, executingCommand_(false)
, planner_(std::move(planner))
{
    
}
    

ControlPlannerDirector::~ControlPlannerDirector(void)
{
    // For std::unique_ptr
}
    
    
void ControlPlannerDirector::handleData(const ControlCommand& command, const std::string& channel)
{
    command_ = command;
    
    dataTrigger_.setPredicate(true);
    dataTrigger_.broadcast();
}
    
    
void ControlPlannerDirector::handleData(const hssh::LocalPose& pose, const std::string& channel)
{
    pose_ = pose;
    
    dataTrigger_.setPredicate(true);
    dataTrigger_.broadcast();
}


void ControlPlannerDirector::handleData(const hssh::LocalPerceptualMap& lpm, const std::string& channel)
{
    map_ = lpm;
    
    dataTrigger_.setPredicate(true);
    dataTrigger_.broadcast();
}


void ControlPlannerDirector::handleData(const mpepc::metric_planner_status_message_t& status, 
                                        const std::string& channel)
{
    mpepcStatus_ = status;
}


void ControlPlannerDirector::handleData(const vulcan_lcm::direct_control_command& command, const std::string& channel)
{
    std::cout << "INFO: ControlPlanner: Received direct_control_command on channel " << channel << ". Issuing as an "
        << "AbsoluteMotionTask.\n";

    // Convert the control command into a generic ControlCommand.
    AbsoluteMotion motion;
    switch(command.command)
    {
    case vulcan_lcm::direct_control_command::STOP:
        motion = AbsoluteMotion::stop;
        break;
        
    case vulcan_lcm::direct_control_command::GO_STRAIGHT:
        motion = AbsoluteMotion::go_straight;
        break;
        
    case vulcan_lcm::direct_control_command::TURN_LEFT:
        motion = AbsoluteMotion::turn_left;
        break;
        
    case vulcan_lcm::direct_control_command::TURN_RIGHT:
        motion = AbsoluteMotion::turn_right;
        break;
        
    default:
        std::cerr << "ERROR: ControlPlannerDirector: Failed to convert direct_control_command to ControlCommand."
            << " Unknown command:" << command.command << " Issuing stop command.\n";
        motion = AbsoluteMotion::stop;
        break;
    }
    
    handleData(ControlCommand(command.timestamp, 
                              command.source, 
                              std::make_shared<AbsoluteMotionTask>(motion, command.speed)),
               channel);
}


void ControlPlannerDirector::subscribeToData(system::ModuleCommunicator& communicator)
{
    communicator.subscribeTo<hssh::LocalPose>(this);
    communicator.subscribeTo<hssh::LocalPerceptualMap>(this);
    communicator.subscribeTo<mpepc::metric_planner_status_message_t>(this);
    communicator.subscribeTo<vulcan_lcm::direct_control_command>(this);
}


void ControlPlannerDirector::shutdown(system::ModuleCommunicator& communicator)
{
    // Nothing to do when shutting down
}


system::TriggerStatus ControlPlannerDirector::waitForTrigger(void)
{
    /*
    * The conditions for running an update are:
    * 
    *   - An LPM has been received at least once
    *   - A command is currently running or a new one has arrived
    *   - A new pose has arrived
    */
    
    dataTrigger_.timedWait(100);
    dataTrigger_.setPredicate(false);

    if(!haveLPM_ && map_.hasData())
    {
        haveLPM_ = true;
    }

    // If all conditions are met, then perform an update. Otherwise, keep waiting.
    if(pose_.hasData() && haveLPM_ && (executingCommand_ || command_.hasData()))
    {
        return system::TriggerStatus::ready;
    }

    return system::TriggerStatus::not_ready;
}


system::UpdateStatus ControlPlannerDirector::runUpdate(system::ModuleCommunicator& communicator)
{
    auto state = loadState();
    issueNewCommandIfNeeded(state);
    
    auto result = executeTask(state);
    communicator.sendMessage(result.status);
    
    if(result.task)
    {
        communicator.sendMessage(result.task);
    }

    // control_planner always runs
    return system::UpdateStatus::running;
}


ControlState ControlPlannerDirector::loadState(void)
{
    if(map_.hasData())
    {
        map_.swapBuffers();
    }
    
    if(pose_.hasData())
    {
        pose_.swapBuffers();
    }
    
    ControlState state;
    state.map = &(map_.read());
    state.pose = pose_;
    
    if(mpepcStatus_.hasData())
    {
        mpepcStatus_.swapBuffers();
        state.metricPlannerStatus = mpepcStatus_;
    }
    
    return state;
}


void ControlPlannerDirector::issueNewCommandIfNeeded(const ControlState& state)
{
    if(command_.hasData())
    {
        command_.swapBuffers();
        planner_->assignTask(command_, state);
        executingCommand_ = true;
    }
}


ControlTaskResult ControlPlannerDirector::executeTask(const ControlState& state)
{
    // Run the current planner task
    auto result = planner_->executeTask(state);
    
    // If no tasks remain, turn off module updates until a new command arrives
    if(!planner_->haveTask())
    {
        executingCommand_ = false;
    }
    
    return result;
}

} // namespace planner
} // namespace vulcan

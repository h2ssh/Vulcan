/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     logical_interface_frame.h
* \author   Collin Johnson
*
* Declaration of LogicalInterfaceFrame.
*/

#ifndef UI_LOGICAL_LOGICAL_INTERFACE_FRAME_H
#define UI_LOGICAL_LOGICAL_INTERFACE_FRAME_H

#include <ui/common/ui_main_frame.h>
#include <ui/logical/logical_interface.h>
#include <ui/logical/logical_interface_experiment.h>
#include <hssh/global_topological/global_location.h>
#include <memory>

namespace vulcan
{
namespace planner
{
    class DecisionTarget;
    class GoalTarget;
}

namespace ui
{

class LogicalInterfaceFrame : public LogicalFrame
{
public:

    /**
    * Constructor LogicalInterfaceFrame.
    *
    * \param    params              Parameters for the logical interface UI
    * \param    evaluationFile      Filename of the evaluation file
    */
    LogicalInterfaceFrame(const logical_interface_params_t& params);

    virtual ~LogicalInterfaceFrame(void);

    // UIMainFrame interface
    virtual void connectConsumersToDataDistributor(system::ModuleCommunicator& producer);
    virtual void connectProducersToOutputConsumer(system::ModuleCommunicator* consumer);

    // LocalTopologyDataConsumer interface
//     virtual void handleData(const hssh::local_topology_place_event_t& event);

    virtual void handleData(const hssh::TopologicalMap& map, const std::string& channel);

private:

    enum interface_state_t
    {
        INITIALIZING_EXPERIMENT,
        ASSIGNING_TASK,
        SELECTING_TASK,
        EXECUTING_TASK,
        COMPLETED_TASK,
        COMPLETED_EXPERIMENT
    };

    void runInterfaceStateMachine(void);
    void initializeExperiment(void);
    void assignTask          (void);
    void selectTask          (void);
    void executeTask         (void);
    void completeTask        (void);
    void completeExperiment  (void);

    void sendSetMapMessage      (const std::string& filename);
    void sendRelativePlaceTarget(std::shared_ptr<planner::DecisionTarget> target);
    void sendGlobalPlaceTarget  (std::shared_ptr<planner::GoalTarget>     target);

    void decisionButtonPressed(wxCommandEvent& event);
    void goalButtonPressed    (wxCommandEvent& event);

    // redrawing event handlers
    void paint(wxPaintEvent& event);
    void timerFired(wxTimerEvent& event);

    LogicalInterfaceExperiment experiment;
    logical_experiment_task_t  currentTask;
    hssh::GlobalLocation    currentLocation;

    int sequenceId;

    interface_state_t state;

    system::ModuleCommunicator* consumer;

    DECLARE_EVENT_TABLE()
};

}
}

#endif // UI_LOGICAL_LOGICAL_INTERFACE_FRAME_H

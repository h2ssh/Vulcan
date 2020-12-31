/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     logical_interface_frame.cpp
* \author   Collin Johnson
*
* Definition of LogicalInterfaceFrame.
*/

#include "ui/logical/logical_interface_frame.h"
#include "ui/logical/logical_interface_dialogs.h"
#include "hssh/global_topological/messages.h"
#include "system/module_communicator.h"

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(LogicalInterfaceFrame, wxFrame)
    EVT_BUTTON(ID_DECISION_BUTTON, LogicalInterfaceFrame::decisionButtonPressed)
    EVT_BUTTON(ID_GOAL_BUTTON,     LogicalInterfaceFrame::goalButtonPressed)
    EVT_PAINT(LogicalInterfaceFrame::paint)
//     EVT_TIMER(TIMER_ID, LogicalInterfaceFrame::timerFired)
END_EVENT_TABLE()


LogicalInterfaceFrame::LogicalInterfaceFrame(const logical_interface_params_t& params, const std::string& evaluationFile)
    : LogicalFrame(0)
    , experiment(params.experimentParams, evaluationFile)
    , sequenceId(0)
    , consumer(0)
{
//     plannerWidget->setRenderContext(glContext);
//     plannerWidget->setPlaceManager(&placeManager);

    initialize(nullptr, 15, plannerWidget);
}


LogicalInterfaceFrame::~LogicalInterfaceFrame(void)
{
    // Nothing to do for now
}


void LogicalInterfaceFrame::connectConsumersToDataDistributor(system::ModuleCommunicator& producer)
{
    
}


void LogicalInterfaceFrame::connectProducersToOutputConsumer(system::ModuleCommunicator* consumer)
{
    // The LogicalInterfaceFrame handles all the communication, so just save the consumer instance here
    this->consumer = consumer;
}


void LogicalInterfaceFrame::handleData(const hssh::TopologicalMap& map, const std::string& channel)
{
    currentLocation = map.getGlobalLocation();

    Refresh();
}


void LogicalInterfaceFrame::runInterfaceStateMachine(void)
{
    interface_state_t currentState = state;

    do
    {
        currentState = state;

        switch(state)
        {
        case INITIALIZING_EXPERIMENT:
            initializeExperiment();
            break;

        case ASSIGNING_TASK:
            assignTask();
            break;

        case SELECTING_TASK:
            selectTask();
            break;

        case EXECUTING_TASK:
            executeTask();
            break;

        case COMPLETED_TASK:
            completeTask();
            break;

        case COMPLETED_EXPERIMENT:
            completeExperiment();
            break;

        default:
            std::cerr<<"ERROR:LogicalInterfaceFrame: Invalid state machine state. WTF, yo?\n";
        }
    } while(currentState != state);
}


void LogicalInterfaceFrame::initializeExperiment(void)
{
    decisionButton->Disable();
    goalButton->Disable();

    sendSetMapMessage(experiment.getExperimentMapFilename());

    state = ASSIGNING_TASK;
}


void LogicalInterfaceFrame::assignTask(void)
{
    if(experiment.hasNextTask())
    {
        currentTask = experiment.nextTask();

        LogicalTaskDialog dialog(this, currentTask.description, currentTask.level);

        dialog.ShowModal();

        state = SELECTING_TASK;
    }
    else
    {
        state = COMPLETED_EXPERIMENT;
    }
}


void LogicalInterfaceFrame::selectTask(void)
{
    switch(currentTask.level)
    {
    case LOGICAL_ANY:
        goalButton->Enable();
        decisionButton->Enable();
        break;

    case LOGICAL_DECISION:
        goalButton->Disable();
        decisionButton->Enable();
        break;

    case LOGICAL_GOAL:
        goalButton->Enable();
        decisionButton->Disable();
        break;

    default:
        std::cerr<<"ERROR:LogicalInterfaceFrame: Invalid logical level detected for current task:"<<currentTask.level<<" Skipping task and trying the next one.\n";
        state = ASSIGNING_TASK;
    }

    experiment.startedTask(currentTask);
}


void LogicalInterfaceFrame::executeTask(void)
{
    if(experiment.isTaskComplete(currentLocation))
    {
        state = COMPLETED_TASK;

        experiment.finishedTask(currentTask);
    }
}


void LogicalInterfaceFrame::completeTask(void)
{
    if(experiment.hasNextTask())
    {
        LogicalTaskCompleteDialog dialog(this);

        dialog.ShowModal();

        state = ASSIGNING_TASK;
    }
    else
    {
        state = COMPLETED_EXPERIMENT;
    }
}


void LogicalInterfaceFrame::completeExperiment(void)
{
    LogicalExperimentCompleteDialog dialog(this);

    dialog.ShowModal();
}


void LogicalInterfaceFrame::sendSetMapMessage(const std::string& filename)
{
    hssh::global_topo_message_t message;
    message.type                = hssh::CORRECT_GLOBAL_MAP;
    message.correctMap.source   = hssh::SOURCE_FILE;
    message.correctMap.filename = filename;

}


void LogicalInterfaceFrame::sendRelativePlaceTarget(std::shared_ptr<planner::DecisionTarget> target)
{
    std::vector<std::shared_ptr<planner::DecisionTarget>> targets;

    targets.push_back(target);
    targets.push_back(std::shared_ptr<planner::DecisionTarget>(new planner::LocalPathTarget(planner::LOCAL_TOPO_PATH_END)));

    
}


void LogicalInterfaceFrame::sendGlobalPlaceTarget(std::shared_ptr<planner::GoalTarget> target)
{
    
}


void LogicalInterfaceFrame::decisionButtonPressed(wxCommandEvent& event)
{
    LogicalDecisionDialog dialog(this);

    experiment.startedSelection(currentTask);

    if(dialog.ShowModal() == wxID_OK)
    {
        experiment.finishedSelection(currentTask);

        sendRelativePlaceTarget(dialog.getDecisionTarget());

        Refresh();
    }
}


void LogicalInterfaceFrame::goalButtonPressed(wxCommandEvent& event)
{
    LogicalGoalDialog dialog(this, experiment.getPlaceDescriptions(), experiment.getExperimentMap());

    experiment.startedSelection(currentTask);

    if(dialog.ShowModal() == wxID_OK)
    {
        experiment.finishedSelection(currentTask);

        sendGlobalPlaceTarget(dialog.getGoalTarget());

        Refresh();
    }
}


void LogicalInterfaceFrame::paint(wxPaintEvent& event)
{
    wxPaintDC dc(this);

    runInterfaceStateMachine();
}


void LogicalInterfaceFrame::timerFired(wxTimerEvent& event)
{
    Refresh();
}

} // namespace ui
} // namespace vulcan

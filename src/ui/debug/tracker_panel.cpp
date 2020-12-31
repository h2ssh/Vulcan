/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
[* \file     tracker_panel.cpp
* \author   Collin Johnson
*
* Definition of TrackerPanel.
*/

#include "ui/debug/tracker_panel.h"
#include "ui/debug/debug_ui.h"
#include "ui/debug/tracker_display_widget.h"
#include "system/module_communicator.h"
#include <cassert>

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(TrackerPanel, wxEvtHandler)
    EVT_CHECKBOX(ID_TRACKER_FOLLOW_ROBOT_BOX, TrackerPanel::followRobotChecked)
    EVT_CHECKBOX(ID_SHOW_LASER_OBJECTS_BOX, TrackerPanel::showLaserObjectsChecked)
    EVT_CHECKBOX(ID_SHOW_LASER_OBJECT_POINTS_BOX, TrackerPanel::showLaserPointsChecked)
    EVT_CHECKBOX(ID_SHOW_LASER_UNCERTAINTY_BOX, TrackerPanel::showLaserUncertaintyChecked)
    EVT_RADIOBOX(ID_LASER_OBJ_BOUNDARY_RADIO, TrackerPanel::boundaryToShowChanged)
    EVT_CHECKBOX(ID_SHOW_TRACKED_OBJECTS_BOX, TrackerPanel::showTrackedObjectsChecked)
    EVT_CHECKBOX(ID_SHOW_TRACKED_ACCELERATION_BOX, TrackerPanel::showAccelerationChecked)
    EVT_RADIOBOX(ID_RIGID_OBJECT_STATE_RADIO, TrackerPanel::rigidObjectStateToShowChanged)
    EVT_RADIOBOX(ID_TRACKING_UNCERTAINTY_RADIO, TrackerPanel::trackingUncertaintyToShowChanged)
    EVT_CHECKBOX(ID_SHOW_RECENT_OBJECT_TRAJECTORY_BOX, TrackerPanel::showRecentTrajectoryChecked)
    EVT_RADIOBOX(ID_OBJECT_GOALS_TO_SHOW_RADIO, TrackerPanel::goalToShowChanged)
    EVT_TOGGLEBUTTON(ID_EVALUATE_OBJECT_GOALS_BUTTON, TrackerPanel::evaluateGoalsPressed)
    EVT_RADIOBOX(ID_MOTION_PREDICTIONS_TO_SHOW_RADIO, TrackerPanel::predictedTrajToShowChanged)
END_EVENT_TABLE()

tracker::BoundaryType boundary_selection_to_type(int selection);
RigidObjectState rigid_state_to_type(int selection);
TrackingUncertainty uncertainty_radio_to_type(int selection);
PredictionType prediction_radio_to_type(int selection);


TrackerPanel::TrackerPanel(const ui_params_t& params, const tracker_panel_widgets_t& widgets)
: widgets_(widgets)
{
    assert(widgets_.displayWidget);
    assert(widgets_.showLaserObjectsBox);
    assert(widgets_.showLaserPointsBox);
    assert(widgets_.showLaserUncertaintyBox);
    assert(widgets_.boundaryToShowRadio);
    assert(widgets_.showTrackedObjectsBox);
    assert(widgets_.showAccelerationBox);
    assert(widgets_.uncertaintyToShowRadio);
    assert(widgets_.showRecentTrajBox);
    assert(widgets_.goalToShowRadio);
    assert(widgets_.predictionToShowRadio);
    assert(widgets_.predictedTrajDurationText);

    widgets_.displayWidget->showLaserObjects(widgets_.showLaserObjectsBox->IsChecked());
    widgets_.displayWidget->showLaserPoints(widgets_.showLaserPointsBox->IsChecked());
    widgets_.displayWidget->setBoundaryToShow(boundary_selection_to_type(widgets_.boundaryToShowRadio->GetSelection()));

    widgets_.displayWidget->showTrackedObjects(widgets_.showTrackedObjectsBox->IsChecked());
    widgets_.displayWidget->showAccleration(widgets_.showAccelerationBox->IsChecked());
    widgets_.displayWidget->setTrackingUncertaintyToShow(
        uncertainty_radio_to_type(widgets_.uncertaintyToShowRadio->GetSelection()));
    widgets_.displayWidget->showRecentTrajectory(widgets_.showRecentTrajBox->IsChecked());
    widgets_.displayWidget->setGoalPredictionToShow(
        prediction_radio_to_type(widgets_.goalToShowRadio->GetSelection()));
    widgets_.displayWidget->setTrajectoryPredictionToShow(
        prediction_radio_to_type(widgets_.predictionToShowRadio->GetSelection()));
    widgets_.displayWidget->setWidgetParams(params);
}


void TrackerPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widgets_.displayWidget->setStatusBar(statusBar);
    widgets_.displayWidget->setRenderContext(context);
}


void TrackerPanel::subscribe(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<hssh::LocalPerceptualMap>        (widgets_.displayWidget);
    producer.subscribeTo<tracker::DynamicObjectCollection>(widgets_.displayWidget);
    producer.subscribeTo<tracker::LaserObjectCollection>  (widgets_.displayWidget);
    producer.subscribeTo<motion_state_t>           (widgets_.displayWidget);
}


void TrackerPanel::setConsumer(system::ModuleCommunicator* consumer)
{
    // No output from the TrackerPanel, so no need to save the consumer
}


void TrackerPanel::update(void)
{
    long durationMs = 0;
    if(widgets_.predictedTrajDurationText->GetValue().ToLong(&durationMs) && (durationMs > 0))
    {
        widgets_.displayWidget->setPredictedTrajectoryDuration(durationMs);
    }

    widgets_.displayWidget->Refresh();
}


void TrackerPanel::saveSettings(utils::ConfigFileWriter& config)
{
    // TODO
}


void TrackerPanel::loadSettings(const utils::ConfigFile& config)
{
    // TODO
}


void TrackerPanel::followRobotChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->shouldFollowRobot(event.IsChecked());
}


void TrackerPanel::showLaserObjectsChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showLaserObjects(event.IsChecked());
}


void TrackerPanel::showLaserPointsChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showLaserPoints(event.IsChecked());
}


void TrackerPanel::showLaserUncertaintyChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showLaserUncertainty(event.IsChecked());
}


void TrackerPanel::boundaryToShowChanged(wxCommandEvent& event)
{
    widgets_.displayWidget->setBoundaryToShow(boundary_selection_to_type(event.GetSelection()));
}


void TrackerPanel::showTrackedObjectsChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showTrackedObjects(event.IsChecked());
}


void TrackerPanel::showAccelerationChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showAccleration(event.IsChecked());
}


void TrackerPanel::rigidObjectStateToShowChanged(wxCommandEvent& event)
{
    widgets_.displayWidget->setRigidObjectStateToShow(rigid_state_to_type(event.GetSelection()));
}


void TrackerPanel::trackingUncertaintyToShowChanged(wxCommandEvent& event)
{
    widgets_.displayWidget->setTrackingUncertaintyToShow(uncertainty_radio_to_type(event.GetSelection()));
}


void TrackerPanel::showRecentTrajectoryChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showRecentTrajectory(event.IsChecked());
}


void TrackerPanel::goalToShowChanged(wxCommandEvent& event)
{
    widgets_.displayWidget->setGoalPredictionToShow(prediction_radio_to_type(event.GetSelection()));
}


void TrackerPanel::evaluateGoalsPressed(wxCommandEvent& event)
{
    widgets_.displayWidget->shouldEvaluateIntentions(event.IsChecked());
}


void TrackerPanel::predictedTrajToShowChanged(wxCommandEvent& event)
{
    widgets_.displayWidget->setTrajectoryPredictionToShow(prediction_radio_to_type(event.GetSelection()));
}


tracker::BoundaryType boundary_selection_to_type(int selection)
{
    switch(selection)
    {
    case 0:
        return tracker::BoundaryType::best;
    case 1:
        return tracker::BoundaryType::rectangle;
    case 2:
        return tracker::BoundaryType::one_circle;
    case 3:
        return tracker::BoundaryType::two_circles;
    default:
        return tracker::BoundaryType::unknown;
    }

    return tracker::BoundaryType::unknown;
}


RigidObjectState rigid_state_to_type(int selection)
{
    enum RigidStateButton
    {
        fast,
        slow,
    };

    switch(selection)
    {
    case fast:
        return RigidObjectState::fast;

    case slow:
        return RigidObjectState::slow;

    default:
        std::cerr << "ERROR: Unknown rigid object state selected: " << selection << '\n';
        assert((selection >= fast) && (selection <= slow));
    }

    return RigidObjectState::fast;
}


TrackingUncertainty uncertainty_radio_to_type(int selection)
{
    enum UncertaintyButton
    {
        position,
        velocity,
        acceleration,
        none,
    };

    switch(selection)
    {
    case position:
        return TrackingUncertainty::position;

    case velocity:
        return TrackingUncertainty::velocity;

    case acceleration:
        return TrackingUncertainty::acceleration;

    case none:
        return TrackingUncertainty::none;

    default:
        std::cerr << "ERROR: Unknown tracking uncertainty type selected: " << selection << '\n';
        assert((selection >= position) && (selection <= none));
    }

    // Never get here due to assertion
    return TrackingUncertainty::none;
}


PredictionType prediction_radio_to_type(int selection)
{
    enum PredictionRadio
    {
        best,
        all,
        none,
    };

    switch(selection)
    {
    case best:
        return PredictionType::best;

    case all:
        return PredictionType::all;

    case none:
        return PredictionType::none;

    default:
        std::cerr << "ERROR: Unknown prediction type selected: " << selection << '\n';
        assert((selection >= best) && (selection <= none));
    }

    // Never get here due to assertion
    return PredictionType::none;
}

} // namespace ui
} // namespace vulcan

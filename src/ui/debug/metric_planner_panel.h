/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     metric_planner_panel.h
 * \author   Collin Johnson and Jong Jin Park
 *
 * Definition of the MetricPlannerPanel, which provides event handling
 * for the MetricPlanner display tab.
 */

#ifndef UI_DEBUG_METRIC_PLANNER_PANEL_H
#define UI_DEBUG_METRIC_PLANNER_PANEL_H

#include "mpepc/control/control_law_coordinates.h"
#include "ui/common/ui_forward_declarations.h"
#include "ui/common/ui_panel.h"
#include "ui/common/ui_params.h"
#include <memory>
#include <wx/wx.h>

namespace vulcan
{
namespace mpepc
{
struct trajectory_planner_debug_info_t;
struct robot_trajectory_debug_info_t;
struct named_pose_t;
}   // namespace mpepc

namespace ui
{

class MetricPlannerDisplayWidget;
class PoseSelector;

struct metric_planner_panel_widgets_t
{
    MetricPlannerDisplayWidget* displayWidget;

    wxTextCtrl* scriptNameText;
    wxTextCtrl* updatePlanTimeText;
    wxChoice* trajectoryCostChoice;
    wxStaticText* numTrajectoriesLabel;
    wxTextCtrl* numTrajectoriesText;
    wxStaticText* evaluatedCostLabel;
    wxStaticText* evaluatedCostText;
    wxTextCtrl* motionTargetRText;
    wxTextCtrl* motionTargetThetaText;
    wxTextCtrl* motionTargetDeltaText;
    wxTextCtrl* motionTargetGainText;
    wxTextCtrl* motionTargetK1Text;
    wxTextCtrl* motionTargetK2Text;
    wxStaticText* motionTargetCostText;
};


/**
 * MetricPlannerPanel is responsible for event handling in the metric planner panel.
 */
class MetricPlannerPanel : public UIPanel
{
public:
    /**
     * Constructor for MetricPlannerPanel.
     *
     * \param    params          Parameters to the panel and widget
     * \param    widget          MetricPlannerDisplayWidget instance to be controlled by the panel
     */
    MetricPlannerPanel(const ui_params_t& params, const metric_planner_panel_widgets_t& widgets);

    ~MetricPlannerPanel(void);

    // UIPanel interface
    void setup(wxGLContext* context, wxStatusBar* statusBar) override;
    void subscribe(system::ModuleCommunicator& producer) override;
    void setConsumer(system::ModuleCommunicator* consumer) override;
    void update(void) override;
    void saveSettings(utils::ConfigFileWriter& config) override;
    void loadSettings(const utils::ConfigFile& config) override;

private:
    // event handlers
    void showRobotPoseChecked(wxCommandEvent& event);
    void showDestinationPoseChecked(wxCommandEvent& event);
    void showTrackedObjectsChecked(wxCommandEvent& event);
    void showEstimatedObjectMotionsChecked(wxCommandEvent& event);
    void showOptimalPathChecked(wxCommandEvent& event);
    void showVisibilityAnalysisChecked(wxCommandEvent& event);
    void showTopoSituationChecked(wxCommandEvent& event);
    void loadScriptPressed(wxCommandEvent& event);
    void showWaypointsChecked(wxCommandEvent& event);
    void useRRTStarChecked(wxCommandEvent& event);
    void stopAtWaypointsChecked(wxCommandEvent& event);
    void showGraphChecked(wxCommandEvent& event);
    void simpleFollowingChecked(wxCommandEvent& event);
    void ignoreObjectSpeedChecked(wxCommandEvent& event);
    void updateWaypointsPressed(wxCommandEvent& event);
    void sendWaypointsPressed(wxCommandEvent& event);
    void skipWaypointPressed(wxCommandEvent& event);
    void cancelFollowingPressed(wxCommandEvent& event);
    void loopThroughWaypointsPressed(wxCommandEvent& event);
    void mapTypeSelected(wxCommandEvent& event);
    void trjGroupSelected(wxCommandEvent& event);
    void trjCostSelected(wxCommandEvent& event);
    void trjDisplayModeSelected(wxCommandEvent& event);
    void trjNumEntered(wxCommandEvent& event);
    void increaseTrjNumPressed(wxCommandEvent& event);
    void decreaseTrjNumPressed(wxCommandEvent& event);
    void showMotionTargetsChecked(wxCommandEvent& event);
    void overlayRobotPosesChecked(wxCommandEvent& event);
    void selectDestinationPosePressed(wxCommandEvent& event);
    void sendDestinationPosePressed(wxCommandEvent& event);
    void cancelDestinationPosePressed(wxCommandEvent& event);
    void motionTargetREntered(wxCommandEvent& event);
    void motionTargetThetaEntered(wxCommandEvent& event);
    void motionTargetDeltaEntered(wxCommandEvent& event);
    void motionTargetGainEntered(wxCommandEvent& event);
    void motionTargetK1Entered(wxCommandEvent& event);
    void motionTargetK2Entered(wxCommandEvent& event);
    void selectMotionTargetPressed(wxCommandEvent& event);
    void evaluateMotionTargetPressed(wxCommandEvent& event);
    void clearMotionTargetPressed(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()

    void updateMotionTargetTexts(void);

    metric_planner_panel_widgets_t widgets_;   // interface to display widget

    size_t numTrjEvaluated_;   // total number of trajectories evaluated for the most recent cycle
    size_t numTrjToShow_;      // number (last N) of trajectories to show or id of a single trajectory to show

    bool isSelectingDestinationPose_;
    bool isSelectingMotionTarget_;

    std::shared_ptr<PoseSelector> destinationPoseSelector_;
    std::shared_ptr<PoseSelector> motionTargetPoseSelector_;

    mpepc::control_law_coordinates_t controlLawCoords_;
    double velocityGain_;
    mpepc::motion_target_t motionTarget_;

    std::string scriptPath_;
    std::vector<mpepc::named_pose_t> goals_;

    system::ModuleCommunicator* consumer_;

    metric_planner_display_params_t params_;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_DEBUG_METRIC_PLANNER_PANEL_H

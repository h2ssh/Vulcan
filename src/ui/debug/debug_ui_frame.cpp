/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     debug_ui_frame.cpp
 * \author   Collin Johnson
 *
 * Definition of DebugUIFrame.
 */

#include "ui/debug/debug_ui_frame.h"
#include "ui/common/ui_params.h"
#include "ui/debug/calibration_panel.h"
#include "ui/debug/decision_planner_panel.h"
#include "ui/debug/evaluation_panel.h"
#include "ui/debug/exploration_panel.h"
#include "ui/debug/global_metric_panel.h"
#include "ui/debug/global_topo_panel.h"
#include "ui/debug/goal_planner_panel.h"
#include "ui/debug/local_metric_display_widget.h"
#include "ui/debug/local_metric_panel.h"
#include "ui/debug/local_metric_panel_text_updater.h"
#include "ui/debug/local_topo_panel.h"
#include "ui/debug/metric_planner_panel.h"
#include "ui/debug/relocalization_panel.h"
#include "ui/debug/scripting_panel.h"
#include "ui/debug/tracker_panel.h"
#include "ui/debug/vision_panel.h"
#include "utils/timestamp.h"
#include <iostream>

namespace vulcan
{
namespace ui
{

DebugUIFrame::DebugUIFrame(const utils::ConfigFile& config)
: DebugFrame(0)
, lpmPanel_(0)
, localTopoPanel_(0)
, globalTopoPanel_(0)
, relocalizationPanel_(0)
, calibrationPanel_(0)
, metricPlannerPanel_(0)
, decisionPlannerPanel_(0)
, goalPlannerPanel_(0)
, visionPanel_(0)
, explorationPanel_(0)
, evaluationPanel_(0)
{
    ui_params_t params = load_ui_params(config);

    setupLocalMetricPanel(params);
    setupLocalTopologyPanel(params);
    setupGlobalTopologyPanel(params);
    setupGlobalMetricPanel(params);
    setupRelocalizationPanel(params);
    setupScriptingPanel(params);
    setupCalibrationPanel(params);
    setupTrackerPanel(params);
    setupMetricPlannerPanel(params);
    setupDecisionPlannerPanel(params);
    setupGoalPlannerPanel(params);
    setupVisionPanel(params);
    setupSystemPanel(params);
    setupExplorationPanel(params);
    setupEvaluationPanel(params);

    // Now that all panels are setup, do the final initialization for the frame
    initialize(frameNotebook, params.mainFrameParams.framesPerSecond, localMetricWidget, gridCellStatusBar);
}


DebugUIFrame::~DebugUIFrame(void)
{
    // Don't delete the allocated widgets because the UI will segfault -- wxWidgets internally handles all
    // the destruction
}


void DebugUIFrame::setupLocalMetricPanel(const ui_params_t& params)
{
    local_metric_panel_text_widgets_t updaterWidgets;
    updaterWidgets.pose = poseDisplay;
    updaterWidgets.velocity = velocityDisplay;
    updaterWidgets.command = commandDisplay;
    updaterWidgets.imuAccel = imuAccelDisplay;
    updaterWidgets.imuVelocity = imuVelocityDisplay;
    updaterWidgets.imuOrientation = imuOrientationDisplay;
    updaterWidgets.leftWheel = leftWheelDisplay;
    updaterWidgets.rightWheel = rightWheelDisplay;

    LocalMetricPanelTextUpdater* lpmInfo = new LocalMetricPanelTextUpdater(updaterWidgets);

    local_metric_panel_widgets_t widgets;
    widgets.lpmWidget = localMetricWidget;
    widgets.updater = lpmInfo;
    widgets.localizationModeCheck = localMetricModeBox;
    widgets.gridToShowRadio = metricGridToShowRadio;
    widgets.centerOnRobotCheck = centerOnRobotCheckBox;
    widgets.laserToShowRadio = laserToShowRadio;
    widgets.showRaysCheck = scanLineCheckBox;
    widgets.showExtractedCheck = showExtractedLinesCheckBox;
    widgets.showIntensityCheck = showIntensityPlotsCheckBox;
    widgets.showPoseTraceCheck = showPoseTraceCheckBox;
    widgets.showMotionTraceCheck = showMotionTraceCheckBox;
    widgets.showErrorCheck = showUncertaintyEllipseCheckBox;
    widgets.showParticlesCheck = showParticlesCheckBox;
    widgets.showGlassIntensityCheck = showGlassIntensityCheckBox;
    widgets.showWallsCheck = showGlassWallsCheckBox;
    widgets.showAnglesCheck = showGlassAnglesCheckBox;
    widgets.anglesToShowRadio = glassAnglesToShowRadio;
    widgets.flattenThresholdSlider = glassFlattenThreshSlider;
    widgets.highlyVisibleThresholdSlider = glassHighlyVisibleThreshSlider;
    widgets.rotationAngleText = rotateLPMText;
    widgets.mainFrame = this;

    lpmPanel_ = new LocalMetricPanel(params, widgets);

    addPanel(lpmPanel_, localMetricPanel);
}


void DebugUIFrame::setupLocalTopologyPanel(const ui_params_t& params)
{
    local_topo_panel_widgets_t widgets;
    widgets.displayWidget = localTopoWidget;
    widgets.showVoronoiGridBox = showVoronoiGridCheckbox;
    widgets.showDistanceGradientBox = showDistanceGradientCheckBox;
    widgets.showFullSkeletonBox = showFullSkeletonCheckBox;
    widgets.showReducedSkeletonBox = showReducedSkeletonCheckBox;
    widgets.showFrontiersBox = showFrontiersCheckbox;
    widgets.centerOnRobotBox = followRobotTopoCheckBox;
    widgets.gatewayTypeChoice = gatewayTypeChoice;
    widgets.showAreaGraphBox = showAreaGraphCheckBox;
    widgets.showVisibilityGraphBox = showVisibilityGraphBox;
    widgets.areaHypothesisValueRadio = areaHypothesisValueRadio;
    widgets.hypValueText = hypFeatureValueText;
    widgets.hypFeatureChoice = hypFeatureChoice;
    widgets.hypothesesToShowRadio = hypothesesToShowRadio;
    widgets.labelDistributionText = labelDistributionText;
    widgets.cspIterationSlider = cspIterationSlider;
    widgets.cspSpeedSlider = cspSpeedSlider;
    widgets.showHeatMapBox = showLocalHeatMapCheckBox;
    widgets.numPathsText = numHeatMapPathsText;
    widgets.eventsList = localTopoEventList;
    widgets.showEventsVisualizationBox = showLocalTopoEventCheckbox;
    widgets.isovistLocationRadio = isovistLocationRadio;
    widgets.selectIsovistsButton = selectIsovistsButton;
    widgets.isovistScalarChoice = fieldScalarChoice;
    widgets.showIsovistBox = showIsovistBox;
    widgets.showIsovistFieldBox = showIsovistFieldBox;
    widgets.showIsovistDerivFieldBox = showIsovistFieldDerivBox;
    widgets.showGradientsBox = showCellGradientsCheckbox;
    widgets.showLocalMaximaBox = showIsovistMaximaCheckbox;
    widgets.showGatewayProbabilitiesBox = showGatewayProbabilitiesBox;
    widgets.gatewayProbabilitySlider = gatewayCutoffSlider;
    widgets.classifierToUseRadio = gatewayClassifierRadio;
    widgets.visibilityFeatureChoice = visibilityFeatureChoice;

    localTopoPanel_ = new LocalTopoPanel(params, widgets);
    addPanel(localTopoPanel_, localTopologyPanel);
}


void DebugUIFrame::setupGlobalTopologyPanel(const ui_params_t& params)
{
    global_topo_panel_widgets_t widgets;
    widgets.displayWidget = globalTopoWidget;
    widgets.activeMapComboBox = topoHypothesisComboBox;
    widgets.numHypothesesLabel = numActiveHypothesesLabel;
    widgets.numCompleteLabel = numCompleteHypothesesLabel;
    widgets.hypothesisInfoGrid = hypothesisInfoGrid;

    globalTopoPanel_ = new GlobalTopoPanel(params, widgets);
    addPanel(globalTopoPanel_, globalTopologyPanel);
}


void DebugUIFrame::setupGlobalMetricPanel(const ui_params_t& params)
{
    global_metric_panel_widgets_t widgets;
    widgets.display = globalMetricWidget;
    widgets.mapNameText = globalMetricMapNameText;
    widgets.saveMapButton = saveGlobalMapButton;
    widgets.relocalizeButton = globalMetricRelocalizeButton;

    globalMetricPanel_ = new GlobalMetricPanel(widgets, params);
    addPanel(globalMetricPanel_, globalMetricPanel);
}


void DebugUIFrame::setupRelocalizationPanel(const ui_params_t& params)
{
    relocalization_widgets_t widgets;

    widgets.displayWidget = relocalizationWidget;
    widgets.loadGMMButton = relocalizeLoadGMMButton;
    widgets.loadLPMButton = relocalizeLoadLPMButton;
    widgets.showLaserBox = relocalizeShowLaserCheckBox;
    widgets.showErrorBox = relocalizeShowErrorCheckBox;
    widgets.showParticlesBox = relocalizeShowParticlesCheckBox;
    widgets.freeSpacePosesPerCellText = freeSpacePosesPerCellText;
    widgets.freeSpaceStrideText = freeSpaceCellStrideText;
    widgets.regionNumSamplesText = regionNumSamplesText;
    widgets.regionPosesPerPosText = regionPosesPerPositionText;

    relocalizationPanel_ = new RelocalizationPanel(params, widgets);
    addPanel(relocalizationPanel_, relocalizationPanel);
}


void DebugUIFrame::setupScriptingPanel(const ui_params_t& params)
{
    scripting_panel_widgets_t widgets;

    widgets.scriptingWidget = scriptingWidget;
    widgets.poseTaskButton = scriptPoseTargetButton;
    widgets.elevatorTaskButton = scriptElevatorTargetButton;
    widgets.targetSetList = scriptingTargetSetList;
    widgets.targetNameText = scriptTargetNameText;
    widgets.targetPoseText = scriptTargetPoseText;
    widgets.scriptList = scriptTargetsList;

    scriptingPanel_ = new ScriptingPanel(params, widgets);
    addPanel(scriptingPanel_, plannerScriptingPanel);
}


void DebugUIFrame::setupCalibrationPanel(const ui_params_t& params)
{
    calibration_panel_widgets_t widgets;

    widgets.displayWidget = calibrationWidget;
    widgets.showFrontLaser = showFrontLaserCheckBox;
    widgets.showBackLaser = showBackLaserCheckBox;
    widgets.frontLaserTextX = frontLaserCoordsXText;
    widgets.frontLaserTextY = frontLaserCoordsYText;
    widgets.frontLaserTextTheta = frontLaserCoordsThetaText;
    widgets.frontLaserTextPitch = frontLaserPitchText;
    widgets.frontLaserTextRoll = frontLaserRollText;
    widgets.backLaserTextX = backLaserCoordsXText;
    widgets.backLaserTextY = backLaserCoordsYText;
    widgets.backLaserTextTheta = backLaserCoordsThetaText;
    widgets.backLaserTextPitch = backLaserPitchText;
    widgets.backLaserTextRoll = backLaserRollText;

    widgets.calibratePitchButton = calibratePitchButton;
    widgets.calibrateRollButton = calibrateRollButton;
    widgets.tiltLaserToShowRadio = tiltLaserRadio;
    widgets.tiltStartIndex = lineStartIndexText;
    widgets.tiltEndIndex = lineEndIndexText;

    calibrationPanel_ = new CalibrationPanel(params, widgets);
    addPanel(calibrationPanel_, calibrationPanel);
}


void DebugUIFrame::setupTrackerPanel(const ui_params_t& params)
{
    tracker_panel_widgets_t widgets;

    widgets.displayWidget = trackerWidget;

    widgets.showLaserObjectsBox = showLaserObjectsCheckbox;
    widgets.showLaserPointsBox = showLaserObjectPointsCheckbox;
    widgets.showLaserUncertaintyBox = showLaserUncertaintyCheckbox;
    widgets.boundaryToShowRadio = laserObjBoundaryRadio;

    widgets.showTrackedObjectsBox = showTrackedObjectsCheckbox;
    widgets.showAccelerationBox = showTrackedAcclerationCheckbox;
    widgets.uncertaintyToShowRadio = trackingUncertaintyRadio;
    widgets.showRecentTrajBox = showRecentObjectTrajectoryCheckbox;

    widgets.goalToShowRadio = objectGoalsToShowRadio;

    widgets.predictionToShowRadio = motionPredictionsToShowRadio;
    widgets.predictedTrajDurationText = predictionDurationText;

    trackerPanel_ = new TrackerPanel(params, widgets);
    addPanel(trackerPanel_, trackerPanel);
}


void DebugUIFrame::setupMetricPlannerPanel(const ui_params_t& params)
{
    metric_planner_panel_widgets_t widgets;

    widgets.displayWidget = metricPlannerWidget;
    widgets.scriptNameText = metricScriptFileText;
    widgets.trajectoryCostChoice = trajectoryCostChoice;
    widgets.numTrajectoriesLabel = trajectoryCountLabel;
    widgets.numTrajectoriesText = numTrajectoriesText;
    widgets.evaluatedCostLabel = evaluatedCostLabel;
    widgets.evaluatedCostText = evaluatedCostText;
    widgets.motionTargetRText = motionTargetRText;
    widgets.motionTargetThetaText = motionTargetThetaText;
    widgets.motionTargetDeltaText = motionTargetDeltaText;
    widgets.motionTargetGainText = motionTargetGainText;
    widgets.motionTargetK1Text = motionTargetK1Text;
    widgets.motionTargetK2Text = motionTargetK2Text;
    widgets.motionTargetCostText = motionTargetCostText;

    metricPlannerPanel_ = new MetricPlannerPanel(params, widgets);
    addPanel(metricPlannerPanel_, metricPlannerPanel);
}


void DebugUIFrame::setupDecisionPlannerPanel(const ui_params_t& params)
{
    decision_planner_panel_widgets_t widgets;

    widgets.widget = decisionPlannerWidget;
    widgets.commandQueueList = decisionCommandQueueList;
    widgets.placeState = localPlaceStateGrid;
    widgets.pathState = localPathStateGrid;

    decisionPlannerPanel_ = new DecisionPlannerPanel(params, widgets);
    addPanel(decisionPlannerPanel_, decisionPlannerPanel);
}


void DebugUIFrame::setupGoalPlannerPanel(const ui_params_t& params)
{
    goal_planner_panel_widgets_t widgets;

    widgets.displayWidget = goalPlannerWidget;
    widgets.representationBox = globalTopoMapViewRadioBox;
    widgets.routeDisplayBox = globalRouteDisplayRadioBox;
    widgets.animateFPSText = animateFPSText;

    goalPlannerPanel_ = new GoalPlannerPanel(params, widgets);
    addPanel(goalPlannerPanel_, goalPlannerPanel);
}


void DebugUIFrame::setupVisionPanel(const ui_params_t& params)
{
    vision_panel_widgets_t widgets;
    widgets.displayWidget = visionWidget;
    widgets.minEdgeWeightSlider = minEdgeWeightSlider;
    widgets.maxEdgeWeightSlider = maxEdgeWeightSlider;
    widgets.pixelSigmaSlider = pixelSigmaSlider;
    widgets.creditMultiplierSlider = creditMultiplierSlider;
    widgets.filterWidthSlider = filterWidthSlider;

    visionPanel_ = new VisionPanel(params, widgets);
    addPanel(visionPanel_, visionPanel);
}


void DebugUIFrame::setupSystemPanel(const ui_params_t& params)
{
    // TODO: Create system panel
}


void DebugUIFrame::setupExplorationPanel(const ui_params_t& params)
{
    exploration_panel_widgets_t widgets;
    widgets.display = explorationWidget;
    widgets.remainingText = remainingLocalTopoAreasText;
    widgets.visitedText = visitedLocalTopoAreasText;
    widgets.totalText = totalLocalTopoAreasText;
    widgets.centerOnRobotCheckbox = explorationCenterOnRobotCheckbox;

    explorationPanel_ = new ExplorationPanel(params, widgets);
    addPanel(explorationPanel_, explorationPanel);
}


void DebugUIFrame::setupEvaluationPanel(const ui_params_t& params)
{
    evaluation_panel_widgets_t widgets;
    widgets.widget = evaluationWidget;
    widgets.drawBoundaryBox = drawEvalBoundaryBox;
    widgets.drawStarBox = drawEvalStarBox;

    evaluationPanel_ = new EvaluationPanel(params, widgets);
    addPanel(evaluationPanel_, evaluationPanel);
}

}   // namespace ui
}   // namespace vulcan

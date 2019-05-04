/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Mar 28 2016)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __DEBUG_UI_H__
#define __DEBUG_UI_H__

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
class UIMainFrame;
namespace vulcan{ namespace ui{ class CalibrationDisplayWidget; } }
namespace vulcan{ namespace ui{ class DecisionPlannerDisplayWidget; } }
namespace vulcan{ namespace ui{ class EvaluationDisplayWidget; } }
namespace vulcan{ namespace ui{ class ExplorationDisplayWidget; } }
namespace vulcan{ namespace ui{ class GlobalMetricDisplayWidget; } }
namespace vulcan{ namespace ui{ class GlobalTopoDisplayWidget; } }
namespace vulcan{ namespace ui{ class GoalPlannerDisplayWidget; } }
namespace vulcan{ namespace ui{ class LocalMetricDisplayWidget; } }
namespace vulcan{ namespace ui{ class LocalTopoDisplayWidget; } }
namespace vulcan{ namespace ui{ class MetricPlannerDisplayWidget; } }
namespace vulcan{ namespace ui{ class PlannerScriptingWidget; } }
namespace vulcan{ namespace ui{ class RelocalizationDisplayWidget; } }
namespace vulcan{ namespace ui{ class TrackerDisplayWidget; } }
namespace vulcan{ namespace ui{ class VisionDisplayWidget; } }

#include "ui/common/ui_main_frame.h"
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/stattext.h>
#include <wx/statline.h>
#include <wx/sizer.h>
#include <wx/radiobox.h>
#include <wx/checkbox.h>
#include <wx/statbox.h>
#include <wx/button.h>
#include <wx/slider.h>
#include <wx/textctrl.h>
#include <wx/scrolwin.h>
#include <wx/panel.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/choice.h>
#include <wx/gbsizer.h>
#include <wx/tglbtn.h>
#include <wx/listbox.h>
#include <wx/combobox.h>
#include <wx/grid.h>
#include <wx/listctrl.h>
#include <wx/radiobut.h>
#include <wx/aui/auibook.h>
#include <wx/statusbr.h>
#include <wx/menu.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////

namespace vulcan
{
	namespace ui
	{
		#define ID_LOCAL_METRIC_WIDGET 1000
		#define ID_LOCAL_METRIC_MODE_BOX 1001
		#define ID_CENTER_ON_ROBOT 1002
		#define ID_METRIC_GRID_TO_SHOW_RADIO 1003
		#define ID_LASER_TO_SHOW_RADIO 1004
		#define ID_SHOW_LASER_LINES_BOX 1005
		#define ID_SHOW_EXTRACTED_LINES_BOX 1006
		#define ID_SHOW_INTENSITY_PLOTS_BOX 1007
		#define ID_SHOW_POSE_TRACE 1008
		#define ID_SHOW_MOTION_TRACE_BOX 1009
		#define ID_SHOW_UNCERTAINTY_ELLIPSE 1010
		#define ID_SHOW_PARTICLES 1011
		#define ID_CLEAR_POSES_BUTTON 1012
		#define ID_CLEAR_MOTION_BUTTON 1013
		#define ID_SHOW_GLASS_INTENSITY_BOX 1014
		#define ID_SHOW_GLASS_WALLS_BOX 1015
		#define ID_SHOW_GLASS_ANGLES_BOX 1016
		#define ID_GLASS_ANGLES_TO_SHOW_RADIO 1017
		#define ID_RUN_FLATTEN_MAP_BUTTON 1018
		#define ID_RUN_DYNAMIC_FILTER_BUTTON 1019
		#define ID_ROTATE_LPM_BUTTON 1020
		#define ID_SAVE_CURRENT_LPM_BUTTON 1021
		#define ID_SAVE_GLASS_MAP_BUTTON 1022
		#define ID_LOAD_GLASS_MAP_BUTTON 1023
		#define ID_SAVE_POSES_BUTTON 1024
		#define ID_SAVE_SCANS_BUTTON 1025
		#define ID_METRIC_PLANNER_WIDGET 1026
		#define ID_SHOW_ROBOT_POSE_CHECKBOX 1027
		#define ID_SHOW_TRACKED_OBJECTS_CHECKBOX 1028
		#define ID_SHOW_DESTINATION_POSE_CHECKBOX 1029
		#define ID_SHOW_OBJECTS_MOTION_CHECKBOX 1030
		#define ID_SHOW_OPTIMAL_PATH_CHECKBOX 1031
		#define ID_SHOW_VISIBILITY_ANALYSIS_CHECKBOX 1032
		#define ID_SHOW_SITUATIONS_CHECKBOX 1033
		#define ID_SELECT_DESTINATION_POSE_BUTTON 1034
		#define ID_SEND_DESTINATION_POSE_BUTTON 1035
		#define ID_CANCEL_DESTINATION_POSE_BUTTON 1036
		#define ID_METRIC_LOAD_SCRIPT_BUTTON 1037
		#define ID_SEND_WAYPOINTS_BUTTON 1038
		#define ID_SKIP_WAYPOINT_BUTTON 1039
		#define ID_CANCEL_FOLLOWING_BUTTON 1040
		#define ID_LOOP_WAYPOINTS_BUTTON 1041
		#define ID_SELECT_MAP_TYPE_RADIOBOX 1042
		#define ID_SELECT_TRJ_GROUP_RADIOBOX 1043
		#define ID_SELECT_TRJ_COST_CHOICE 1044
		#define ID_SELECT_TRJ_DISPLAY_MODE_RADIOBOX 1045
		#define ID_CHANGE_TRJ_NUM_TEXT 1046
		#define ID_DECREASE_TRJ_NUM_BUTTON 1047
		#define ID_INCREASE_TRJ_NUM_BUTTON 1048
		#define ID_SHOW_MOTION_TARGETS_CHECKBOX 1049
		#define ID_OVERLAY_ROBOT_POSES_CHECKBOX 1050
		#define ID_MOTION_TARGET_R_TEXT 1051
		#define ID_MOTION_TARGET_THETA_TEXT 1052
		#define ID_MOTION_TARGET_DELTA_TEXT 1053
		#define ID_MOTION_TARGET_GAIN_TEXT 1054
		#define ID_MOTION_TARGET_K1_TEXT 1055
		#define ID_MOTION_TARGET_K2_TEXT 1056
		#define ID_SELECT_MOTION_TARGET_BUTTON 1057
		#define ID_EVALUATE_MOTION_TARGET_BUTTON 1058
		#define ID_CLEAR_MOTION_TARGET_BUTTON 1059
		#define ID_SHOW_WAYPOINTS_BOX 1060
		#define ID_USE_RRT_STAR_BOX 1061
		#define ID_STOP_AT_WAYPOINTS_BOX 1062
		#define ID_SHOW_GRAPH_BOX 1063
		#define ID_SIMPLE_FOLLOWING_BOX 1064
		#define ID_IGNORE_OBJECT_SPEED_BOX 1065
		#define ID_UPDATE_PLAN_TIME_TEXT 1066
		#define ID_UPDATE_WAYPOINTS_BUTTON 1067
		#define ID_TRACKER_WIDGET 1068
		#define ID_TRACKER_FOLLOW_ROBOT_BOX 1069
		#define ID_SHOW_LASER_OBJECTS_BOX 1070
		#define ID_SHOW_LASER_OBJECT_POINTS_BOX 1071
		#define ID_SHOW_LASER_UNCERTAINTY_BOX 1072
		#define ID_LASER_OBJ_BOUNDARY_RADIO 1073
		#define ID_SHOW_TRACKED_OBJECTS_BOX 1074
		#define ID_SHOW_TRACKED_ACCELERATION_BOX 1075
		#define ID_RIGID_OBJECT_STATE_RADIO 1076
		#define ID_TRACKING_UNCERTAINTY_RADIO 1077
		#define ID_SHOW_RECENT_OBJECT_TRAJECTORY_BOX 1078
		#define ID_OBJECT_GOALS_TO_SHOW_RADIO 1079
		#define ID_EVALUATE_OBJECT_GOALS_BUTTON 1080
		#define ID_MOTION_PREDICTIONS_TO_SHOW_RADIO 1081
		#define ID_LOCAL_TOPO_WIDGET 1082
		#define ID_LOCAL_TOPO_MODE_RADIO 1083
		#define ID_SHOW_VORONOI_GRID_BOX 1084
		#define ID_SHOW_DISTANCE_GRADIENT_BOX 1085
		#define ID_SHOW_FULL_SKELETON_BOX 1086
		#define ID_SHOW_REDUCED_SKELETON_BOX 1087
		#define ID_FOLLOW_ROBOT_TOPO_BOX 1088
		#define ID_SHOW_FRONTIERS_BOX 1089
		#define ID_SHOW_GATEWAYS_BOX 1090
		#define ID_GATEWAY_TYPE_CHOICE 1091
		#define ID_SHOW_NORMALS_BOX 1092
		#define ID_SHOW_AREAS_BOX 1093
		#define ID_SAVE_LOCAL_TOPO_MAP_BUTTON 1094
		#define ID_LOAD_LOCAL_TOPO_MAP_BUTTON 1095
		#define ID_SEND_LOCAL_TOPO_MAP_BUTTON 1096
		#define ID_SHOW_SMALL_STAR_BOX 1097
		#define ID_SHOW_AREA_GATEWAYS_BOX 1098
		#define ID_SHOW_AREA_GRAPH_BOX 1099
		#define ID_AREA_HYPOTHESIS_VALUE_RADIO 1100
		#define ID_HYP_FEATURE_CHOICE 1101
		#define ID_HYPOTHESES_TO_SHOW_RADIO 1102
		#define ID_CSP_LOAD_BUTTON 1103
		#define ID_CSP_PLAY_BUTTON 1104
		#define ID_CSP_PAUSE_BUTTON 1105
		#define ID_CSP_STOP_BUTTON 1106
		#define ID_CSP_JUMP_TO_START_BUTTON 1107
		#define ID_CSP_PREV_ITERATION_BUTTON 1108
		#define ID_CSP_NEXT_ITERATION_BUTTON 1109
		#define ID_CSP_JUMP_TO_END_BUTTON 1110
		#define ID_CSP_ITERATION_SLIDER 1111
		#define ID_CSP_SPEED_SLIDER 1112
		#define ID_SHOW_LOCAL_HEAT_MAP_BOX 1113
		#define ID_GENERATE_LOCAL_HEAT_MAP_BUTTON 1114
		#define ID_LOCAL_TOPO_EVENT_LIST 1115
		#define ID_SHOW_LOCAL_TOPO_EVENT_BOX 1116
		#define ID_SHOW_ISOVIST_BOX 1117
		#define ID_SHOW_ISOVIST_FIELD_BOX 1118
		#define ID_SHOW_ISOVIST_DERIV_FIELD_BOX 1119
		#define ID_CALCULATE_ISOVISTS_BUTTON 1120
		#define ID_SELECT_ISOVISTS_BUTTON 1121
		#define ID_FIELD_SCALAR_CHOICE 1122
		#define ID_CALCULATE_GRADIENTS_BUTTON 1123
		#define ID_SHOW_CELL_GRADIENTS_BOX 1124
		#define ID_SHOW_ISOVIST_MAXIMA_BOX 1125
		#define ID_LOAD_GATEWAY_CLASSIFIER_BUTTON 1126
		#define ID_CALCULATE_GATEWAY_PROBABILITIES_BUTTON 1127
		#define ID_SHOW_GATEWAY_PROBABILITIES_BOX 1128
		#define ID_GATEWAY_CUTOFF_SLIDER 1129
		#define ID_SHOW_VISIBILITY_GRAPH_BOX 1130
		#define ID_VISIBILITY_FEATURE_CHOICE 1131
		#define ID_GLOBAL_TOPO_WIDGET 1132
		#define ID_GLOBAL_TOPO_MAP_VIEW_RADIO_BOX 1133
		#define ID_SHOW_BEST_TOPO_MAP_CHECK_BOX 1134
		#define ID_TOPO_HYPOTHESIS_COMBO_BOX 1135
		#define ID_PREVIOUS_HYPOTHESIS_BUTTON 1136
		#define ID_NEXT_HYPOTHESIS_BUTTON 1137
		#define ID_USE_GLOBAL_TOPO_MAP_BUTTON 1138
		#define ID_LOAD_GLOBAL_TOPO_MAP_FROM_FILE_BUTTON 1139
		#define ID_SAVE_CURRENT_MAP_BUTTON 1140
		#define ID_SAVE_TREE_BUTTON 1141
		#define ID_LOAD_TREE_BUTTON 1142
		#define ID_SAVE_MAP_CACHE_BUTTON 1143
		#define ID_LOAD_MAP_CACHE_BUTTON 1144
		#define ID_HYPOTHESIS_INFO_GRID 1145
		#define ID_CLEAR_GLOBAL_TOPO_MAPS_BUTTON 1146
		#define ID_EVALUATION_PANEL 1147
		#define ID_EVALUATION_WIDGET 1148
		#define ID_LOAD_EVAL_MAP_BUTTON 1149
		#define ID_IMPORT_STABILITY_LOG_BUTTON 1150
		#define ID_CLEAR_STABILITY_EVAL_BUTTON 1151
		#define ID_DRAW_EVAL_BOUNDARY_BOX 1152
		#define ID_DRAW_EVAL_STAR_BOX 1153
		#define ID_LOAD_MPEPC_RESULTS_FILE_BUTTON 1154
		#define ID_EXPLORATION_PANEL 1155
		#define ID_EXPLORATION_WIDGET 1156
		#define ID_EXPLORATION_CENTER_ON_ROBOT_BOX 1157
		#define ID_RELOCALIZATION_WIDGET 1158
		#define ID_RELOCALIZE_SHOW_LASER_BOX 1159
		#define ID_RELOCALIZE_SHOW_ERROR_BOX 1160
		#define ID_RELOCALIZE_SHOW_PARTICLES_BOX 1161
		#define ID_RELOCALIZE_LOAD_LPM_BUTTON 1162
		#define ID_RELOCALIZE_LOAD_GMM_BUTTON 1163
		#define ID_RELOCALIZATION_MODE_RADIO 1164
		#define ID_SET_REGION_RELOCALIZE_BUTTON 1165
		#define ID_SEND_REGION_MESSAGE_BUTTON 1166
		#define ID_SEND_FREE_SPACE_MESSAGE_BUTTON 1167
		#define ID_SEND_SCAN_MATCHING_MESSAGE_BUTTON 1168
		#define ID_PLANNER_SCRIPTING_WIDGET 1169
		#define ID_SCRIPTING_LOAD_MAP_BUTTON 1170
		#define ID_SCRIPTING_CAPTURE_MAP_BUTTON 1171
		#define ID_SCRIPTING_TARGET_SET_LIST 1172
		#define ID_SCRIPT_POSE_TARGET_BUTTON 1173
		#define ID_SCRIPT_ELEVATOR_TARGET_BUTTON 1174
		#define ID_SCRIPT_TARGET_POSE_TEXT 1175
		#define ID_SCRIPT_TARGET_SELECT_POSE_BUTTON 1176
		#define ID_SCRIPT_TARGET_CURRENT_BUTTON 1177
		#define ID_SCRIPT_CREATE_TARGET_BUTTON 1178
		#define ID_SCRIPT_ERASE_TARGET_BUTTON 1179
		#define ID_SCRIPT_SAVE_TARGETS_BUTTON 1180
		#define ID_SCRIPT_LOAD_TARGETS_BUTTON 1181
		#define ID_SCRIPT_TARGETS_LIST 1182
		#define ID_SCRIPT_ADD_TARGET_BUTTON 1183
		#define ID_SCRIPT_REMOVE_TARGET_BUTTON 1184
		#define ID_SCRIPT_SAVE_BUTTON 1185
		#define ID_SCRIPT_LOAD_BUTTON 1186
		#define ID_SCRIPT_SEND_BUTTON 1187
		#define ID_GLOBAL_METRIC_WIDGET 1188
		#define ID_GLOBAL_METRIC_CAPTURE_LPM_BUTTON 1189
		#define ID_LOAD_LPM_FOR_GLOBAL_METRIC_BUTTON 1190
		#define ID_LOAD_GLOBAL_METRIC_MAP_BUTTON 1191
		#define ID_GLOBAL_METRIC_MAP_NAME_TEXT 1192
		#define ID_SAVE_GLOBAL_MAP_BUTTON 1193
		#define ID_GLOBAL_METRIC_RELOCALIZE_BUTTON 1194
		#define ID_CALIBRATION_WIDGET 1195
		#define ID_SHOW_FRONT_LASER_BOX 1196
		#define ID_SHOW_BACK_LASER_BOX 1197
		#define ID_FRONT_LASER_COORDS_X 1198
		#define ID_FRONT_LASER_COORDS_Y 1199
		#define ID_FRONT_LASER_COORDS_THETA 1200
		#define ID_FRONT_LASER_PITCH_TEXT 1201
		#define ID_FRONT_LASER_ROLL_TEXT 1202
		#define ID_BACK_LASER_COORDS_X 1203
		#define ID_BACK_LASER_COORDS_Y 1204
		#define ID_BACK_LASER_COORDS_THETA 1205
		#define ID_BACK_LASER_PITCH_TEXT 1206
		#define ID_BACK_LASER_ROLL_TEXT 1207
		#define ID_LOAD_TILT_DATA_BUTTON 1208
		#define ID_TILT_LASER_RADIO 1209
		#define ID_LINE_START_INDEX_TEXT 1210
		#define ID_LINE_END_INDEX_TEXT 1211
		#define ID_CALIBRATE_PITCH_BUTTON 1212
		#define ID_CALIBRATE_ROLL_BUTTON 1213
		#define ID_DECISION_PLANNER_WIDGET 1214
		#define ID_DECISION_COMMAND_QUEUE_LIST 1215
		#define ID_FORWARD_TARGET_BUTTON 1216
		#define ID_LEFT_TARGET_BUTTON 1217
		#define ID_RIGHT_TARGET_BUTTON 1218
		#define ID_BACK_TARGET_BUTTON 1219
		#define ID_PATH_TARGET_START_BUTTON 1220
		#define ID_PATH_TARGET_END_BUTTON 1221
		#define ID_REMOVE_RELATIVE_TARGET_BUTTON 1222
		#define ID_CLEAR_RELATIVE_TARGETS_BUTTON 1223
		#define ID_SEND_RELATIVE_TARGETS_BUTTON 1224
		#define ID_GOAL_PLANNER_WIDGET 1225
		#define ID_MAP_REPRESENTATION_RADIO_BOX 1226
		#define ID_GLOBAL_ROUTE_DISPLAY_RADIO_BOX 1227
		#define ID_SET_LOCATION_BUTTON 1228
		#define ID_SEND_LOCATION_BUTTON 1229
		#define ID_SET_GOAL_BUTTON 1230
		#define ID_SEND_GOAL_BUTTON 1231
		#define ID_ANIMATE_FPS_TEXT 1232
		#define ID_ANIMATE_SEARCH_BUTTON 1233
		#define ID_CONFIRM_ROUTE_BUTTON 1234
		#define ID_CANCEL_ROUTE_BUTTON 1235
		#define ID_VISION_WIDGET 1236
		#define ID_SHOW_SEGMENTS_BOX 1237
		#define ID_MIN_EDGE_WEIGHT_SLIDER 1238
		#define ID_MAX_EDGE_WEIGHT_SLIDER 1239
		#define ID_PIXEL_SIGMA_SLIDER 1240
		#define ID_CREDIT_MULTIPLIER_SLIDER 1241
		#define ID_FILTER_WIDTH_SLIDER 1242
		#define ID_GRID_CELL_STATUS_BAR 1243
		#define ID_LOCAL_METRIC_WINDOW_ITEM 1244
		#define ID_LOCAL_TOPOLOGY_WINDOW_ITEM 1245
		#define ID_GLOBAL_TOPOLOGY_WINDOW_ITEM 1246
		#define ID_RELOCALIZATION_WINDOW_ITEM 1247
		#define ID_METRIC_PLANNER_WINDOW_ITEM 1248
		#define ID_DECISION_PLANNER_WINDOW_ITEM 1249
		#define ID_GOAL_PLANNER_WINDOW_ITEM 1250
		#define ID_VISION_WINDOW_ITEM 1251
		#define ID_SYSTEM_WINDOW_ITEM 1252
		
		///////////////////////////////////////////////////////////////////////////////
		/// Class DebugFrame
		///////////////////////////////////////////////////////////////////////////////
		class DebugFrame : public UIMainFrame
		{
			private:
			
			protected:
				wxAuiNotebook* frameNotebook;
				wxPanel* localMetricPanel;
				LocalMetricDisplayWidget* localMetricWidget;
				wxStaticText* poseLabel;
				wxStaticText* poseDisplay;
				wxStaticText* measuredVelLabel;
				wxStaticText* velocityDisplay;
				wxStaticText* imuLabel;
				wxStaticText* imuAccelDisplay;
				wxStaticText* commandLabel;
				wxStaticText* commandDisplay;
				wxStaticText* leftWheelLabel;
				wxStaticText* leftWheelDisplay;
				wxStaticText* imuVelocityLabel;
				wxStaticText* imuVelocityDisplay;
				wxStaticLine* m_staticline2;
				wxStaticLine* m_staticline3;
				wxStaticText* rightWheelLabel;
				wxStaticText* rightWheelDisplay;
				wxStaticText* imuOrientationLabel;
				wxStaticText* imuOrientationDisplay;
				wxScrolledWindow* localMetricScrollWindow;
				wxRadioBox* localMetricModeBox;
				wxCheckBox* centerOnRobotCheckBox;
				wxRadioBox* metricGridToShowRadio;
				wxRadioBox* laserToShowRadio;
				wxCheckBox* scanLineCheckBox;
				wxCheckBox* showExtractedLinesCheckBox;
				wxCheckBox* showIntensityPlotsCheckBox;
				wxCheckBox* showPoseTraceCheckBox;
				wxCheckBox* showMotionTraceCheckBox;
				wxCheckBox* showUncertaintyEllipseCheckBox;
				wxCheckBox* showParticlesCheckBox;
				wxButton* clearPosesButton;
				wxButton* clearMotionButton;
				wxCheckBox* showGlassIntensityCheckBox;
				wxCheckBox* showGlassWallsCheckBox;
				wxCheckBox* showGlassAnglesCheckBox;
				wxRadioBox* glassAnglesToShowRadio;
				wxStaticText* flattenThreshLabel;
				wxSlider* glassFlattenThreshSlider;
				wxStaticText* highlyVisibleGlassThreshLabel;
				wxSlider* glassHighlyVisibleThreshSlider;
				wxButton* runFlattenMapButton;
				wxButton* runDynamicFilterButton;
				wxTextCtrl* rotateLPMText;
				wxButton* rotateLPMButton;
				wxButton* saveCurrentLPMButton;
				wxButton* saveGlassMapButton;
				wxButton* loadGlassMapButton;
				wxButton* savePosesButton;
				wxButton* saveScansButton;
				wxPanel* metricPlannerPanel;
				MetricPlannerDisplayWidget* metricPlannerWidget;
				wxScrolledWindow* metricPlannerScrollWindow;
				wxCheckBox* showRobotCheckBox;
				wxCheckBox* showTrackedObjectsCheckBox;
				wxCheckBox* showDestinationPoseCheckBox;
				wxCheckBox* showObjectsMotionCheckBox;
				wxCheckBox* showOptimalPathCheckBox;
				wxCheckBox* showVisibilityAnalysisCheckBox;
				wxCheckBox* showSituationsCheckBox;
				wxButton* selectDestinationPoseButton;
				wxButton* sendDestinationPoseButton;
				wxButton* cancelDestinationPoseButton;
				wxTextCtrl* metricScriptFileText;
				wxButton* metricLoadScriptButton;
				wxButton* sendWaypointsButton;
				wxButton* skipWaypointButton;
				wxButton* cancelFollowingButton;
				wxButton* loopWaypointsButton;
				wxRadioBox* selectMapTypeRadioBox;
				wxRadioBox* selectTrjGroupRadioBox;
				wxStaticText* trjCostChoiceLabel;
				wxChoice* trajectoryCostChoice;
				wxRadioBox* trjDisplayModeRadioBox;
				wxStaticText* trajectoryCountLabel;
				wxTextCtrl* numTrajectoriesText;
				wxButton* decreaseTrajectoriesButton;
				wxButton* increaseTrajectoriesButton;
				wxCheckBox* showMotionTargetCheckBox;
				wxCheckBox* overlayRobotPosesCheckBox;
				wxStaticText* evaluatedCostLabel;
				wxStaticText* evaluatedCostText;
				wxStaticText* motionTargetRLabel;
				wxTextCtrl* motionTargetRText;
				wxStaticText* motionTargetThetaLabel;
				wxTextCtrl* motionTargetThetaText;
				wxStaticText* motionTargetDeltaLabel;
				wxTextCtrl* motionTargetDeltaText;
				wxStaticText* motionTargetGainLabel;
				wxTextCtrl* motionTargetGainText;
				wxStaticText* motionTargetK1Label;
				wxTextCtrl* motionTargetK1Text;
				wxStaticText* motionTargetK2Label;
				wxTextCtrl* motionTargetK2Text;
				wxStaticText* motionTargetCostText;
				wxButton* selectMotionTargetButton;
				wxButton* evaluateMotionTargetButton;
				wxButton* clearMotionTargetButton;
				wxCheckBox* showWaypointsCheckBox;
				wxCheckBox* useRRTStarCheckBOx;
				wxCheckBox* stopAtWaypointsCheckBox;
				wxCheckBox* showGraphCheckBox;
				wxCheckBox* SimpleFollowingCheckBox;
				wxCheckBox* ignoreObjectSpeedCheckBox;
				wxTextCtrl* updatePlanTimeText;
				wxButton* updateWaypointsButton;
				wxPanel* trackerPanel;
				TrackerDisplayWidget* trackerWidget;
				wxScrolledWindow* trackerPanelScrollWindow;
				wxCheckBox* trackerFollowRobotCheckbox;
				wxCheckBox* showLaserObjectsCheckbox;
				wxCheckBox* showLaserObjectPointsCheckbox;
				wxCheckBox* showLaserUncertaintyCheckbox;
				wxRadioBox* laserObjBoundaryRadio;
				wxCheckBox* showTrackedObjectsCheckbox;
				wxCheckBox* showTrackedAcclerationCheckbox;
				wxRadioBox* rigidObjectStateRadio;
				wxRadioBox* trackingUncertaintyRadio;
				wxCheckBox* showRecentObjectTrajectoryCheckbox;
				wxRadioBox* objectGoalsToShowRadio;
				wxToggleButton* evaluateObjectGoalsButton;
				wxRadioBox* motionPredictionsToShowRadio;
				wxStaticText* predictionDurationLabel;
				wxTextCtrl* predictionDurationText;
				wxPanel* localTopologyPanel;
				LocalTopoDisplayWidget* localTopoWidget;
				wxScrolledWindow* localTopoScrollWindow;
				wxRadioBox* localTopoModeRadio;
				wxCheckBox* showVoronoiGridCheckbox;
				wxCheckBox* showDistanceGradientCheckBox;
				wxCheckBox* showFullSkeletonCheckBox;
				wxCheckBox* showReducedSkeletonCheckBox;
				wxCheckBox* followRobotTopoCheckBox;
				wxCheckBox* showFrontiersCheckbox;
				wxCheckBox* showGatewaysCheckBox;
				wxChoice* gatewayTypeChoice;
				wxCheckBox* showNormalsCheckbox;
				wxRadioBox* showAreasRadioBox;
				wxButton* saveLocalTopoMapButton;
				wxButton* loadLocalTopoMapButton;
				wxButton* sendLocalTopoMapButton;
				wxCheckBox* showSmallStarCheckBox;
				wxCheckBox* showAreaGatewaysCheckBox;
				wxCheckBox* showAreaGraphCheckBox;
				wxRadioBox* areaHypothesisValueRadio;
				wxStaticText* hypFeatureLabel;
				wxStaticText* hypFeatureValueText;
				wxChoice* hypFeatureChoice;
				wxRadioBox* hypothesesToShowRadio;
				wxStaticText* labelDistributionText;
				wxButton* cspLoadButton;
				wxButton* cspPlayButton;
				wxButton* cspPauseButton;
				wxButton* cspStopButton;
				wxButton* cspJumpToStartButton;
				wxButton* cspPrevIterationButton;
				wxButton* cspNextIterationButton;
				wxButton* cspJumpToEndButton;
				wxSlider* cspIterationSlider;
				wxSlider* cspSpeedSlider;
				wxCheckBox* showLocalHeatMapCheckBox;
				wxStaticText* numPathsHeatMapLabel;
				wxTextCtrl* numHeatMapPathsText;
				wxButton* generateLocalHeatMapButton;
				wxListBox* localTopoEventList;
				wxCheckBox* showLocalTopoEventCheckbox;
				wxCheckBox* showIsovistBox;
				wxCheckBox* showIsovistFieldBox;
				wxCheckBox* showIsovistFieldDerivBox;
				wxRadioBox* isovistLocationRadio;
				wxButton* calculateIsovistsButton;
				wxToggleButton* selectIsovistsButton;
				wxStaticText* scalarChoiceLabel;
				wxChoice* fieldScalarChoice;
				wxButton* calculateGradientsButton;
				wxCheckBox* showCellGradientsCheckbox;
				wxCheckBox* showIsovistMaximaCheckbox;
				wxButton* loadGatewayClassifierButton;
				wxButton* calculateGatewayProbabilityButton;
				wxCheckBox* showGatewayProbabilitiesBox;
				wxStaticText* gatewayProbCutoffLabel;
				wxSlider* gatewayCutoffSlider;
				wxRadioBox* gatewayClassifierRadio;
				wxCheckBox* showVisibilityGraphBox;
				wxStaticText* visibilityFeatureChoiceLabel;
				wxChoice* visibilityFeatureChoice;
				wxPanel* globalTopologyPanel;
				GlobalTopoDisplayWidget* globalTopoWidget;
				wxScrolledWindow* globalTopoScrolledWindow;
				wxRadioBox* globalTopoMapViewRadioBox;
				wxStaticText* activeHypothesesText;
				wxStaticText* numActiveHypothesesLabel;
				wxStaticText* numCompleteHypothesesText;
				wxStaticText* numCompleteHypothesesLabel;
				wxCheckBox* showBestTopoMapCheckBox;
				wxStaticText* mapToShowLabel;
				wxComboBox* topoHypothesisComboBox;
				wxButton* previousHypothesisButton;
				wxButton* nextHypothesisButton;
				wxButton* useGlobalTopoMapButton;
				wxButton* loadGlobalTopoMapButton;
				wxButton* saveCurrentMapButton;
				wxButton* saveTreeButton;
				wxButton* loadTreeButton;
				wxButton* saveMapCacheButton;
				wxButton* loadMapCacheButton;
				wxGrid* hypothesisInfoGrid;
				wxButton* clearGlobalTopoMapsButton;
				wxPanel* evaluationPanel;
				EvaluationDisplayWidget* evaluationWidget;
				wxScrolledWindow* evaluationScrollWindow;
				wxButton* loadEvalMapButton;
				wxButton* importStabilityLogButton;
				wxButton* clearStabilityEvalButton;
				wxCheckBox* drawEvalBoundaryBox;
				wxCheckBox* drawEvalStarBox;
				wxButton* loadMPEPCResultsFileButton;
				wxPanel* explorationPanel;
				ExplorationDisplayWidget* explorationWidget;
				wxStaticText* totalLocalTopoAreasLabel;
				wxStaticText* totalLocalTopoAreasText;
				wxStaticText* visitedLocalTopoAreasLabel;
				wxStaticText* visitedLocalTopoAreasText;
				wxStaticText* remainingLocalTopoAreasLabel;
				wxStaticText* remainingLocalTopoAreasText;
				wxCheckBox* explorationCenterOnRobotCheckbox;
				wxPanel* relocalizationPanel;
				RelocalizationDisplayWidget* relocalizationWidget;
				wxScrolledWindow* relocalizationScrollWindow;
				wxCheckBox* relocalizeShowLaserCheckBox;
				wxCheckBox* relocalizeShowErrorCheckBox;
				wxCheckBox* relocalizeShowParticlesCheckBox;
				wxButton* relocalizeLoadLPMButton;
				wxButton* relocalizeLoadGMMButton;
				wxRadioBox* relocalizeModeRadio;
				wxStaticText* regionNumSamplesLabel;
				wxTextCtrl* regionNumSamplesText;
				wxStaticText* regionPosesPerPosLabel;
				wxTextCtrl* regionPosesPerPositionText;
				wxToggleButton* setRegionRelocalizeButton;
				wxButton* sendRegionMessageButton;
				wxStaticText* freeSpaceCellStrideLabel;
				wxTextCtrl* freeSpaceCellStrideText;
				wxStaticText* freeSpacePosesPerCellLabel;
				wxTextCtrl* freeSpacePosesPerCellText;
				wxButton* sendFreeSpaceMessageButton;
				wxButton* sendScanMatchingMessageButton;
				wxPanel* plannerScriptingPanel;
				PlannerScriptingWidget* scriptingWidget;
				wxScrolledWindow* scriptingOptionsScrolledWindow;
				wxButton* scriptingLoadMapButton;
				wxButton* scriptingCaptureMapButton;
				wxListCtrl* scriptingTargetSetList;
				wxStaticText* scriptTargetNameLabel;
				wxTextCtrl* scriptTargetNameText;
				wxStaticText* scriptTargetTypeLael;
				wxRadioButton* scriptPoseTargetButton;
				wxRadioButton* scriptElevatorTargetButton;
				wxStaticText* scriptTargetPoseLabel;
				wxTextCtrl* scriptTargetPoseText;
				wxButton* scriptTargetSelectPoseButton;
				wxButton* scriptTargetCurrentButton;
				wxButton* scriptCreateTargetButton;
				wxButton* scriptEraseTargetButton;
				wxButton* scriptSaveTargetsButton;
				wxButton* scriptLoadTargetsButton;
				wxListBox* scriptTargetsList;
				wxButton* scriptAddTargetButton;
				wxButton* scriptRemoveTargetButton;
				wxButton* scriptSaveButton;
				wxButton* scriptLoadButton;
				wxButton* scriptSendButton;
				wxPanel* globalMetricPanel;
				GlobalMetricDisplayWidget* globalMetricWidget;
				wxScrolledWindow* globalMetricScrolledWindow;
				wxButton* globalMetricCaptureLPMButton;
				wxButton* loadLPMForGlobalMetricButton;
				wxButton* loadGlobalMetricMapButton;
				wxTextCtrl* globalMetricMapNameText;
				wxButton* saveGlobalMapButton;
				wxButton* globalMetricRelocalizeButton;
				wxPanel* calibrationPanel;
				CalibrationDisplayWidget* calibrationWidget;
				wxScrolledWindow* calibrationScrollWindow;
				wxCheckBox* showFrontLaserCheckBox;
				wxCheckBox* showBackLaserCheckBox;
				wxStaticText* frontLaserCoordsXLabel;
				wxTextCtrl* frontLaserCoordsXText;
				wxStaticText* frontLaserCoordsYLabel;
				wxTextCtrl* frontLaserCoordsYText;
				wxStaticText* frontLaserCoordsThetaLabel;
				wxTextCtrl* frontLaserCoordsThetaText;
				wxStaticText* frontLaserPitchLabel;
				wxTextCtrl* frontLaserPitchText;
				wxStaticText* frontLaserRollLabel;
				wxTextCtrl* frontLaserRollText;
				wxStaticText* backLaserCoordsXLabel;
				wxTextCtrl* backLaserCoordsXText;
				wxStaticText* backLaserCoordsYLabel;
				wxTextCtrl* backLaserCoordsYText;
				wxStaticText* backLaserCoordsThetaLabel;
				wxTextCtrl* backLaserCoordsThetaText;
				wxStaticText* backLaserPitchLabel;
				wxTextCtrl* backLaserPitchText;
				wxStaticText* backLaserRollLabel;
				wxTextCtrl* backLaserRollText;
				wxButton* loadTiltDataButton;
				wxRadioBox* tiltLaserRadio;
				wxStaticText* lineStartIndexLabel;
				wxTextCtrl* lineStartIndexText;
				wxStaticText* lineEndIndexLabel;
				wxTextCtrl* lineEndIndexText;
				wxButton* calibratePitchButton;
				wxButton* calibrateRollButton;
				wxPanel* decisionPlannerPanel;
				DecisionPlannerDisplayWidget* decisionPlannerWidget;
				wxScrolledWindow* decisionPlannerScrollWindow;
				wxListBox* decisionCommandQueueList;
				wxGrid* localPlaceStateGrid;
				wxGrid* localPathStateGrid;
				wxButton* forwardTargetButton;
				wxButton* leftTargetButton;
				wxButton* rightTargetButton;
				wxButton* backTargetButton;
				wxButton* pathTargetStartButton;
				wxButton* pathTargetEndButton;
				wxButton* removeRelativeTargetButton;
				wxButton* clearRelativeTargetsButton;
				wxButton* sendRelativeTargetsButton;
				wxPanel* goalPlannerPanel;
				GoalPlannerDisplayWidget* goalPlannerWidget;
				wxScrolledWindow* goalPlannerScrollWindow;
				wxRadioBox* mapRepresentationRadioBox;
				wxRadioBox* globalRouteDisplayRadioBox;
				wxButton* setLocationButton;
				wxButton* sendLocationButton;
				wxButton* setGoalButton;
				wxButton* sendGoalButton;
				wxTextCtrl* animateFPSText;
				wxButton* animateSearchButton;
				wxButton* confirmRouteButton;
				wxButton* cancelRouteButton;
				wxPanel* visionPanel;
				VisionDisplayWidget* visionWidget;
				wxCheckBox* showSegmentsCheckBox;
				wxStaticText* minEdgeWeightLabel;
				wxSlider* minEdgeWeightSlider;
				wxStaticText* maxEdgeWeightLabel;
				wxSlider* maxEdgeWeightSlider;
				wxStaticText* pixelSigmaLabel;
				wxSlider* pixelSigmaSlider;
				wxStaticText* creditMultiplierLabel;
				wxSlider* creditMultiplierSlider;
				wxStaticText* filterWidthLabel;
				wxSlider* filterWidthSlider;
				wxPanel* systemPanel;
				wxGrid* moduleStatusGrid;
				wxStatusBar* gridCellStatusBar;
				wxMenuBar* debugUIMenu;
				wxMenu* windowMenu;
				
				// Virtual event handlers, overide them in your derived class
				virtual void numTrajectoriesTextOnText( wxCommandEvent& event ) { event.Skip(); }
				
			
			public:
				
				DebugFrame( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Vulcan Debug UI"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 1178,914 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
				
				~DebugFrame();
			
		};
		
	} // namespace ui
} // namespace vulcan

#endif //__DEBUG_UI_H__

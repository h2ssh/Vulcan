/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     debug_ui_frame.h
* \author   Collin Johnson
*
* Definition of DebugUIFrame, the main class for the DebugUI. Handles
* initialization of the display panels.
*/

#ifndef UI_DEBUG_DEBUG_UI_FRAME_H
#define UI_DEBUG_DEBUG_UI_FRAME_H

#include <ui/debug/debug_ui.h>

namespace vulcan
{
namespace utils { class ConfigFile; }
namespace ui
{

class  LocalMetricPanel;
class  LocalMetricPanelTextUpdater;
class  LocalTopoPanel;
class  GlobalTopoPanel;
class  GlobalMetricPanel;
class  RelocalizationPanel;
class  ScriptingPanel;
class  CalibrationPanel;
class  TrackerPanel;
class  MetricPlannerPanel;
class  DecisionPlannerPanel;
class  GoalPlannerPanel;
class  VisionPanel;
class  SystemPanel;
class  ExplorationPanel;
class  EvaluationPanel;
struct ui_params_t;

class DebugUIFrame : public DebugFrame
{
public:

    DebugUIFrame(const utils::ConfigFile& config);

    virtual ~DebugUIFrame(void);

private:

    LocalMetricPanel*     lpmPanel_;
    LocalTopoPanel*       localTopoPanel_;
    GlobalTopoPanel*      globalTopoPanel_;
    GlobalMetricPanel*    globalMetricPanel_;
    RelocalizationPanel*  relocalizationPanel_;
    ScriptingPanel*       scriptingPanel_;
    CalibrationPanel*     calibrationPanel_;
    TrackerPanel*         trackerPanel_;
    MetricPlannerPanel*   metricPlannerPanel_;
    DecisionPlannerPanel* decisionPlannerPanel_;
    GoalPlannerPanel*     goalPlannerPanel_;
    VisionPanel*          visionPanel_;
    SystemPanel*          systemPanel_;
    ExplorationPanel*     explorationPanel_;
    EvaluationPanel*      evaluationPanel_;

    void setupLocalMetricPanel    (const ui_params_t& params);
    void setupLocalTopologyPanel  (const ui_params_t& params);
    void setupGlobalTopologyPanel (const ui_params_t& params);
    void setupGlobalMetricPanel   (const ui_params_t& params);
    void setupRelocalizationPanel (const ui_params_t& params);
    void setupScriptingPanel      (const ui_params_t& params);
    void setupCalibrationPanel    (const ui_params_t& params);
    void setupTrackerPanel        (const ui_params_t& params);
    void setupMetricPlannerPanel  (const ui_params_t& params);
    void setupDecisionPlannerPanel(const ui_params_t& params);
    void setupGoalPlannerPanel    (const ui_params_t& params);
    void setupVisionPanel         (const ui_params_t& params);
    void setupSystemPanel         (const ui_params_t& params);
    void setupExplorationPanel    (const ui_params_t& params);
    void setupEvaluationPanel     (const ui_params_t& params);
};

}
}

#endif // UI_DEBUG_DEBUG_UI_FRAME_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     planner_scripting_widget.h
 * \author   Collin Johnson
 *
 * Definition of PlannerScriptingWidget.
 */

#ifndef UI_DEBUG_PLANNER_SCRIPTING_WIDGET_H
#define UI_DEBUG_PLANNER_SCRIPTING_WIDGET_H

#include "mpepc/metric_planner/script/target_set.h"
#include "ui/common/ui_color.h"
#include "ui/components/grid_based_display_widget.h"
#include "utils/mutex.h"
#include <memory>

namespace vulcan
{
namespace hssh
{
class LocalPerceptualMap;
}
namespace ui
{

class OccupancyGridRenderer;
class PoseTargetRenderer;
class RobotRenderer;
struct ui_params_t;

/**
 * PlannerScriptingWidget
 */
class PlannerScriptingWidget : public GridBasedDisplayWidget
{
public:
    PlannerScriptingWidget(wxWindow* parent,
                           wxWindowID id = wxID_ANY,
                           const wxPoint& pos = wxDefaultPosition,
                           const wxSize& size = wxDefaultSize,
                           long style = 0,
                           const wxString& name = wxString((const wxChar*)("GLCanvas")),
                           const wxPalette& palette = wxNullPalette);

    void setParams(const ui_params_t& params);

    void showHoverTarget(bool show) { shouldShowHover = show; }
    void showSelectedTarget(bool show) { shouldShowSelected = show; }

    void setLPM(std::shared_ptr<hssh::LocalPerceptualMap> map);
    void setCurrentPose(const pose_t& pose);
    void setCompletedTargets(const std::vector<mpepc::named_pose_t>& targets);
    void setHoverTarget(const pose_t& pose);
    void setSelectedTarget(const pose_t& pose);
    void clearHoverTarget(void) { haveHover = false; }
    void clearSelectedTarget(void) { haveSelected = false; }

    // GridBasedDisplayWidget interface
    virtual Point<int> convertWorldToGrid(const Point<float>& world) const;

private:
    std::shared_ptr<hssh::LocalPerceptualMap> lpm;
    utils::Mutex lpmLock;

    pose_t currentPose_;
    pose_t hoverTarget;
    pose_t selectedTarget;
    std::vector<mpepc::named_pose_t> targets;

    bool haveNewLPM;
    bool haveHover;
    bool haveSelected;

    bool shouldShowHover;
    bool shouldShowSelected;

    GLColor hoverColor;
    GLColor selectedColor;
    GLColor targetColor;
    GLColor arrowColor;

    std::unique_ptr<OccupancyGridRenderer> lpmRenderer;
    std::unique_ptr<PoseTargetRenderer> targetRenderer;
    std::unique_ptr<RobotRenderer> robotRenderer_;

    // OpenGLWidget interface
    virtual void renderWidget(void);
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_DEBUG_PLANNER_SCRIPTING_WIDGET_H

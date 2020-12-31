/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     decision_planner_display_widget.h
 * \author   Collin Johnson
 *
 * Declaration of DecisionPlannerDisplayWidget.
 */

#ifndef UI_DEBUG_DECISION_PLANNER_DISPLAY_WIDGET_H
#define UI_DEBUG_DECISION_PLANNER_DISPLAY_WIDGET_H

#include "core/pose.h"
#include "hssh/local_metric/lpm.h"
#include "ui/components/grid_based_display_widget.h"
#include "utils/mutex.h"

namespace vulcan
{
namespace ui
{

class OccupancyGridRenderer;
class DecisionPlanRenderer;
class RobotRenderer;

struct lpm_display_params_t;
struct local_topo_display_params_t;
struct decision_planner_display_params_t;

/**
 * DecisionPlannerDisplayWidget provides debugging visualizations for the decision_planner module. The
 * decision planner is a simple module that spits out a DecisionPlan whenever it decides to do something.
 * These plans tell the robot to drive to one end of a hallway or to pass through a particular gateway at
 * a place. The visualization shows the assigned metric_pose_target_t, along with the entry gateway
 * and exit gateway for the place. When navigating a path, the only the target at the end of the hall
 * is currently shown.
 */
class DecisionPlannerDisplayWidget : public GridBasedDisplayWidget
{
public:
    DecisionPlannerDisplayWidget(wxWindow* parent,
                                 wxWindowID id = wxID_ANY,
                                 const wxPoint& pos = wxDefaultPosition,
                                 const wxSize& size = wxDefaultSize,
                                 long style = 0,
                                 const wxString& name = wxString((const wxChar*)("GLCanvas")),
                                 const wxPalette& palette = wxNullPalette);

    virtual ~DecisionPlannerDisplayWidget(void);


    void setWidgetParams(const lpm_display_params_t& lpmParams,
                         const local_topo_display_params_t& localTopoParams,
                         const decision_planner_display_params_t& plannerParams);

    void setLPM(const hssh::LocalPerceptualMap& lpm);
    void setPose(const pose_t& pose);

private:
    // GridBasedDisplayWidget interface
    virtual void renderWidget(void);
    virtual Point<int> convertWorldToGrid(const Point<float>& world) const;

    OccupancyGridRenderer* lpmRenderer;
    DecisionPlanRenderer* planRenderer;
    RobotRenderer* robotRenderer;

    hssh::LocalPerceptualMap lpm;
    pose_t pose;

    bool haveNewLPM;
    bool havePlan;

    utils::Mutex dataLock;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_DEBUG_DECISION_PLANNER_DISPLAY_WIDGET_H

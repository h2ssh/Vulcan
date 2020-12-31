/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     navigation_interface_display.h
 * \author   Collin Johnson
 *
 * Declaration of NavigationInterfaceDisplay.
 */

#ifndef UI_NAVIGATION_NAVIGATION_INTERFACE_DISPLAY_H
#define UI_NAVIGATION_NAVIGATION_INTERFACE_DISPLAY_H

#include "hssh/local_metric/lpm.h"
#include "hssh/local_metric/pose.h"
#include "hssh/local_topological/local_topo_map.h"
#include "mpepc/trajectory/trajectory_planner_info.h"
#include "planner/interface/decision.h"
#include "tracker/dynamic_object_collection.h"
#include "ui/common/color_interpolator.h"
#include "ui/components/grid_based_display_widget.h"
#include "utils/locked_double_buffer.h"

namespace vulcan
{
namespace ui
{

class GatewaysRenderer;
class RobotRenderer;
class OccupancyGridRenderer;
class RobotTrajectoryRenderer;
class DynamicObjectRenderer;
class LocalAreaRenderer;

/**
 * NavigationInterfaceMode controls the mode in which the display currently operates. There are three modes:
 *
 *   - drive : the robot is driving to a target. keep the robot centered and pointing to the top of the map
 *   - select : the user is selecting goals. allow panning and start zoomed to the entire map
 *   - preview : the user is previewing a goal. zoom to include robot, full path, and goal.
 */
enum class NavigationInterfaceMode
{
    drive,
    select,
    preview,
};

/**
 * NavigationInterfaceDisplay displays information needed to describe the current decision state and to allow the user
 * to see what options are available to them.
 *
 * Unlike other visualizations, this display keeps the robot centered and facing forward.
 *
 * The following information is always displayed:
 *
 *   - The current LPM
 *   - The robot's position in the LPM
 *   - The arrows corresponding to the available Decisions
 *
 * The following information can be optionally displayed:
 *
 *   - DynamicObjects around the robot
 *   - The trajectories being considered by MPEPC
 */
class NavigationInterfaceDisplay : public GridBasedDisplayWidget
{
public:
    /**
     * Constructor for NavigationInterfaceDisplay.
     */
    NavigationInterfaceDisplay(wxWindow* parent,
                               wxWindowID id = wxID_ANY,
                               const wxPoint& pos = wxDefaultPosition,
                               const wxSize& size = wxDefaultSize,
                               long style = 0,
                               const wxString& name = wxString((const wxChar*)("GLCanvas")),
                               const wxPalette& palette = wxNullPalette);

    virtual ~NavigationInterfaceDisplay(void);

    /**
     * setMode sets the overall display mode, which affects camera behavior as well as the displayed information.
     */
    void setMode(NavigationInterfaceMode mode);

    void setPose(const pose_t& pose);
    void setLPM(const hssh::LocalPerceptualMap& lpm);
    void setAreas(const hssh::LocalTopoMap& map) { topoMap_ = map; }
    void setGateways(const std::vector<hssh::Gateway>& gateways) { gateways_ = gateways; }
    void setDecisions(const std::vector<planner::Decision>& actions);

    void setHoverAction(int index) { hoverAction_ = index; }
    void setSelectedAction(int index) { selectedAction_ = index; }

    void setHoverArea(hssh::LocalArea::Id id) { hoverArea_ = id; }
    void setSelectedArea(hssh::LocalArea::Id id) { selectedArea_ = id; }

    void setGoalArea(hssh::LocalArea::Id id) { goalId_ = id; }

    void setTrajectories(const mpepc::trajectory_planner_debug_info_t& trajectories);
    void setObjects(const tracker::DynamicObjectCollection& objects) { objects_ = objects; }

    void toggleTrajectories(void) { shouldShowTrajectories_ = !shouldShowTrajectories_; }
    void toggleObjects(void) { shouldShowObjects_ = !shouldShowObjects_; }
    void showAreas(bool show) { shouldShowAreas_ = show; }

private:
    NavigationInterfaceMode mode_;

    pose_t pose_;
    hssh::LocalPerceptualMap map_;
    hssh::LocalTopoMap topoMap_;
    hssh::LocalArea::Id goalId_;
    std::vector<hssh::Gateway> gateways_;
    std::vector<planner::Decision> actions_;
    std::vector<mpepc::robot_trajectory_debug_info_t> trajectories_;
    tracker::DynamicObjectCollection objects_;

    std::size_t hoverAction_;
    std::size_t selectedAction_;

    hssh::LocalArea::Id hoverArea_;
    hssh::LocalArea::Id selectedArea_;

    bool haveNewMap_;

    bool shouldShowTrajectories_;
    bool shouldShowObjects_;
    bool shouldShowAreas_;

    std::unique_ptr<RobotRenderer> robotRenderer_;
    std::unique_ptr<OccupancyGridRenderer> mapRenderer_;
    std::unique_ptr<RobotTrajectoryRenderer> trajectoryRenderer_;
    std::unique_ptr<DynamicObjectRenderer> objectRenderer_;
    std::unique_ptr<GatewaysRenderer> gatewayRenderer_;
    std::unique_ptr<LocalAreaRenderer> areaRenderer_;

    LinearColorInterpolator interpolator_;

    // OpenGLWidget interface
    void renderWidget(void) override;
    void drawActions(const std::vector<planner::Decision>& actions);
    void drawDecision(const planner::Decision& decision, float alpha);

    // GridBasedDisplayWidget interface
    Point<int> convertWorldToGrid(const Point<float>& world) const override;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_NAVIGATION_NAVIGATION_INTERFACE_DISPLAY_H

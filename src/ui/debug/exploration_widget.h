/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     exploration_widget.h
* \author   Collin Johnson
*
* Declaration of ExplorationDisplayWidget.
*/

#ifndef UI_DEBUG_EXPLORATION_WIDGET_H
#define UI_DEBUG_EXPLORATION_WIDGET_H

#include "ui/components/grid_based_display_widget.h"
#include "hssh/local_metric/lpm.h"
#include "mpepc/trajectory/robot_trajectory_info.h"
#include "planner/exploration/local_topo/exploration_map.h"

namespace vulcan
{
namespace mpepc { class NavigationTask; }
namespace mpepc { struct dynamic_object_trajectory_debug_info_t; }
namespace ui
{

class RobotRenderer;
class OccupancyGridRenderer;
class PoseTargetRenderer;
class ExplorationMapRenderer;
class RobotTrajectoryRenderer;
class TrackedObjectRenderer;

/**
* ExplorationDisplayWidget draws the current exploration information for the map. The exploration information is:
*
*   - the LPM
*   - the LocalTopoExplorationMap (showing the current target, current area, visited, and unvisited areas)
*   - the current planner task being executed by the robot
*   - trajectory being followed by the robot
*   - dynamic objects + their estimated trajectories
*/
class ExplorationDisplayWidget : public GridBasedDisplayWidget
{
public:

    /**
    * Constructor for ExplorationDisplayWidget.
    */
    ExplorationDisplayWidget(wxWindow* parent,
                      wxWindowID id = wxID_ANY,
                      const wxPoint& pos = wxDefaultPosition,
                      const wxSize& size = wxDefaultSize,
                      long int style = 0,
                      const wxString& name = wxString((const wxChar*)("GLCanvas")),
                      const wxPalette& palette = wxNullPalette);

    /**
    * Destructor for ExplorationDisplayWidget.
    */
    virtual ~ExplorationDisplayWidget(void);

    void shouldCenterOnRobot(bool center) { centerOnRobot_ = center; }

    void setPose(const pose_t& pose);
    void setLPM(const hssh::LocalPerceptualMap& lpm);
    void setExplorationMap(const planner::LocalTopoExplorationMap& map);
    void setCurrentArea(hssh::LocalArea::Id currentId);
    void setTargetArea(hssh::LocalArea::Id targetId);
    void setNavigationTask(std::shared_ptr<mpepc::NavigationTask> task);
    void setOptimalTrajectory(const mpepc::robot_trajectory_debug_info_t& trajectory);
    void setDynamicObjects(const std::vector<mpepc::dynamic_object_trajectory_debug_info_t>& objects);

private:

    pose_t pose_;
    hssh::LocalPerceptualMap lpm_;
    planner::LocalTopoExplorationMap explorationMap_;
    hssh::LocalArea::Id currentId_;
    hssh::LocalArea::Id targetId_;
    std::shared_ptr<mpepc::NavigationTask> plannerTask_;
    mpepc::robot_trajectory_debug_info_t optimalTrajectory_;
    std::vector<mpepc::dynamic_object_trajectory_debug_info_t> dynamicObjects_;

    bool centerOnRobot_;
    bool haveNewLpm_;

    std::unique_ptr<RobotRenderer> robotRenderer_;
    std::unique_ptr<PoseTargetRenderer> targetRenderer_;
    std::unique_ptr<OccupancyGridRenderer> lpmRenderer_;
    std::unique_ptr<ExplorationMapRenderer> explorationRenderer_;
    std::unique_ptr<RobotTrajectoryRenderer> robotTrajectoryRenderer_;
    std::unique_ptr<TrackedObjectRenderer> objectRenderer_;

    // GridBasedDisplayWidget interface
    Point<int> convertWorldToGrid(const Point<float>& world) const override;
    void renderWidget(void) override;
};

}
}

#endif // UI_DEBUG_EXPLORATION_WIDGET_H

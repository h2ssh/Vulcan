/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     simulator_display.h
 * \author   Zongtai Luo and Collin Johnson
 *
 * Declaration of SimulatorDisplay.
 */

#ifndef UI_SIMULATOR_SIMULATOR_DISPLAY_H
#define UI_SIMULATOR_SIMULATOR_DISPLAY_H

#include "mpepc/trajectory/trajectory_planner_info.h"
#include "simulator/robot_group.h"
#include "ui/common/color_interpolator.h"
#include "ui/components/grid_based_display_widget.h"
#include "ui/simulator/simulator_robot_group_receiver.h"
#include "utils/locked_double_buffer.h"
#include "utils/mutex.h"

namespace vulcan
{
namespace ui
{

class RobotRenderer;
class OccupancyGridRenderer;
class RobotTrajectoryRenderer;
class PoseTargetRenderer;


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


class SimulatorDisplay : public GridBasedDisplayWidget
{
public:
    /**
     * Constructor for SimulatorDisplay.
     */
    SimulatorDisplay(wxWindow* parent,
                     wxWindowID id = wxID_ANY,
                     const wxPoint& pos = wxDefaultPosition,
                     const wxSize& size = wxDefaultSize,
                     long style = 0,
                     const wxString& name = wxString((const wxChar*)("GLCanvas")),
                     const wxPalette& palette = wxNullPalette);

    virtual ~SimulatorDisplay(void);

    // print information to cell status bar
    std::string printCellInformation(Point<int> cell) override;

    /**
     * setMode sets the overall display mode, which affects camera behavior as well as the displayed information.
     */
    void setMode(NavigationInterfaceMode mode);

    void setRobotReveiver(RobotGroupReceiver* robot_receivers) { robot_receivers_ = robot_receivers; }

    void setPose(const pose_t& pose);
    void setHoverDestinationPose(const pose_t& target);
    void setDestinationPose(const pose_t& target);
    void setDestinationPoses(const std::vector<pose_t>& waypointPoses);
    void clearDestinationPose(void);
    void clearHoverDestinationPose(void) { haveHoverDestinationPose_ = false; };
    void setHaveHoverDestinationPose(bool flag_hover) { haveHoverDestinationPose_ = flag_hover; }
    void setLPM(const hssh::LocalPerceptualMap& lpm);

    void setTrajectories(const mpepc::trajectory_planner_debug_info_t& trajectories)
    {
        trajectories_ = trajectories.trajectories;
    }

    void toggleTrajectories(void) { shouldShowTrajectories_ = !shouldShowTrajectories_; }
    void toggleObjects(void) { shouldShowObjects_ = !shouldShowObjects_; }

private:
    NavigationInterfaceMode mode_;

    // data for main robot
    pose_t pose_;
    hssh::LocalPerceptualMap map_;
    std::vector<mpepc::robot_trajectory_debug_info_t> trajectories_;

    // Mutex for handling data
    utils::Mutex dataLock_;

    // Robot receiver
    RobotGroupReceiver* robot_receivers_;

    bool haveNewMap_;

    bool shouldShowTrajectories_;
    bool shouldShowObjects_;

    // flags and vars for destination selection
    bool haveHoverDestinationPose_;
    bool haveSelectedDestinationPose_;
    bool showDestination_;
    pose_t hoverDestinationPose_;
    std::vector<pose_t> destinationPoses_;

    std::unique_ptr<RobotRenderer> robotRenderer_;
    std::unique_ptr<OccupancyGridRenderer> mapRenderer_;
    std::unique_ptr<RobotTrajectoryRenderer> trajectoryRenderer_;
    std::unique_ptr<PoseTargetRenderer> targetRenderer_;

    LinearColorInterpolator interpolator_;

    // OpenGLWidget interface
    void renderWidget(void) override;

    // GridBasedDisplayWidget interface
    Point<int> convertWorldToGrid(const Point<float>& world) const override;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_SIMULATOR_SIMULATOR_DISPLAY_H

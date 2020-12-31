/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     simulator_robot_display.h
* \author   Zongtai Luo and Collin Johnson
* 
* Declaration of SimulatorRobotDisplay.
*/

#ifndef UI_SIMULATOR_SIMULATOR_ROBOT_DISPLAY_H
#define UI_SIMULATOR_SIMULATOR_ROBOT_DISPLAY_H

#include "utils/mutex.h"
#include "ui/components/grid_based_display_widget.h"
#include "ui/common/color_interpolator.h"
#include "hssh/local_metric/lpm.h"
#include "hssh/local_metric/pose.h"
#include "hssh/local_topological/local_topo_map.h"
#include "mpepc/trajectory/trajectory_planner_info.h"
#include "planner/interface/decision.h"
#include "tracker/dynamic_object_collection.h"
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
class PoseTargetRenderer;

enum class SimulatorRobotMode
{
    drive,
    select,
    preview,
};

/**
* SimulatorRobotDisplay displays information needed to describe the current decision state and to allow the user to
* see what options are available to them.
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
class SimulatorRobotDisplay : public GridBasedDisplayWidget
{
public:
    
    /**
    * Constructor for SimulatorRobotDisplay.
    */
    SimulatorRobotDisplay(wxWindow* parent,
                             wxWindowID id = wxID_ANY,
                             const wxPoint& pos = wxDefaultPosition,
                             const wxSize& size = wxDefaultSize,
                             long style = 0,
                             const wxString& name = wxString((const wxChar*)("GLCanvas")),
                             const wxPalette& palette = wxNullPalette);

    virtual ~SimulatorRobotDisplay(void);

    // print information on the grid status bar
    std::string printCellInformation(Point<int> cell) override;

    /**
    * setMode sets the overall display mode, which affects camera behavior as well as the displayed information.
    */
    void setMode(SimulatorRobotMode mode);
    
    // setters for loading the new data
    void setPose(const pose_t& pose);
    void setLPM(const hssh::LocalPerceptualMap& lpm);
    void setAreas(const hssh::LocalTopoMap& map) { topoMap_ = map; }
    void setGateways(const std::vector<hssh::Gateway>& gateways) { gateways_ = gateways; }
    void setTrajectories(const mpepc::trajectory_planner_debug_info_t& trajectories) { trajectories_ = trajectories.trajectories; }
    void setObjects(const tracker::DynamicObjectCollection& objects) { objects_ = objects; }

    // setters for destination selection
    void setDestinationPose(const pose_t& target);
    void setDestinationPoses(const std::vector<pose_t>& waypointPoses);
    void setHoverDestinationPose(const pose_t& target);
    void setHaveHoverDestinationPose(bool flag_hover) {haveHoverDestinationPose_ = flag_hover;}
    void clearDestinationPose(void);
    void clearHoverDestinationPose(void) { haveHoverDestinationPose_ = false; };
    
    // setters for settings flags
    void toggleTrajectories(void) { shouldShowTrajectories_ = !shouldShowTrajectories_; }
    void toggleObjects(void) { shouldShowObjects_ = !shouldShowObjects_; }

private:
    
    SimulatorRobotMode mode_;

    // data coming in
    pose_t pose_;
    hssh::LocalPerceptualMap map_;
    hssh::LocalTopoMap topoMap_;
    hssh::LocalArea::Id goalId_;
    std::vector<hssh::Gateway> gateways_;
    std::vector<mpepc::robot_trajectory_debug_info_t> trajectories_;
    tracker::DynamicObjectCollection objects_;
    
    // Mutex for handling data
    utils::Mutex dataLock_;

    // flag for new map
    bool haveNewMap_;

    // flags for settings
    bool shouldShowTrajectories_;
    bool shouldShowObjects_;
    // bool shouldShowAreas_;

    // flags and vars for destination selection
    bool haveHoverDestinationPose_;
    bool haveSelectedDestinationPose_;
    bool showDestination_;
    pose_t hoverDestinationPose_;
    std::vector<pose_t> destinationPoses_;
    
    // renderer
    std::unique_ptr<RobotRenderer>              robotRenderer_;
    std::unique_ptr<OccupancyGridRenderer>      mapRenderer_;
    std::unique_ptr<RobotTrajectoryRenderer>    trajectoryRenderer_;
    std::unique_ptr<DynamicObjectRenderer>      objectRenderer_;
    std::unique_ptr<GatewaysRenderer>           gatewayRenderer_;
    std::unique_ptr<LocalAreaRenderer>          areaRenderer_;
    std::unique_ptr<PoseTargetRenderer>         targetRenderer_;

    LinearColorInterpolator interpolator_;

    // OpenGLWidget interface
    void renderWidget(void) override;

    // GridBasedDisplayWidget interface
    Point<int> convertWorldToGrid(const Point<float>& world) const override;

};

} // namespace ui
} // namespace vulcan

#endif // UI_SIMULATOR_SIMULATOR_ROBOT_DISPLAY_H

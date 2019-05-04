/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation_interface_display.cpp
* \author   Collin Johnson and Zongtai Luo
* 
* Definition of SimulatorRobotDisplay.
*/

#include <utils/mutex.h>
#include <ui/simulator/simulator_robot_display.h>
#include <ui/common/default_colors.h>
#include <ui/common/gl_shapes.h>
#include <ui/components/gateways_renderer.h>
#include <ui/components/robot_renderer.h>
#include <ui/components/robot_trajectory_renderer.h>
#include <ui/components/occupancy_grid_renderer.h>
#include <ui/components/dynamic_object_renderer.h>
#include <ui/components/local_area_renderer.h>
#include <ui/components/pose_target_renderer.h>
#include <ui/components/grid_based_display_widget.h>

namespace vulcan
{
namespace ui
{
    
const GLColor& action_color(hssh::AreaType type);


SimulatorRobotDisplay::SimulatorRobotDisplay(wxWindow* parent,
                                                   wxWindowID id,
                                                   const wxPoint& pos,
                                                   const wxSize& size,
                                                   long style,
                                                   const wxString& name,
                                                   const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, mode_(SimulatorRobotMode::select)
, haveNewMap_(false)
, shouldShowTrajectories_(true)
, shouldShowObjects_(true)
, haveHoverDestinationPose_(false)
, haveSelectedDestinationPose_(false)
, showDestination_(true)
, robotRenderer_(new RobotRenderer)
, mapRenderer_(new OccupancyGridRenderer)
, trajectoryRenderer_(new RobotTrajectoryRenderer)
, objectRenderer_(new DynamicObjectRenderer)
, gatewayRenderer_(new GatewaysRenderer)
, areaRenderer_(new LocalAreaRenderer)
, targetRenderer_(new PoseTargetRenderer)
{
    GLColor gatewayColor(17, 135, 255, 200);
    gatewayRenderer_->setRenderColors(gatewayColor, gatewayColor, gatewayColor);

    // Copy the colors used from debug_ui.cfg
    std::vector<GLColor> trajectoryColors = { GLColor(37, 212, 18, 125),
                                              GLColor(5, 5, 184, 125),
                                              GLColor(229, 1, 1, 125),};
    interpolator_.setColors(trajectoryColors);
}


SimulatorRobotDisplay::~SimulatorRobotDisplay(void)
{
    // For std::unique_ptr
}


std::string SimulatorRobotDisplay::printCellInformation(Point<int> cell)
{
    float cost = 0.0;
    cost = static_cast<float>(map_.getCost(cell));

    std::string description = "  Cost: "; 
    std::string costString  = std::to_string(cost);
    
    return description.append(costString);
}


void SimulatorRobotDisplay::setMode(SimulatorRobotMode mode)
{
    if(mode == SimulatorRobotMode::select)
    {
        // Center the camera on the map
        setCameraFocalPoint(map_.getGlobalCenter());

        // Zoom it out to include the entire map
        setViewRegion(map_.getWidthInMeters(), map_.getHeightInMeters());

        // Remove any rotation due to drive mode
        auto relativePos = getCameraRelativePosition();
        setCameraPosition(math::SphericalPoint(relativePos.rho, relativePos.phi, 0));
    }

    if(mode != SimulatorRobotMode::drive)
    {
        enablePanning();
        enableScrolling();
    }
    else
    {
        disablePanning();
        disableScrolling();
    }

    mode_ = mode;
}


void SimulatorRobotDisplay::setPose(const pose_t& pose)
{
    pose_ = pose;
    
    if(mode_ == SimulatorRobotMode::drive)
    {
        // Lock the position of the robot in the center of the map with the robot pointing toward the top of the screen
        setCameraFocalPoint(pose.toPoint());
        setViewRegion(15.0f, 15.0f);

        auto relativePos = getCameraRelativePosition();
        setCameraPosition(math::SphericalPoint(relativePos.rho, relativePos.phi, -pose.theta + M_PI_2));
    }
}


void SimulatorRobotDisplay::setLPM(const hssh::LocalPerceptualMap& lpm)
{ 
        map_ = lpm;
        haveNewMap_ = true;
}


void SimulatorRobotDisplay::setHoverDestinationPose(const pose_t& target)
{
    haveSelectedDestinationPose_ = true;
    hoverDestinationPose_     = target;
}


void SimulatorRobotDisplay::setDestinationPose(const pose_t& target)
{
    utils::AutoMutex autoLock(dataLock_);
    haveSelectedDestinationPose_ = true;
    showDestination_ = true; // this only changes the internal state, not the checkbox in the gui.
    
    destinationPoses_.clear();
    destinationPoses_.push_back(target);
}


void SimulatorRobotDisplay::setDestinationPoses(const std::vector<pose_t>& waypointPoses)
{
    utils::AutoMutex autoLock(dataLock_);
    haveSelectedDestinationPose_ = true;
    
    destinationPoses_ = waypointPoses;
}


void SimulatorRobotDisplay::renderWidget(void)
{
    if(haveNewMap_)
    {
        mapRenderer_->setGrid(map_);
        haveNewMap_ = false;
    }
    
    mapRenderer_->renderGrid();
    gatewayRenderer_->renderGateways(gateways_);
    robotRenderer_->renderRobot(pose_);
    
    if(shouldShowObjects_)
    {
        objectRenderer_->renderCollectionStateEstimates(objects_);
    }

    // rendering destination poses
    if(showDestination_ && haveSelectedDestinationPose_)
    {
        for(auto targetIt = destinationPoses_.begin(), targetEnd = destinationPoses_.end(); targetIt != targetEnd; targetIt++)
        {
            targetRenderer_->renderTargetRectangle(*targetIt);
        }
    }
    
    if(haveHoverDestinationPose_) // draw hover pose on top
    {
        targetRenderer_->renderTargetRectangle(hoverDestinationPose_);
    }
    
    if(shouldShowTrajectories_)
    {
        // Use a fixed scale for displaying the trajectories
        for(auto& traj : trajectories_)
        {
            const float kMinTrajectoryCost = -3.0f;
            const float kMaxTrajectoryCost =  0.5f;
            trajectoryRenderer_->renderTrajectory(traj.poses,
                                                  interpolator_.calculateColor((traj.expectedCost - kMinTrajectoryCost)
                                                    / (kMaxTrajectoryCost - kMinTrajectoryCost)));
        }
    }
}


Point<int> SimulatorRobotDisplay::convertWorldToGrid(const Point<float>& world) const
{
    return utils::global_point_to_grid_cell(world, map_);
}


void SimulatorRobotDisplay::clearDestinationPose(void)
{
    haveHoverDestinationPose_    = false;
    haveSelectedDestinationPose_ = false;
}
    
} // namespace ui
} // namespace vulcan

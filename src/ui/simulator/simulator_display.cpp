/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     simulator_display.cpp
* \author   Zongtai Luo
*
* Definition of SimulatorDisplay.
*/

#include "ui/simulator/simulator_display.h"
#include "ui/common/default_colors.h"
#include "ui/common/gl_shapes.h"
#include "ui/components/robot_renderer.h"
#include "ui/components/robot_trajectory_renderer.h"
#include "ui/components/occupancy_grid_renderer.h"
#include "ui/components/pose_target_renderer.h"

namespace vulcan
{
namespace ui
{

SimulatorDisplay::SimulatorDisplay(wxWindow* parent,
                                   wxWindowID id,
                                   const wxPoint& pos,
                                   const wxSize& size,
                                   long style,
                                   const wxString& name,
                                   const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, mode_(NavigationInterfaceMode::select)
, haveNewMap_(false)
, shouldShowTrajectories_(false)
, shouldShowObjects_(false)
, haveHoverDestinationPose_(false)
, haveSelectedDestinationPose_(false)
, showDestination_(true)
, robotRenderer_(new RobotRenderer)
, mapRenderer_(new OccupancyGridRenderer)
, trajectoryRenderer_(new RobotTrajectoryRenderer)
, targetRenderer_(new PoseTargetRenderer)
{
    // Copy the colors used from debug_ui.cfg
    std::vector<GLColor> trajectoryColors = { GLColor(37, 212, 18, 125),
                                              GLColor(5, 5, 184, 125),
                                              GLColor(229, 1, 1, 125),};
    interpolator_.setColors(trajectoryColors);
}


SimulatorDisplay::~SimulatorDisplay(void)
{
}


std::string SimulatorDisplay::printCellInformation(Point<int> cell)
{
    float cost = 0.0;
    cost = static_cast<float>(map_.getCost(cell));

    std::string description = "  Cost: "; 
    std::string costString  = std::to_string(cost);
    
    return description.append(costString);
}


void SimulatorDisplay::setMode(NavigationInterfaceMode mode)
{
    if(mode == NavigationInterfaceMode::select)
    {
        // Center the camera on the map
        setCameraFocalPoint(map_.getGlobalCenter());

        // Zoom it out to include the entire map
        setViewRegion(map_.getWidthInMeters(), map_.getHeightInMeters());

        // Remove any rotation due to drive mode
        auto relativePos = getCameraRelativePosition();
        setCameraPosition(math::SphericalPoint(relativePos.rho, relativePos.phi, 0));
    }

    if(mode != NavigationInterfaceMode::drive)
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


void SimulatorDisplay::setPose(const pose_t& pose)
{
    pose_ = pose;
    
    if(mode_ == NavigationInterfaceMode::drive)
    {
        // Lock the position of the robot in the center of the map with the robot pointing toward the top of the screen
        setCameraFocalPoint(pose.toPoint());
        setViewRegion(15.0f, 15.0f);

        auto relativePos = getCameraRelativePosition();
        setCameraPosition(math::SphericalPoint(relativePos.rho, relativePos.phi, -pose.theta + M_PI_2));
    }
}


void SimulatorDisplay::setHoverDestinationPose(const pose_t& target)
{
    haveSelectedDestinationPose_ = true;
    hoverDestinationPose_     = target;
}


void SimulatorDisplay::setDestinationPose(const pose_t& target)
{
    utils::AutoMutex autoLock(dataLock_);
    haveSelectedDestinationPose_ = true;
    showDestination_ = true; // this only changes the internal state, not the checkbox in the gui.
    
    destinationPoses_.clear();
    destinationPoses_.push_back(target);
}


void SimulatorDisplay::setDestinationPoses(const std::vector<pose_t>& waypointPoses)
{
    utils::AutoMutex autoLock(dataLock_);
    haveSelectedDestinationPose_ = true;
    
    destinationPoses_ = waypointPoses;
}


void SimulatorDisplay::clearDestinationPose(void)
{
    haveHoverDestinationPose_    = false;
    haveSelectedDestinationPose_ = false;
}


void SimulatorDisplay::setLPM(const hssh::LocalPerceptualMap& lpm)
{ 
    map_ = lpm;
    haveNewMap_ = true;
}


void SimulatorDisplay::renderWidget(void)
{
    if(haveNewMap_)
    {
        mapRenderer_->setGrid(map_);
        haveNewMap_ = false;
    }
    
    mapRenderer_->renderGrid();
    robotRenderer_->renderRobot(pose_);
    robot_receivers_->RenderRobots(robotRenderer_);

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
    
    if(false)
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


Point<int> SimulatorDisplay::convertWorldToGrid(const Point<float>& world) const
{
    return Point<int>(0, 0);
}

} // namespace ui
} // namespace vulcan

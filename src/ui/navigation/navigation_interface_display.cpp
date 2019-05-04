/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation_interface_display.cpp
* \author   Collin Johnson
*
* Definition of NavigationInterfaceDisplay.
*/

#include <ui/navigation/navigation_interface_display.h>
#include <ui/common/default_colors.h>
#include <ui/common/gl_shapes.h>
#include <ui/components/gateways_renderer.h>
#include <ui/components/robot_renderer.h>
#include <ui/components/robot_trajectory_renderer.h>
#include <ui/components/occupancy_grid_renderer.h>
#include <ui/components/dynamic_object_renderer.h>
#include <ui/components/local_area_renderer.h>

namespace vulcan
{
namespace ui
{

const GLColor& action_color(hssh::AreaType type);


NavigationInterfaceDisplay::NavigationInterfaceDisplay(wxWindow* parent,
                                                   wxWindowID id,
                                                   const wxPoint& pos,
                                                   const wxSize& size,
                                                   long style,
                                                   const wxString& name,
                                                   const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, mode_(NavigationInterfaceMode::drive)
, hoverAction_(0)
, selectedAction_(0)
, haveNewMap_(false)
, shouldShowTrajectories_(false)
, shouldShowObjects_(false)
, shouldShowAreas_(false)
, robotRenderer_(new RobotRenderer)
, mapRenderer_(new OccupancyGridRenderer)
, trajectoryRenderer_(new RobotTrajectoryRenderer)
, objectRenderer_(new DynamicObjectRenderer)
, gatewayRenderer_(new GatewaysRenderer)
, areaRenderer_(new LocalAreaRenderer)
{
    GLColor gatewayColor(17, 135, 255, 200);
    gatewayRenderer_->setRenderColors(gatewayColor, gatewayColor, gatewayColor);

    // Copy the colors used from debug_ui.cfg
    std::vector<GLColor> trajectoryColors = { GLColor(37, 212, 18, 125),
                                              GLColor(5, 5, 184, 125),
                                              GLColor(229, 1, 1, 125),};
    interpolator_.setColors(trajectoryColors);
}


NavigationInterfaceDisplay::~NavigationInterfaceDisplay(void)
{
    // For std::unique_ptr
}


void NavigationInterfaceDisplay::setMode(NavigationInterfaceMode mode)
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


void NavigationInterfaceDisplay::setPose(const pose_t& pose)
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


void NavigationInterfaceDisplay::setLPM(const hssh::LocalPerceptualMap& lpm)
{
    if(lpm.getId() != map_.getId())
    {
        map_ = lpm;
        haveNewMap_ = true;
    }
}


void NavigationInterfaceDisplay::setTrajectories(const mpepc::trajectory_planner_debug_info_t& trajectories)
{
    trajectories_.clear();
    trajectories_.insert(trajectories_.end(),
                         trajectories.trajectories.begin(),
                         trajectories.trajectories.begin() + trajectories.numTrajectories);
}


void NavigationInterfaceDisplay::setDecisions(const std::vector<planner::Decision>& actions)
{
    actions_ = actions;
    hoverAction_ = actions.size();
    selectedAction_ = actions.size();
}


void NavigationInterfaceDisplay::renderWidget(void)
{
    if(haveNewMap_)
    {
        mapRenderer_->setGrid(map_);
        haveNewMap_ = false;
    }

    mapRenderer_->renderGrid();
    gatewayRenderer_->renderGateways(gateways_);
    robotRenderer_->renderRobot(pose_);
    drawActions(actions_);

    if(shouldShowAreas_)
    {
        areaRenderer_->renderLocalTopoMap(topoMap_);

        if(auto hoverArea = topoMap_.areaWithId(hoverArea_))
        {
            areaRenderer_->renderLocalArea(*hoverArea);
        }

        if(auto selectedArea = topoMap_.areaWithId(selectedArea_))
        {
            areaRenderer_->renderLocalArea(*selectedArea);
        }
    }

    if(auto goalArea = topoMap_.areaWithId(goalId_))
    {
        areaRenderer_->renderLocalArea(*goalArea);
    }

    if(shouldShowObjects_)
    {
        objectRenderer_->renderCollectionStateEstimates(objects_);
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


void NavigationInterfaceDisplay::drawActions(const std::vector<planner::Decision>& actions)
{
    // All actions have an arrow outline
    for(auto& a : actions)
    {
        drawDecision(a, 0.66f);
    }

    // If there is a hover action, it will have a lightly-filled center
    if((hoverAction_ < actions.size()) && (hoverAction_ != selectedAction_))
    {
        drawDecision(actions[hoverAction_], 0.8f);
    }

    // If there is a selected action, it will have a darkly-filled center
    if(selectedAction_ < actions.size())
    {
        drawDecision(actions[selectedAction_], 1.0f);
    }

}


void NavigationInterfaceDisplay::drawDecision(const planner::Decision& decision, float alpha)
{
    const float kArrowLength = 1.0f;

    action_color(decision.areaType()).set(alpha);

    if(decision.isAbsolute())
    {
        gl_draw_large_arrow_polygon_line(decision.position(), kArrowLength, decision.orientation(), 3.0f);
    }
    else // should be drawn relative to robot
    {
        auto relativePos = rotate(Point<double>(0.5, 0.0), decision.orientation());
        gl_draw_large_arrow_polygon_line(pose_.toPoint() + relativePos, kArrowLength, decision.orientation(), 3.0f);
    }
}


Point<int> NavigationInterfaceDisplay::convertWorldToGrid(const Point<float>& world) const
{
    return utils::global_point_to_grid_cell(world, map_);
}


const GLColor& action_color(hssh::AreaType type)
{
    switch(type)
    {
    case hssh::AreaType::path_segment:
        return path_color();

    case hssh::AreaType::destination:
        return destination_color();

    case hssh::AreaType::decision_point:
        return decision_point_color();

    default:
        return area_color();
    }
}

} // namespace ui
} // namespace vulcan

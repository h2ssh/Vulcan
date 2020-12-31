/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     tracker_display_widget.cpp
* \author   Collin Johnson
*
* Definition of TrackerDisplayWidget.
*/

#include "ui/debug/tracker_display_widget.h"
#include "ui/components/laser_object_renderer.h"
#include "ui/components/robot_renderer.h"
#include "ui/components/occupancy_grid_renderer.h"
#include "ui/components/dynamic_object_renderer.h"
#include "ui/components/object_intention_renderer.h"
#include "ui/common/default_colors.h"
#include "ui/common/ui_params.h"
#include "tracker/objects/unclassified.h"
#include "tracker/objects/person.h"
#include "tracker/objects/rigid.h"
#include "tracker/objects/sliding_object.h"
#include "tracker/objects/pivoting_object.h"
#include "utils/auto_mutex.h"

namespace vulcan
{
namespace ui
{

    uint32_t rigid_object_state_to_flags(RigidObjectState state);
uint32_t tracking_uncertainty_to_flags(TrackingUncertainty uncertainty);


TrackerDisplayWidget::TrackerDisplayWidget(wxWindow* parent,
                                           wxWindowID id,
                                           const wxPoint& pos,
                                           const wxSize& size,
                                           long style,
                                           const wxString& name,
                                           const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, haveNewLPM_(false)
, laserObjectsIndex_(0)
, lpmRenderer_(new OccupancyGridRenderer)
, dynamicObjectRenderer_(new DynamicObjectRenderer)
, laserObjectRenderer_(new LaserObjectRenderer)
, robotRenderer_(new RobotRenderer)
, intentionRenderer_(new ObjectIntentionRenderer(1.0))
, shouldFollowRobot_(true)
, shouldShowLaserObjects_(true)
, shouldShowLaserPoints_(true)
, shouldShowLaserUncertainty_(false)
, shouldShowDynamicObjects_(true)
, shouldShowAccleration_(false)
, shouldShowRecentTrajectory_(true)
, rigidObjectStateToShow_(RigidObjectState::fast)
, trackingUncertaintyToShow_(TrackingUncertainty::position)
, goalsToShow_(PredictionType::none)
, trajectoryPredictionToShow_(PredictionType::none)
, predictedTrajDurationMs_(5000)
{
    lpmRenderer_->setDynamicColor(GLColor(255, 255, 255, 255));
    lpmRenderer_->setLimitedVisibilityColor(occupied_color());
    lpmRenderer_->setQuasiStaticColor(occupied_color());
    lpmRenderer_->setHazardColor(occupied_color());
}


TrackerDisplayWidget::~TrackerDisplayWidget(void)
{
    // Nothing to do here
}

void TrackerDisplayWidget::setWidgetParams(const ui_params_t& params)
{
    robotRenderer_->setRobotColor(params.lpmParams.robotColor);
    lpmRenderer_->setDynamicColor(GLColor(255, 255, 255, 255));
}


void TrackerDisplayWidget::shouldEvaluateIntentions(bool evaluate)
{
    utils::AutoMutex autoLock(dataLock_);

    // If starting evaluation, then reset any existing evaluation to store the new state
    if(evaluate && !shouldEvaluateIntentions_)
    {
        intentions_.reset();
        std::cout << "Reset intentions.\n";
    }
    else if(!evaluate && intentions_)
    {
        intentions_->saveToFile("debug_ui_intentions.txt");
        std::cout << "Saved intentions to debug_ui_intentions.txt\n";
    }

    shouldEvaluateIntentions_ = evaluate;
}


void TrackerDisplayWidget::handleData(const motion_state_t& state, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    robotState_ = state;

    if(shouldFollowRobot_)
    {
        setCameraFocalPoint(state.pose.toPoint());
    }
}


void TrackerDisplayWidget::handleData(const hssh::LocalPerceptualMap& lpm, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    lpm_        = lpm;
    haveNewLPM_ = true;
}


void TrackerDisplayWidget::handleData(const tracker::LaserObjectCollection& objects, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    laserObjects_.resize(1);
    laserObjects_[0] = objects;
//     laserObjects_[objects.laserId()] = objects;
}


void TrackerDisplayWidget::handleData(const tracker::DynamicObjectCollection& objects, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    dynObjCollection_ = objects;

    if(shouldEvaluateIntentions_ && !objects.empty())
    {
        // Just assume the first object is the one being tracked
        tracker::DynamicObject::ConstPtr object = *objects.begin();
        if(!intentions_)
        {
            intentions_ = std::make_unique<tracker::AreaIntentionEstimates>(0, object->goals());
        }
        else
        {
            intentions_->addSample(*object);
        }
    }
}


Point<int> TrackerDisplayWidget::convertWorldToGrid(const Point<float>& world) const
{
    return utils::global_point_to_grid_cell(world, lpm_);
}


void TrackerDisplayWidget::renderWidget(void)
{
    utils::AutoMutex autoLock(dataLock_);

    if(haveNewLPM_)
    {
        lpmRenderer_->setGrid(lpm_);
        haveNewLPM_ = false;
    }

    lpmRenderer_->renderGrid();

    if(shouldShowLaserObjects_)
    {
        int options = createLaserOptions();
        for(auto& objs : laserObjects_)
        {
            laserObjectRenderer_->renderObjects(objs, boundaryToShow_, options);
        }
//         laserObjectRenderer_->renderObjects(laserObjects_[0], options);
//         laserObjectRenderer_->renderObjects(laserObjects_[1], options);
    }

    if(shouldShowDynamicObjects_)
    {
        uint32_t rigidFlags = shouldShowAccleration_ ? DynamicObjectRenderer::kShowAcceleration : 0;
        rigidFlags |= tracking_uncertainty_to_flags(trackingUncertaintyToShow_);
        rigidFlags |= rigid_object_state_to_flags(rigidObjectStateToShow_);
        dynamicObjectRenderer_->renderCollectionStateEstimates(dynObjCollection_, rigidFlags);
    }

    if(shouldShowDynamicObjects_ && (goalsToShow_ != PredictionType::none))
    {
        for(auto& obj : dynObjCollection_)
        {
            dynamicObjectRenderer_->renderObjectGoals(*obj, (goalsToShow_ == PredictionType::all));
        }
    }

    if(shouldEvaluateIntentions_ && intentions_)
    {
        intentionRenderer_->renderIntentions(*intentions_);
    }

    if(shouldFollowRobot_)
    {
        robotRenderer_->renderRobot(robotState_.pose);
    }
}


int TrackerDisplayWidget::createLaserOptions(void) const
{
    int options = 0;

    if(shouldShowLaserPoints_)
    {
        options |= LaserObjectRenderer::kShowPoints;
    }

    if(shouldShowLaserUncertainty_)
    {
        options |= LaserObjectRenderer::kShowUncertainty;
    }

    return options;
}


uint32_t rigid_object_state_to_flags(RigidObjectState state)
{
    switch(state)
    {
    case RigidObjectState::fast:
        return DynamicObjectRenderer::kShowFastState;

    case RigidObjectState::slow:
        return DynamicObjectRenderer::kShowSlowState;

    default:
        return 0;
    }
}


uint32_t tracking_uncertainty_to_flags(TrackingUncertainty uncertainty)
{
    switch(uncertainty)
    {
    case TrackingUncertainty::position:
        return DynamicObjectRenderer::kShowPositionUncertainty;

    case TrackingUncertainty::velocity:
        return DynamicObjectRenderer::kShowVelocityUncertainty;

    case TrackingUncertainty::acceleration:
        return DynamicObjectRenderer::kShowAccelerationUncertainty;

    case TrackingUncertainty::none:
        // intentional fall-through
    default:
        return 0;
    }
}

} // namespace ui
} // namespace vulcan

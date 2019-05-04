/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_metric_display_widget.cpp
* \author   Collin Johnson
* 
* Definition of GlobalMetricDisplayWidget.
*/

#include <ui/debug/global_metric_display_widget.h>
#include <ui/components/occupancy_grid_renderer.h>
#include <ui/components/robot_renderer.h>
#include <ui/components/pose_trace_renderer.h>
#include <ui/components/particles_renderer.h>
#include <hssh/global_metric/map.h>
#include <utils/timestamp.h>

namespace vulcan
{
namespace ui
{
    
using AutoLock = std::unique_lock<std::mutex>;
    

GlobalMetricDisplayWidget::GlobalMetricDisplayWidget(wxWindow*        parent,
                                                     wxWindowID       id,
                                                     const wxPoint&   pos,
                                                     const wxSize&    size,
                                                     long             style,
                                                     const wxString&  name,
                                                     const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, moduleState_      (ModuleState::Waiting)
, haveNewMap_       (false)
, mapRenderer_      (new OccupancyGridRenderer)
, robotRenderer_    (new RobotRenderer)
, traceRenderer_    (new PoseTraceRenderer)
, particlesRenderer_(new ParticlesRenderer)
{
    
}


void GlobalMetricDisplayWidget::setParams(const ui_params_t& params)
{
    
}


void GlobalMetricDisplayWidget::setMap(const std::shared_ptr<const hssh::GlobalMetricMap>& map)
{
    AutoLock lock(dataLock_);
    map_        = map;
    haveNewMap_ = true;
}


void GlobalMetricDisplayWidget::setPose(const hssh::GlobalPose& pose)
{
    AutoLock lock(dataLock_);
    pose_ = pose;
    
    trace_.addPose(pose.pose());
    traceRenderer_->setPoseTrace(trace_);
}


void GlobalMetricDisplayWidget::setLocalizationInfo(const hssh::particle_filter_debug_info_t& info)
{
    AutoLock lock(dataLock_);
    localizationInfo_ = info;
    moduleState_      = ModuleState::Localized;
}


void GlobalMetricDisplayWidget::setRelocalizationInfo(const hssh::metric_relocalization_debug_info_t& info)
{
    AutoLock lock(dataLock_);
    
    if(moduleState_ != ModuleState::Relocalizing)
    {
        relocalizationStartTime_ = utils::system_time_us();
    }
    
    relocalizationInfo_ = info;
    moduleState_        = ModuleState::Relocalizing;
}


void GlobalMetricDisplayWidget::renderWidget(void)
{
//     const int64_t kInitialParticlesDisplayDuration = 5000000;
    
    AutoLock lock(dataLock_);
    
    if(map_ && haveNewMap_)
    {
        mapRenderer_->setGrid(*map_);
    }
    
    mapRenderer_->renderGrid();
    traceRenderer_->renderTrace();
    robotRenderer_->renderRobot(pose_.pose());
    
    switch(moduleState_)
    {
    case ModuleState::Waiting:
        // Nothing to draw if waiting for something to happen
        break;
        
    case ModuleState::Localized:
        particlesRenderer_->renderParticles(localizationInfo_.particles);
        break;
        
    case ModuleState::Relocalizing:
//         if(utils::system_time_us() - relocalizationStartTime_ < kInitialParticlesDisplayDuration)
//         {
            particlesRenderer_->renderParticles(relocalizationInfo_.initialParticles);
//         }
        
        particlesRenderer_->renderParticles(relocalizationInfo_.particleFilterInfo.particles);
        break;
    }
}


Point<int> GlobalMetricDisplayWidget::convertWorldToGrid(const Point<float>& world) const
{
    if(map_)
    {
        return utils::global_point_to_grid_cell(world, *map_);
    }
    
    return Point<int>(-1, -1);
    
}
    
} // namespace ui
} // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     relocalization_display_widget.cpp
 * \author   Collin Johnson
 *
 * Definition of RelocalizationDisplayWidget.
 */

#include "ui/debug/relocalization_display_widget.h"
#include "ui/common/gl_shapes.h"
#include "ui/common/ui_params.h"
#include "ui/components/laser_scan_renderer.h"
#include "ui/components/occupancy_grid_renderer.h"
#include "ui/components/particles_renderer.h"
#include "ui/components/robot_renderer.h"
#include "utils/auto_mutex.h"

namespace vulcan
{
namespace ui
{

RelocalizationDisplayWidget::RelocalizationDisplayWidget(wxWindow* parent,
                                                         wxWindowID id,
                                                         const wxPoint& pos,
                                                         const wxSize& size,
                                                         long style,
                                                         const wxString& name,
                                                         const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, mapRenderer_(new OccupancyGridRenderer())
, particlesRenderer_(new ParticlesRenderer())
, robotRenderer_(new RobotRenderer())
, scanRenderer_(new LaserScanRenderer())
, initialRegionColor(0, 197, 246, 200)
, initialParticlesColor(245, 156, 0, 200)
{
}


void RelocalizationDisplayWidget::setWidgetParams(const ui_params_t& params)
{
}


void RelocalizationDisplayWidget::setMap(const std::shared_ptr<hssh::OccupancyGrid>& map)
{
    std::unique_lock<std::mutex> autoLock(dataLock_);
    map_ = map;
    haveNewMap_ = true;
}


void RelocalizationDisplayWidget::setRelocalizationInfo(const hssh::metric_relocalization_debug_info_t& info)
{
    std::unique_lock<std::mutex> autoLock(dataLock_);

    particleInfo_ = info.particleFilterInfo;
    meanPose_ = info.pose;
    initialParticles_ = info.initialParticles;
}


void RelocalizationDisplayWidget::setInitializerRegion(const math::Rectangle<float>& rectangle)
{
    std::unique_lock<std::mutex> autoLock(dataLock_);

    regionInitializerRectangle_ = rectangle;
}


void RelocalizationDisplayWidget::renderWidget(void)
{
    std::unique_lock<std::mutex> lock(dataLock_);

    renderMap();

    if (shouldShowInitialRegion_) {
        renderInitialization();
    }
}


Point<int> RelocalizationDisplayWidget::convertWorldToGrid(const Point<float>& world) const
{
    return map_ ? utils::global_point_to_grid_cell(world, *map_) : Point<int>(0, 0);
}


void RelocalizationDisplayWidget::renderMap(void)
{
    if (haveNewMap_) {
        mapRenderer_->setGrid(*map_);
        haveNewMap_ = false;
    }

    mapRenderer_->renderGrid();

    if (shouldShowLaser_) {
        scanRenderer_->renderScan(particleInfo_.scan, meanPose_.toPose());
    }

    if (shouldShowError_) {
        gl_draw_gaussian_distribution(meanPose_.uncertainty, 3.0f, GLColor(0.0f, 0.9f, 0.25f, 0.75f), 2.0f);
    }

    if (shouldShowParticles_) {
        particlesRenderer_->renderParticles(particleInfo_.particles);
    }
}


void RelocalizationDisplayWidget::renderInitialization(void)
{
    initialRegionColor.set(0.33f);
    gl_draw_filled_rectangle(regionInitializerRectangle_);
    initialRegionColor.set();
    gl_draw_line_rectangle(regionInitializerRectangle_, 2.0f);

    particlesRenderer_->renderParticles(initialParticles_);
}

}   // namespace ui
}   // namespace vulcan

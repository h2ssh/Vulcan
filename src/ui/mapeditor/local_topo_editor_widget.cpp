/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     local_topo_editor_widget.cpp
 * \author   Collin Johnson
 *
 * Implementation of LocalTopoEditorWidget.
 */

#include "ui/mapeditor/local_topo_editor_widget.h"
#include "hssh/local_metric/lpm.h"
#include "ui/common/default_colors.h"
#include "ui/common/gl_shapes.h"
#include "ui/common/ui_params.h"
#include "ui/components/area_subgraph_renderer.h"
#include "ui/components/gateways_renderer.h"
#include "ui/components/place_grid_renderer.h"
#include "utils/auto_mutex.h"

namespace vulcan
{
namespace ui
{

LocalTopoEditorWidget::LocalTopoEditorWidget(wxWindow* parent,
                                             wxWindowID id,
                                             const wxPoint& pos,
                                             const wxSize& size,
                                             long style,
                                             const wxString& name,
                                             const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, isDirtySkeleton_(false)
, skeletonRenderer_(new VoronoiSkeletonGridRenderer)
, gatewayRenderer_(new GatewaysRenderer)
, areaRenderer_(new AreaHypothesisRenderer)
{
    skeletonRenderer_->showDistances(false);
    skeletonRenderer_->showFullSkeleton(false);
    skeletonRenderer_->showReducedSkeleton(true);
    skeletonRenderer_->showFrontiers(true);
}


LocalTopoEditorWidget::~LocalTopoEditorWidget(void)
{
    // For std::unique_ptr
}


void LocalTopoEditorWidget::setParams(const ui_params_t& params)
{
    gatewayRenderer_->setRenderColors(params.localTopoParams.gatewayColor,
                                      params.localTopoParams.frontierColor,
                                      GLColor());

    skeletonRenderer_->setRenderColors(params.localTopoParams.frontierColor,
                                       params.localTopoParams.skeletonCellColor,
                                       params.localTopoParams.reducedCellColor);
}


void LocalTopoEditorWidget::setSkeleton(hssh::VoronoiSkeletonGrid skeleton)
{
    utils::AutoMutex autoLock(dataLock_);
    skeleton_ = std::move(skeleton);
    isDirtySkeleton_ = true;
    setViewRegion(skeleton_.getWidthInMeters(), skeleton_.getHeightInMeters());
    setCameraFocalPoint(skeleton_.getGlobalCenter());
}


void LocalTopoEditorWidget::setAreas(const std::vector<hssh::AreaHypothesis*>& areas)
{
    utils::AutoMutex autoLock(dataLock_);
    areas_ = areas;
}


void LocalTopoEditorWidget::setGateways(const std::vector<hssh::Gateway>& gateways)
{
    utils::AutoMutex autoLock(dataLock_);
    gateways_ = gateways;
}


void LocalTopoEditorWidget::setHoverGateway(boost::optional<hssh::Gateway> gateway)
{
    utils::AutoMutex autoLock(dataLock_);
    hoverGateway_ = gateway;
}


void LocalTopoEditorWidget::setSelectedGateway(boost::optional<hssh::Gateway> gateway)
{
    utils::AutoMutex autoLock(dataLock_);
    selectedGateway_ = gateway;
}


Point<int> LocalTopoEditorWidget::convertWorldToGrid(const Point<float>& world) const
{
    return utils::global_point_to_grid_cell(world, skeleton_);
}


void LocalTopoEditorWidget::renderWidget(void)
{
    utils::AutoMutex autoLock(dataLock_);

    if (isDirtySkeleton_) {
        skeletonRenderer_->setGrid(skeleton_);
        isDirtySkeleton_ = false;
    }

    skeletonRenderer_->renderGrid();
    renderAreas();
    renderGateways();
}


void LocalTopoEditorWidget::renderAreas(void) const
{
    for (auto& area : areas_) {
        areaRenderer_->renderHypothesis(*area, skeleton_.metersPerCell());
    }
}


void LocalTopoEditorWidget::renderGateways(void) const
{
    gatewayRenderer_->renderGateways(gateways_);

    if (hoverGateway_) {
        GLColor hoverColor(quasi_static_color());
        hoverColor.alpha(0.33);
        gatewayRenderer_->renderGateway(*hoverGateway_, false, &hoverColor);
    }

    if (selectedGateway_) {
        GLColor selectedColor(hazard_color());
        gatewayRenderer_->renderGateway(*selectedGateway_, false, &selectedColor);
    }
}

}   // namespace ui
}   // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     evaluation_widget.cpp
* \author   Collin Johnson
*
* Definition of EvaluationDisplayWidget.
*/

#include <ui/debug/evaluation_display_widget.h>
#include <ui/common/default_colors.h>
#include <ui/components/place_grid_renderer.h>
#include <ui/components/pose_trace_renderer.h>
#include <ui/components/stable_area_renderer.h>
#include <mpepc/evaluation/path_summary.h>
#include <utils/cell_grid_utils.h>

namespace vulcan
{
namespace ui
{

EvaluationDisplayWidget::EvaluationDisplayWidget(wxWindow* parent,
                                                 wxWindowID id,
                                                 const wxPoint& pos,
                                                 const wxSize& size,
                                                 long int style,
                                                 const wxString& name,
                                                 const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, mapRenderer_(std::make_unique<VoronoiSkeletonGridRenderer>())
, areaRenderer_(std::make_unique<StableAreaRenderer>())
, traceRenderer_(std::make_unique<PoseTraceRenderer>())
, areaMask_(StableAreaRenderer::kDrawBoundary)
{
    // Only want to render the map itself
    mapRenderer_->showDistances(false);
    mapRenderer_->showFrontiers(false);
    mapRenderer_->showFullSkeleton(false);
    mapRenderer_->showReducedSkeleton(false);

    traceRenderer_->showVelocityTicks(false);
}


EvaluationDisplayWidget::~EvaluationDisplayWidget(void)
{
    // For std::unique_ptr
}


void EvaluationDisplayWidget::setMap(const std::shared_ptr<hssh::LocalTopoMap>& map)
{
    if(map)
    {
        map_ = map;
        haveNewMap_ = true;

        // When the map changes, auto-zoom
        setViewRegion(map_->voronoiSkeleton().getWidthInMeters(), map_->voronoiSkeleton().getHeightInMeters());
        setCameraFocalPoint(map_->voronoiSkeleton().getGlobalCenter());
    }
}


void EvaluationDisplayWidget::setAreas(const std::shared_ptr<hssh::AreaStabilityAnalyzer>& areas)
{
    areas_ = areas;
}


void EvaluationDisplayWidget::setPaths(const mpepc::PathSummary& paths)
{
    socialTraces_.clear();
    socialTraces_.insert(socialTraces_.end(), paths.beginSocial(), paths.endSocial());

    regularTraces_.clear();
    regularTraces_.insert(regularTraces_.end(), paths.beginRegular(), paths.endRegular());
}


void EvaluationDisplayWidget::shouldDrawBoundaries(bool draw)
{
    if(draw)
    {
        areaMask_ |= StableAreaRenderer::kDrawBoundary;
    }
    else
    {
        areaMask_ &= ~StableAreaRenderer::kDrawBoundary;
    }
}


void EvaluationDisplayWidget::shouldDrawStars(bool draw)
{
    if(draw)
    {
        areaMask_ |= StableAreaRenderer::kDrawStar;
    }
    else
    {
        areaMask_ &= ~StableAreaRenderer::kDrawStar;
    }
}


Point<int> EvaluationDisplayWidget::convertWorldToGrid(const Point<float>& world) const
{
    if(map_)
    {
        return utils::global_point_to_grid_cell(world, map_->voronoiSkeleton());
    }

    return Point<int>(0, 0);
}


void EvaluationDisplayWidget::renderWidget(void)
{
    if(haveNewMap_)
    {
        mapRenderer_->setGrid(map_->voronoiSkeleton());
        haveNewMap_ = false;
    }

    if(map_)
    {
        mapRenderer_->renderGrid();
    }

    if(areas_)
    {
        areaRenderer_->renderAll(*areas_, areaMask_);
    }

    if(!socialTraces_.empty())
    {
        for(auto& trace : socialTraces_)
        {
            auto color = social_mpepc_color();
            color.alpha(0.5);
            renderTrace(trace, color);
        }
    }

    if(!regularTraces_.empty())
    {
        for(auto& trace : regularTraces_)
        {
            auto color = regular_mpepc_color();
            color.alpha(0.5);
            renderTrace(trace, color);
        }
    }
}


void EvaluationDisplayWidget::renderTrace(const utils::PoseTrace& trace, const GLColor& color)
{
    traceRenderer_->setRenderColor(color);
    traceRenderer_->setPoseTrace(trace);
    traceRenderer_->renderTrace();
}

} // namespace ui
} // namespace vulcan

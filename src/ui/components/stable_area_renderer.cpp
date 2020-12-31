/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     stable_area_renderer.cpp
* \author   Collin Johnson
*
* Definition of StableAreaRenderer.
*/

#include "ui/components/stable_area_renderer.h"
#include "ui/components/small_scale_star_renderer.h"
#include "ui/common/ui_color.h"
#include "ui/common/gl_shapes.h"
#include "ui/common/hssh_colors.h"
#include "hssh/local_topological/evaluation/stability_analyzer.h"

namespace vulcan
{
namespace ui
{

StableAreaRenderer::StableAreaRenderer(void)
: starRenderer_(std::make_unique<SmallScaleStarRenderer>())
{
}


StableAreaRenderer::~StableAreaRenderer(void)
{
    // For std::unique_ptr
}


void StableAreaRenderer::render(const hssh::StableArea& area, float alpha, uint8_t mask)
{
    if(mask & kDrawBoundary)
    {
        auto color = color_from_local_area_type(area.type);
        color.set(alpha);
        gl_draw_filled_rectangle(area.boundary);
        occupied_color().set();
        gl_draw_line_rectangle(area.boundary, 2.0f);
    }

    if(mask & kDrawStar)
    {
        starRenderer_->render(area.star, area.boundary.center());
    }
}


void StableAreaRenderer::renderAll(const hssh::AreaStabilityAnalyzer& analyzer, uint8_t mask)
{
    // TODO: Do something smarter for rendering
    const float kAlpha = 0.25f;

    for(auto& area : analyzer)
    {
        render(area, kAlpha, mask);
    }
}

} // namespace ui
} // namespace vulcan

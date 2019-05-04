/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     small_scale_star_renderer.cpp
* \author   Collin Johnson
*
* Definition of SmallScaleStarRenderer.
*/

#include <ui/components/small_scale_star_renderer.h>
#include <ui/common/color_generator.h>
#include <ui/common/default_colors.h>
#include <ui/common/gl_shapes.h>
#include <hssh/local_topological/small_scale_star.h>

namespace vulcan
{
namespace ui
{

void SmallScaleStarRenderer::render(const hssh::SmallScaleStar& star, Point<float> origin)
{
    const float kArrowWidth = 2.0f;

    auto& colors = generateColors(star.getNumPaths());

    for(auto& frag : star)
    {
        if(frag.type != hssh::AreaType::destination)
        {
            colors[frag.pathId].set();
        }
        else
        {
            destination_color().set();
        }

        gl_draw_small_arrow(origin, 0.5, angle_to_point(origin, frag.gateway.center()), kArrowWidth);
    }
}


void SmallScaleStarRenderer::renderRelative(const hssh::SmallScaleStar& star, Point<float> origin)
{
    const float kArrowWidth = 2.0f;

    auto& colors = generateColors(star.getNumPaths());

    for(auto& frag : star)
    {
        if(frag.type != hssh::AreaType::destination)
        {
            colors[frag.pathId].set();
        }
        else
        {
            destination_color().set();
        }

        gl_draw_small_arrow(Point<float>(0.0, 0.0),
                            0.5,
                            angle_to_point(origin, frag.gateway.center()),
                            kArrowWidth);
    }
}


const SmallScaleStarRenderer::StarColors& SmallScaleStarRenderer::generateColors(int numPaths)
{
    // Clamp the paths to a reasonable range
    numPaths = std::min(std::max(0, numPaths), 30);

    if(numPaths >= static_cast<int>(colors_.size()))
    {
        for(int n = colors_.size(); n <= numPaths; ++n)
        {
            colors_.push_back(generate_colors(n, 0.8, false));
        }
    }

    return colors_[numPaths];
}

} // namespace ui
} // namespace vulcan

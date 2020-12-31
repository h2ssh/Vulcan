/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     small_scale_star_renderer.h
 * \author   Collin Johnson
 *
 * Declaration of SmallScaleStarRenderer.
 */

#ifndef UI_COMPONENTS_SMALL_SCALE_STAR_RENDERER_H
#define UI_COMPONENTS_SMALL_SCALE_STAR_RENDERER_H

#include "core/point.h"
#include "ui/common/ui_color.h"
#include <vector>

namespace vulcan
{
namespace hssh
{
class SmallScaleStar;
}
namespace ui
{

/**
 * SmallScaleStarRenderer renders a small-scale star using the following:
 *
 *   - All transitions leading to destinations are red.
 *   - Each pair of aligned transitions is a new color.
 *   - Colors for aligned transitions are spaced evenly through hue in HSV space -- except for red.
 */
class SmallScaleStarRenderer
{
public:
    /**
     * render renders the small-scale star. All rays emanate from the specified origin.
     */
    void render(const hssh::SmallScaleStar& star, Point<float> origin);

    /**
     * Render the relative position of the star, centered at (0, 0) of its global place. Still make angle computations
     * relative to the provided origin so the angles are all correct though.
     */
    void renderRelative(const hssh::SmallScaleStar& star, Point<float> origin);

private:
    using StarColors = std::vector<GLColor>;

    // cache all generated colors -- index is the number of paths in the star
    std::vector<StarColors> colors_;


    const StarColors& generateColors(int numPaths);
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_SMALL_SCALE_STAR_RENDERER_H

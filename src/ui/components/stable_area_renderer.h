/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     stable_area_renderer.h
 * \author   Collin Johnson
 *
 * Declaration of StableAreaRenderer.
 */

#ifndef UI_COMPONENTS_STABLE_AREA_RENDERER_H
#define UI_COMPONENTS_STABLE_AREA_RENDERER_H

#include <cstdint>
#include <memory>

namespace vulcan
{
namespace hssh
{
struct StableArea;
}
namespace hssh
{
class AreaStabilityAnalyzer;
}
namespace ui
{

/**
 * StableAreaRenderer draws stable areas, which are used for evaluation of the place labeling
 * algorithm.
 *
 * StableAreas are drawn using their rectangle boundary and their SmallScaleStar.
 *
 * The alpha is determined based on the total number of areas occupying a particular location.
 */
class StableAreaRenderer
{
public:
    // Flags that control exactly which parts of a stable area are drawn
    enum : uint8_t
    {
        kDrawBoundary = 0x01,
        kDrawStar = 0x02,
    };


    StableAreaRenderer(void);
    ~StableAreaRenderer(void);

    /**
     * render draws a single StableArea. The mask specifies which bits of the area are to
     * be drawn.
     *
     * \param    area            Area to be drawn
     * \param    alpha           How much alpha is assigned to this area
     * \param    mask            Mask with flags controlling which parts of the area to draw
     */
    void render(const hssh::StableArea& area, float alpha, uint8_t mask);

    /**
     * renderAll renders a collection of StableAreas. The alpha is automatically computed for
     * these areas based on the results from the analyzer.
     *
     * \param    analyzer        Analyzer with the state about the StableAreas
     * \param    mask            Mask with flags controlling which parts of an area to draw
     */
    void renderAll(const hssh::AreaStabilityAnalyzer& analyzer, uint8_t mask);

private:
    std::unique_ptr<class SmallScaleStarRenderer> starRenderer_;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_STABLE_AREA_RENDERER_H

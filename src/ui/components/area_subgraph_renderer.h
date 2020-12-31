/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     area_subgraph_renderer.h
 * \author   Collin Johnson
 *
 * Definition of AreaHypothesisRenderer.
 */

#ifndef UI_COMPONENTS_AREA_SUBGRAPH_RENDERER_H
#define UI_COMPONENTS_AREA_SUBGRAPH_RENDERER_H

#include "hssh/local_topological/area_detection/labeling/hypothesis_type.h"
#include "ui/common/color_interpolator.h"
#include "ui/common/ui_color.h"
#include "ui/components/area_extent_renderer.h"

namespace vulcan
{
namespace hssh
{
class AreaHypothesis;
}
namespace hssh
{
class DebugHypothesis;
}
namespace ui
{

/**
 * AreaHypothesisRenderer draws the AreaHypotheses used in the AreaParser to the screen. The drawn hypotheses show the
 * following information:
 *
 *   - Bounding box based on the contained nodes
 *   - Color based on the HypothesisType
 *   - Direction of the main axis
 *   - Magnitude of the major axis
 */
class AreaHypothesisRenderer
{
public:
    /**
     * Default constructor for AreaHypothesisRenderer.
     *
     * Sets the default colors to use for areas
     */
    AreaHypothesisRenderer(void);

    /**
     * setRenderColor sets the color to render the hypotheses based on their types. There is a separate color for each
     * possible combination of types.
     */
    void setRenderColors(const GLColor& unknownColor,
                         const GLColor& decisionPointColor,
                         const GLColor& destinationColor,
                         const GLColor& pathColor,
                         const std::vector<GLColor>& valueColors);

    /**
     * renderHypothesis draws the non-debug hypothesis. Only the boundary of the hypothesis is drawn.
     */
    void renderHypothesis(const hssh::AreaHypothesis& hypothesis, float metersPerCell) const;

    /**
     * renderHypothesis draws the hypothesis representation as described in the class description.
     */
    void renderHypothesis(const hssh::DebugHypothesis& hypothesis, float metersPerCell) const;

    /**
     * renderHypothesis draws the hypothesis using a color as determined by the value of the specified featureIndex.
     *
     * \param    hypothesis          Hypothesis to be rendered
     * \param    normalizedValue     Value assigned to the hypothesis in the range [0, 1].
     */
    void renderHypothesis(const hssh::DebugHypothesis& hypothesis, double normalizedValue, float metersPerCell) const;

    /**
     * renderHypothesisDistribution renders the hypothesis using colors assigned from the probability distribution. Each
     * color is assigned its own channel -- red, green, or blue -- and the final color is a weighted combination of
     * these colors.
     */
    void renderHypothesisDistribution(const hssh::DebugHypothesis& hypothesis, float metersPerCell) const;

private:
    GLColor unknownColor_;
    GLColor decisionPointColor_;
    GLColor destinationColor_;
    GLColor pathColor_;
    LinearColorInterpolator valueInterpolator_;

    AreaExtentRenderer extentRenderer_;

    // Helpers for doing the rendering
    void drawBoundary(const hssh::AreaExtent& extent,
                      const GLColor& boundaryColor,
                      float metersPerCell,
                      float filledScale = 1.0) const;
    void drawStatistics(const hssh::DebugHypothesis& hypothesis) const;
    void drawEndpointsIfPath(const hssh::DebugHypothesis& hypothesis) const;
    GLColor selectColorFromType(hssh::HypothesisType type) const;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_AREA_SUBGRAPH_RENDERER_H

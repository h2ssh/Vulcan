/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     voronoi_isovist_gradients_renderer.h
 * \author   Collin Johnson
 *
 * Declaration of VoronoiIsovistGradientsRenderer.
 */

#ifndef UI_COMPONENTS_VORONOI_ISOVIST_GRADIENTS_RENDERER_H
#define UI_COMPONENTS_VORONOI_ISOVIST_GRADIENTS_RENDERER_H

#include "hssh/types.h"
#include "ui/common/color_interpolator.h"
#include <GL/gl.h>
#include <vector>

namespace vulcan
{
namespace hssh
{
class VoronoiIsovistGradients;
}
namespace hssh
{
class VoronoiIsovistMaxima;
}
namespace hssh
{
class VoronoiSkeletonGrid;
}
namespace ui
{

/**
 * VoronoiIsovistGradientsRenderer
 */
class VoronoiIsovistGradientsRenderer
{
public:
    /**
     * setRenderColors sets the color to render the local maxima and the color for rendering the relative
     * values of the gradients.
     */
    void setRenderColors(const GLColor& localMaximaColor, const std::vector<GLColor>& gradientColors);

    /**
     * setGradients sets all the current gradient information to be rendered. It takes some time to setup,
     * so change the gradients whenever they are changed by the program.
     */
    void setGradients(const hssh::VoronoiIsovistGradients& gradients,
                      const hssh::VoronoiIsovistMaxima& maxima,
                      const hssh::VoronoiSkeletonGrid& grid);

    /**
     * setProbabilities sets the probabilities associated with the cells in the skeleton.
     */
    void setProbabilities(const hssh::CellToTypeMap<double>& probabilities, const hssh::VoronoiSkeletonGrid& grid);

    /**
     * renderCellGradients renders the gradients
     */
    void renderCellGradients(void) const;

    /**
     * renderLocalMaxima renders the anchors associated with the local maxima.
     */
    void renderLocalMaxima(void) const;

    /**
     * renderProbabilities renders the probabilities associated with the cells.
     */
    void renderProbabilities(void) const;

private:
    std::vector<GLfloat> cellGradientVertices_;
    std::vector<GLfloat> cellGradientColors_;

    std::vector<GLfloat> localMaximaVertices_;
    std::vector<GLfloat> localMaximaColors_;

    std::vector<GLfloat> probabilitiesVertices_;
    std::vector<GLfloat> probabilitiesColors_;

    LinearColorInterpolator interpolator_;
    GLColor maximaColor_;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_VORONOI_ISOVIST_GRADIENTS_RENDERER_H

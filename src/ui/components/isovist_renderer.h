/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     isovist_renderer.h
 * \author   Collin Johnson
 *
 * Definition of IsovistRenderer.
 */

#ifndef UI_COMPONENTS_ISOVIST_RENDERER_H
#define UI_COMPONENTS_ISOVIST_RENDERER_H

#include "ui/common/color_interpolator.h"
#include "ui/common/ui_color.h"
#include "utils/isovist.h"

namespace vulcan
{
namespace math
{
class Histogram;
}
namespace ui
{

/**
 * IsovistRenderer provides two options for rendering isovists. The first option is to render a single isovist. When
 * drawing a single isovist, the isovist will be drawn as both the individual rays and a line going around the border.
 * When drawing a scalar field, the field will be a large number of colored rectangles -- not the most efficient, but
 * it works.
 */
class IsovistRenderer
{
public:
    /**
     * setRenderColors sets the colors for rendering the isovists. For the field colors, a LinearColorInterpolator is
     * used with these colors.
     */
    void setRenderColors(const GLColor& rayColor, const std::vector<GLColor>& fieldColors);

    /**
     * renderIsovist draws a single isovist as a collection of rays and a boundary.
     */
    void renderIsovist(const utils::Isovist& isovist) const;

    /**
     * renderIsovist draws an isovist using the provided color.
     */
    void renderIsovist(const utils::Isovist& isovist, const GLColor& color) const;


    /**
     * renderIsovistField renders a scalar field for the isovists using the specified value.
     */
    void renderIsovistField(utils::IsovistField::Iter begin,
                            utils::IsovistField::Iter end,
                            utils::Isovist::Scalar value,
                            double scale) const;

    /**
     * renderIsovistDerivField renders the scalar derivative field for isovists using the specified value.
     */
    void renderIsovistDerivField(utils::IsovistField::Iter begin,
                                 utils::IsovistField::Iter end,
                                 utils::Isovist::Scalar value,
                                 double scale) const;

private:
    GLColor rayColor_;
    LinearColorInterpolator scalarInterpolator_;
    CircularColorInterpolator orientationInterpolator_;

    void renderScalarField(utils::IsovistField::Iter begin,
                           utils::IsovistField::Iter end,
                           utils::Isovist::Scalar value,
                           double scale) const;
    void renderOrientationField(utils::IsovistField::Iter begin,
                                utils::IsovistField::Iter end,
                                utils::Isovist::Scalar scalar,
                                double scale) const;
    void renderDerivField(utils::IsovistField::Iter begin,
                          utils::IsovistField::Iter end,
                          utils::Isovist::Scalar value,
                          double scale) const;
    void drawScalar(const Point<double>& position, double value, double scale) const;
    void drawOrientation(const Point<double>& position, double orientation, double scale) const;
    void drawValueRect(const Point<double>& position, double scale) const;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_ISOVIST_RENDERER_H

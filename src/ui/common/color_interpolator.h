/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     color_interpolator.h
* \author   Collin Johnson
*
* Declaration of LinearColorInterpolator and CircularColorInterpolator.
*/

#ifndef UI_COMMON_COLOR_INTERPOLATOR_H
#define UI_COMMON_COLOR_INTERPOLATOR_H

#include "ui/common/ui_color.h"
#include <array>
#include <vector>

namespace vulcan
{
namespace ui
{

/**
* LinearColorInterpolator provides basic functionality for interpolating between a set of colors. The idea is that some
* quantities are best visualized based on their magnitude. To show this difference, low magnitude is assigned
* one color, while high magnitude is assigned another color. Values between the extremes should be assigned
* some color in between. The approach taken for calculating this color in the LinearColorInterpolator is to select
* color values based on the vector running between the two adjacent colors in the interval from [0,1] where
* colors are assigned along intervals [1.0/(num_colors-1)]. The RGB values will be assigned based on
* how far along this vector the desired color is.
*/
class LinearColorInterpolator
{
public:

    /**
    * Default constructor for LinearColorInterpolator.
    */
    LinearColorInterpolator(void);

    /**
    * Constructor for LinearColorInterpolator.
    *
    * \param    lowColor        Color for low values
    * \param    highColor       Color for high values
    */
    LinearColorInterpolator(const GLColor& lowColor, const GLColor& highColor);

    /**
    * Constructor for LinearColorInterpolator.
    *
    * \pre      colors.size() > 1
    * \param    colors          Colors to interpolate between
    */
    LinearColorInterpolator(const std::vector<GLColor>& colors);

    /**
    * setColors changes the colors used for interpolation.
    *
    * \param    lowColor        Color for low values
    * \param    highColor       Color for high values
    */
    void setColors(const GLColor& lowColor, const GLColor& highColor);

    /**
    * setColors changes the colors used for interpolation.
    *
    * \pre      colors.size() > 1
    * \param    colors          Colors to use for interpolation
    */
    void setColors(const std::vector<GLColor>& colors);

    /**
    * calculateColor determines the color for a value with the specified value within the range [0, 1].
    *
    * \param    value           Value of the color to be calculated
    * \param    values          Vector of values of the color to be calculated
    */
    GLColor              calculateColor(float                     value) const;
    std::vector<GLColor> calculateColor(const std::vector<float>& values) const;

private:
    
    using ColorDelta = std::array<float, 4>;

    std::vector<GLColor>    colors;
    std::vector<ColorDelta> colorDeltas;
    float                   interval;
};


/**
* CircularColorInterpolator is a color interpolator for values that wraparound, mostly just angles. The value range
* is one of three common angle ranges. The range is specified and then the value is directly mapped onto the hue for
* the color. The saturation and value are set to 1.0. The alpha can be specified to whatever value is desired.
*/
class CircularColorInterpolator
{
public:

    enum ValueRange
    {
        pi_over_two,        ///< Range of the value is [-pi/2, pi/2] -- the range result from atan()
        pi,                 ///< Range of the value is [-pi, pi] -- the normal range for angles in Vulcan
        two_pi              ///< Range of the value is [0, 2pi] -- the representation for angle_range_t and others
    };

    /**
    * calculateColor interpolate
    */
    GLColor calculateColor(float value, ValueRange range, float alpha = 0.8f) const;

    /**
    * calculateColors
    */
    std::vector<GLColor> calculateColors(const std::vector<float>& values, ValueRange range, float alpha = 0.8f) const;

private:

    float calculateHue(float value, ValueRange range) const;
};

/**
* ValueColorInterpolator provides a simple interpolation for the Value in HSV. The RGB color is provided. It is
* converted to HSV. Then the Value is scaled based on the provided value for the interpolation in the range [0, 1].
*/
class ValueColorInterpolator
{
public:
    
    /**
    * Constructor for ValueColorInterpolator.
    * 
    * The alpha for this GLColor will always be the same.
    * 
    * \param    color       The full value color to use -- this color is returned when calculateColor(1.0) is called.
    */
    ValueColorInterpolator(const GLColor& color);
    
    /**
    * calculateColor interpolates the 
    */
    GLColor calculateColor(double value);
    
private:
    
    float hue_;
    float saturation_;
    float value_;
    float alpha_;
};

} // namespace ui
} // namespace vulcan

#endif // UI_COMMON_COLOR_INTERPOLATOR_H

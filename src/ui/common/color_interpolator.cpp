/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     color_interpolator.cpp
* \author   Collin Johnson
*
* Definition of LinearColorInterpolator and CircularColorInterpolator.
*/

#include <ui/common/color_interpolator.h>
#include <core/angle_functions.h>
#include <boost/algorithm/clamp.hpp>
#include <wx/image.h>
#include <algorithm>
#include <cassert>

namespace vulcan
{
namespace ui
{

/////////////////////// LinearColorInterpolator implementation /////////////////////////////

LinearColorInterpolator::LinearColorInterpolator(void)
: interval(1.0f)
{
}


LinearColorInterpolator::LinearColorInterpolator(const GLColor& lowColor, const GLColor& highColor)
{
    setColors(lowColor, highColor);
}


LinearColorInterpolator::LinearColorInterpolator(const std::vector<GLColor>& colors)
{
    setColors(colors);
}


void LinearColorInterpolator::setColors(const GLColor& lowColor, const GLColor& highColor)
{
    std::vector<GLColor> newColors(2);
    newColors[0] = lowColor;
    newColors[1] = highColor;

    setColors(newColors);
}


void LinearColorInterpolator::setColors(const std::vector<GLColor>& colors)
{
    assert(colors.size() > 1);

    this->colors = colors;
    interval     = 1.0f / (colors.size() - 1);

    colorDeltas.clear();

    // Store the gap between colors and then linearly interpolate between the them based on the value
    for(size_t n = 1; n < colors.size(); ++n)
    {
        colorDeltas.push_back({{colors[n].red()   - colors[n-1].red(),
                                colors[n].green() - colors[n-1].green(),
                                colors[n].blue()  - colors[n-1].blue(),
                                colors[n].alpha() - colors[n-1].alpha()}});
    }
    
    colorDeltas.push_back({{0.0f, 0.0f, 0.0f, 0.0f}});
}


GLColor LinearColorInterpolator::calculateColor(float value) const
{
    assert(colors.size() > 1u);
    
    // Clamp to ensure a valid value
    value = boost::algorithm::clamp(value, 0.0f, 1.0f);
    
    // clamp to the maximum index if the value is 1 so the index doesn't creep over colors.size()-1
    std::size_t colorIndex = (value == 1.0f) ? (colors.size() - 2) : (value / interval);
    float       colorScale = (value - colorIndex*interval) / interval;   // how close to the next color is the value?
    
    colorIndex = boost::algorithm::clamp(colorIndex, 0u, colors.size()-1u);
    
    return GLColor(colors[colorIndex].red()   + colorDeltas[colorIndex][0]*colorScale,
                   colors[colorIndex].green() + colorDeltas[colorIndex][1]*colorScale,
                   colors[colorIndex].blue()  + colorDeltas[colorIndex][2]*colorScale,
                   colors[colorIndex].alpha() + colorDeltas[colorIndex][3]*colorScale);
}

std::vector<GLColor> LinearColorInterpolator::calculateColor(const std::vector<float>& values) const
{
    std::vector<GLColor> outputColors;
    outputColors.reserve(values.size());
    
    for(auto value : values)
    {
        outputColors.push_back(calculateColor(value));
    }
    
    return outputColors;
}

/////////////////////// CircularColorInterpolator implementation /////////////////////////////

GLColor CircularColorInterpolator::calculateColor(float      value,
                                                  ValueRange range,
                                                  float      alpha) const
{
    auto rgbColor = wxImage::HSVtoRGB(wxImage::HSVValue(calculateHue(value, range), 1.0, 1.0));
    return GLColor(rgbColor.red / 255.0, rgbColor.green / 255.0, rgbColor.blue / 255.0, alpha);
}


std::vector<GLColor> CircularColorInterpolator::calculateColors(const std::vector<float>& values,
                                                                ValueRange                range,
                                                                float                     alpha) const
{
    std::vector<GLColor> colors;
    colors.reserve(values.size());

    for(auto v : values)
    {
        colors.push_back(calculateColor(v, range, alpha));
    }

    return colors;
}


float CircularColorInterpolator::calculateHue(float value, ValueRange range) const
{
    switch(range)
    {
    case pi_over_two:
        return (wrap_to_pi_2(value) + M_PI_2) / M_PI;

    case pi:
        return (wrap_to_pi(value) + M_PI) / (2.0 * M_PI);

    case two_pi:
        return wrap_to_2pi(value) / (2.0 * M_PI);
    }

    // Default means an unknown ValueRange, so just return the values
    return boost::algorithm::clamp(value, 0.0f, 1.0f);
}

/////////////////////// ValueColorInterpolator implementation /////////////////////////////

ValueColorInterpolator::ValueColorInterpolator(const GLColor& color)
{
    auto hsvColor = wxImage::RGBtoHSV(wxImage::RGBValue(color.red() * 255, color.green() * 255, color.blue() * 255));
    
    hue_ = hsvColor.hue;
    saturation_ = hsvColor.saturation;
    value_ = hsvColor.value;
    alpha_ = color.alpha();
}


GLColor ValueColorInterpolator::calculateColor(double value)
{
    auto rgbColor = wxImage::HSVtoRGB(wxImage::HSVValue(hue_, saturation_, value * value_));
    return GLColor(rgbColor.red / 255.0, rgbColor.green / 255.0, rgbColor.blue / 255.0, alpha_);
}

} // namespace ui
} // namespace vulcan

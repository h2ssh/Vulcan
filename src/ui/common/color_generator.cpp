/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "ui/common/color_generator.h"
#include "ui/common/ui_color.h"
#include <wx/image.h>
#include <cstdlib>
#include <ctime>

namespace vulcan
{
namespace ui
{

const int NUM_DEFINED_COLORS = 8;

GLColor FIXED_COLORS[NUM_DEFINED_COLORS] = {
    GLColor(135, 206, 235, 255),
    GLColor(255, 99, 71, 255),
    GLColor(106, 90, 205, 255),
    GLColor(218, 112, 214, 255),
    GLColor(50, 205, 50, 255),
    GLColor(128, 0, 128, 255),
    GLColor(46, 139, 87, 255),
    GLColor(160, 82, 45, 255)
};

std::vector<GLColor> generate_colors(int numColors, float alpha, bool randomAlpha)
{
    std::vector<GLColor> colors;

    for(int n = 0; n < numColors; ++n)
    {
        if(n < NUM_DEFINED_COLORS)
        {
            colors.emplace_back(FIXED_COLORS[n].red(), FIXED_COLORS[n].green(), FIXED_COLORS[n].blue(), randomAlpha ? drand48() : FIXED_COLORS[n].alpha());
        }
        else
        {
            colors.push_back(generate_color(alpha, randomAlpha));
        }
    }

    return colors;
}


std::vector<GLColor> generate_colors_even(int numColors, float minHue, float maxHue, float alpha)
{
    numColors = std::max(1, numColors);
    minHue = std::min(std::max(0.0f, minHue), maxHue);
    maxHue = std::max(std::min(1.0f, maxHue), minHue);

    float hueStep = (numColors > 1) ? (maxHue - minHue) / (numColors - 1) : 0.0f;

    std::vector<GLColor> colors;
    for(int n = 0; n < numColors; ++n)
    {
        auto rgbColor = wxImage::HSVtoRGB(wxImage::HSVValue(minHue + (n*hueStep), 1.0, 1.0));
        colors.emplace_back(rgbColor.red / 255.0, rgbColor.green / 255.0, rgbColor.blue / 255.0, alpha);
    }

    return colors;
}


GLColor generate_color(float alpha, bool randomAlpha)
{
    static bool seeded = false;

    if(!seeded)
    {
        srand48(time(0));
        seeded = true;
    }

    // Sample in HSV space because the colors are of higher quality -- not as much
    // ugly brown as with RGB, which is a very non-linear space
    auto rgbColor = wxImage::HSVtoRGB(wxImage::HSVValue(drand48(),              // sample full range of hues
                                                        0.3 + drand48()*0.7,    // don't have the saturation too low
                                                        1.0));                  // always full value -- alpha controls brightness
    return GLColor(rgbColor.red   / 255.0,
                   rgbColor.green / 255.0,
                   rgbColor.blue  / 255.0,
                   randomAlpha ? drand48() : alpha);
}

}
}

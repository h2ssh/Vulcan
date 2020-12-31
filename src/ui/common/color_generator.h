/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UI_COMMON_COLOR_GENERATOR_H
#define UI_COMMON_COLOR_GENERATOR_H

#include <vector>

namespace vulcan
{
namespace ui
{

class GLColor;

/**
 * generate_colors generates random GLColor. Alpha can optionally be set or generated depending
 * on the need.
 *
 * \param    numColors           Number of colors to generate
 * \param    alpha               Fixed alpha value to use for colors (default = 1.0f)
 * \param    randomAlpha         Randomly generate alpha             (default = false)
 * \return   Vector containing specified number of colors.
 */
std::vector<GLColor> generate_colors(int numColors, float alpha = 1.0f, bool randomAlpha = false);

/**
 * generate_colors_even generates evenly spaced GLColors in the requested range of hues. Alpha can optionally be set.
 *
 * Range is [minHue, maxHue], with min clamped to [0, maxHue), and max clamped to (minHue,1.0].
 *
 * \param    numColors           Number of colors to generate
 * \param    minHue              First hue value to use
 * \param    maxHue              Highest hue value to use
 * \param    alpha               Fixed alpha value to use for colors (default = 1.0f)
 * \return   Vector containing specified number of colors.
 */
std::vector<GLColor> generate_colors_even(int numColors, float minHue, float maxHue, float alpha = 1.0f);

/**
 * generate_color generates a single color.
 *
 * \param    alpha               Fixed alpha value to use for colors (default = 1.0f)
 * \param    randomAlpha         Randomly generate alpha             (default = false)
 * \return   Vector containing specified number of colors.
 */
GLColor generate_color(float alpha = 1.0f, bool randomAlpha = false);

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMMON_COLOR_GENERATOR_H

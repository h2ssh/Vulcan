/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     default_colors.h
 * \author   Collin Johnson
 *
 * Declaration of accessors for a variety of default colors for use in rendering data in the UIs.
 * These colors can be used for default constructed rendering objects. It allows renderers to be
 * created without an associated config file to maintain consistency across various implementations.
 * While the config file/params approach is more flexible, we really just want to draw stuff the
 * same color everywhere so its consistent. Renderers can leave the option of specifying a color, but
 * they can default to the system-wide colors when feasible. If a system color doesn't make sense, just
 * provide a good default color and hopefully future uses will be compatible with it.
 */

#ifndef UI_COMMON_DEFAULT_COLORS_H
#define UI_COMMON_DEFAULT_COLORS_H

namespace vulcan
{
namespace ui
{

class GLColor;

// Standard colors for rendering LPMs and other information in local metric space
const GLColor& robot_color(void);
const GLColor& occupied_color(void);
const GLColor& dynamic_color(void);
const GLColor& quasi_static_color(void);
const GLColor& limited_visibility_color(void);
const GLColor& hazard_color(void);
const GLColor& caution_color(void);
const GLColor& frontier_color(void);
const GLColor& target_color(void);

// Standard colors for rendering different area types
const GLColor& decision_point_color(void);
const GLColor& destination_color(void);
const GLColor& path_color(void);
const GLColor& area_color(void);

// Standard colors for rendering different information in MPEPC
const GLColor& social_mpepc_color(void);
const GLColor& regular_mpepc_color(void);

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMMON_DEFAULT_COLORS_H

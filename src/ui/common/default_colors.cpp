/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     default_colors.cpp
* \author   Collin Johnson
*
* Definition of the default color accessor functions.
*/

#include <ui/common/default_colors.h>
#include <ui/common/ui_color.h>

namespace vulcan
{
namespace ui
{

const GLColor& robot_color(void)
{
    static GLColor robot(140, 134, 233, 215);
    return robot;
}


const GLColor& occupied_color(void)
{
    static GLColor occ(0, 0, 0, 255);
    return occ;
}


const GLColor& dynamic_color(void)
{
    static GLColor dynamic(0, 255, 0, 255);
    return dynamic;
}


const GLColor& quasi_static_color(void)
{
    static GLColor quasi(67, 169, 234, 255);
    return quasi;
}


const GLColor& limited_visibility_color(void)
{
    static GLColor limited(97, 0, 172, 255);
    return limited;
}


const GLColor& hazard_color(void)
{
    static GLColor hazard(231, 49, 7, 255);
    return hazard;
}


const GLColor& caution_color(void)
{
    static GLColor caution(255, 230, 22, 160);
    return caution;
}


const GLColor& frontier_color(void)
{
    static GLColor frontier(184, 12, 255, 200);
    return frontier;
}


const GLColor& target_color(void)
{
    static GLColor target(181, 0, 17, 100);
    return target;
}


const GLColor& decision_point_color(void)
{
    static GLColor decision(7, 55, 237, 200);
    return decision;
}


const GLColor& destination_color(void)
{
    static GLColor destination(224, 6, 6, 200);
    return destination;
}


const GLColor& path_color(void)
{
    static GLColor path(24, 145, 0, 200);
    return path;
}


const GLColor& area_color(void)
{
    static GLColor path(0, 0, 0, 200);
    return path;
}


const GLColor& social_mpepc_color(void)
{
    static GLColor social(255, 99, 71, 255);
    return social;
}


const GLColor& regular_mpepc_color(void)
{
    static GLColor regular(135, 206, 235, 255);
    return regular;
}

} // namespace ui
} // namespace vulcan

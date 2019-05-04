/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hssh_colors.h
* \author   Collin Johnson
*
* Declaration of color functions for converting HSSH types to colors for the UI.
*/

#ifndef UI_COMMON_HSSH_COLORS_H
#define UI_COMMON_HSSH_COLORS_H

#include <ui/common/default_colors.h>
#include <hssh/types.h>
#include <hssh/local_topological/area_detection/labeling/hypothesis_type.h>

namespace vulcan
{
namespace ui
{

inline const GLColor& color_from_local_area_type(hssh::AreaType type)
{
    switch(type)
    {
    case hssh::AreaType::decision_point:
        return decision_point_color();

    case hssh::AreaType::destination:
        return destination_color();

    case hssh::AreaType::path_segment:
        return path_color();

    default:
        return area_color();
    }
}


inline const GLColor& color_from_hypothesis_type(hssh::HypothesisType type)
{
    switch(type)
    {
        case hssh::HypothesisType::kDest:
            return destination_color();

        case hssh::HypothesisType::kDecision:
            return decision_point_color();

        case hssh::HypothesisType::kPath:
            return path_color();

        case hssh::HypothesisType::kArea:
        default:
            return area_color();
    }

    return area_color();
}

}
}

#endif // UI_COMMON_HSSH_COLORS_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     small_scale_star_builder.cpp
 * \author   Collin Johnson
 *
 * Definition SmallScaleStarBuilder.
 */

#include "hssh/local_topological/area_detection/labeling/small_scale_star_builder.h"
#include "hssh/local_topological/area_detection/labeling/beam_star_builder.h"
#include "hssh/local_topological/area_detection/labeling/path_similarity_star_builder.h"
#include <cassert>
#include <iostream>

namespace vulcan
{
namespace hssh
{

std::unique_ptr<SmallScaleStarBuilder> create_small_scale_star_builder(const small_scale_star_builder_params_t& params)
{
    if (params.starBuilderType == BEAM_STAR_BUILDER_TYPE) {
        return std::unique_ptr<SmallScaleStarBuilder>(new BeamStarBuilder(params.beamBuilderParams));
    } else if (params.starBuilderType == PATH_SIMILARITY_STAR_BUILDER_TYPE) {
        return std::unique_ptr<SmallScaleStarBuilder>(new PathSimilarityStarBuilder(params.similarityBuilderParams));
    } else {
        std::cerr << "ERROR:create_small_star_builder:Unknown SmallScaleStarBuilder type:" << params.starBuilderType
                  << std::endl;
        assert(false);
        return std::unique_ptr<SmallScaleStarBuilder>();
    }
}

}   // namespace hssh
}   // namespace vulcan

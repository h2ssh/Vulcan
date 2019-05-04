/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     beam_star_builder.h
* \author   Collin Johnson
*
* Declaration of BeamStarBuilder for the beam-based star building approach.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_BEAM_STAR_BUILDER_H
#define HSSH_LOCAL_TOPOLOGICAL_BEAM_STAR_BUILDER_H

#include <hssh/local_topological/area_detection/labeling/small_scale_star_builder.h>
#include <hssh/local_topological/params.h>

namespace vulcan
{
namespace hssh
{

const std::string BEAM_STAR_BUILDER_TYPE("beam");

/**
* BeamStarBuilder constructs a SmallScaleStar using a simple criteria to determine if two
* gateways are part of a single path. The beam criteria requires lines extending from the
* endpoints and in the normal direction of each gateway in the path to overlap. Furthermore,
* the path must be unique. If a gateway could potentially belong to two paths, then it belongs
* to no paths.
*/
class BeamStarBuilder : public SmallScaleStarBuilder
{
public:

    /**
    * Constructor for BeamStarBuilder.
    *
    * \param    params      Parameters for tweaking the star building
    */
    BeamStarBuilder(const beam_star_builder_params_t& params);

    // SmallScaleStarBuilder interface
    SmallScaleStar buildStar(const std::vector<Gateway>& gateways) const override;
    SmallScaleStar buildStar(const std::vector<Gateway>&   gateways,
                                     const Point<float>&     center,
                                     const math::Rectangle<float>& boundary) const override;
    int numGatewaysAlignedToAxis(const std::vector<Gateway>& gateways, double axisDirection) const override;
    bool areGatewaysAligned(const Gateway& lhs, const Gateway& rhs, const Point<float>& center) const override;

private:

    beam_star_builder_params_t params;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_BEAM_STAR_BUILDER_H

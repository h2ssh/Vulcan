/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     small_scale_star_builder.h
* \author   Collin Johnson
*
* Declaration of SmallScaleStarBuilder interface and create_small_scale_star_builder() factory.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_SMALL_SCALE_STAR_BUILDER_H
#define HSSH_LOCAL_TOPOLOGICAL_SMALL_SCALE_STAR_BUILDER_H

#include "math/geometry/rectangle.h"
#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh
{

class Gateway;
struct AreaExtent;
struct small_scale_star_builder_params_t;
class  SmallScaleStar;
class  SmallScaleStarBuilder;

/**
 * create_small_scale_star_builder is a factory for creating new instances of SmallScaleStarBuilder
 * based on a string identifier for the desired star builder type. The builder type to be created is
 * a field in small_scale_star_builder_params_t.
 *
 * \param    params          Parameters for star builders
 * \return   Instance of SmallScaleStarBuilder.
 */
std::unique_ptr<SmallScaleStarBuilder> create_small_scale_star_builder(const small_scale_star_builder_params_t& params);

/**
* SmallScaleStarBuilder is an interface for classes handling the task of creating the small-scale star representation of a
* place based on the active gateways bounding it.
*
* A number of choices exist in the implementation of the small-scale star. The particulars of all SmallScaleStarBuilder instances
* are as follows:
*
*   0) The path indices are numeric and start with 0.
*   1) Path indices are assigned in order of increasing orientation angle, with index 0 assigned to the path closest
*      to 0 radians.
*   2) The plus direction is assigned to the path fragment in a path belonging to the gateway with the angle closer to
*      0 when the angle is the direction of a vector pointing from the center of the small-scale star to the center of
*      the gateway.
*/
class SmallScaleStarBuilder
{
public:

    virtual ~SmallScaleStarBuilder(void) {}

    /**
    * buildStar builds a new small-scale star using the provided gateways. The boundary of the area is the bounding box
    * around the gateways.
    *
    * \param    gateways            Gateways to use for building the star
    * \return   SmallScaleStar descriptor for the place bounded by the gateways.
    */
    virtual SmallScaleStar buildStar(const std::vector<Gateway>& gateways) const = 0;

    /**
    * buildStar builds a new small-scale star using the provided gateways. The boundary specifies the extent
    * of the area associated with the gateways.
    *
    * \param    gateways            Gateways associated with the area for the star
    * \param    center              Center of the area containing the gateways
    * \param    boundary            Boundary of the area
    * \return   SmallScaleStar descriptor for the place bounded by the gateways.
    */
    virtual SmallScaleStar buildStar(const std::vector<Gateway>&   gateways,
                                     const Point<float>&     center,
                                     const math::Rectangle<float>& boundary) const = 0;
    
    /**
    * numGatewaysAlignedToAxis uses the method for building the star to check if all of the provided gateways are
    * aligned to the provided axis direction, such that they form a single path.
    * 
    * \param    gateways            Gateways to check for alignment with the axis
    * \param    axisDirection       Direction of the axis of alignment
    * \return   Number of gateways aligned to the specified axis.
    */
    virtual int numGatewaysAlignedToAxis(const std::vector<Gateway>& gateways, double axisDirection) const = 0;
    
    /**
    * areGatewaysAligned asks if a pair of gateways are aligned. Aligned gateways do not necessarily belong to the same
    * path in the small-scale star because they could be ambiguously aligned with other other gateways. For the sake of
    * considering the constraints on an area though, only this pairwise alignment criteria matters.
    * 
    * \param    lhs                 One of the gateways
    * \param    rhs                 The other gateway
    * \param    center              Center of the area bounded by the gateways
    * \return   True if the two gateways are aligned.
    */
    virtual bool areGatewaysAligned(const Gateway& lhs, const Gateway& rhs, const Point<float>& center) const = 0;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_SMALL_SCALE_STAR_BUILDER_H

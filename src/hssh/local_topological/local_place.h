/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_place.h
* \author   Collin Johnson
*
* Declaration of LocalPlaceOld.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_LOCAL_PLACE_H
#define HSSH_LOCAL_TOPOLOGICAL_LOCAL_PLACE_H

#include <core/pose.h>
#include <hssh/local_metric/lpm.h>
#include <hssh/local_topological/small_scale_star.h>
#include <hssh/local_topological/gateway.h>
#include <hssh/local_topological/place_extent.h>

namespace vulcan
{
namespace hssh
{

/**
* LocalPlaceOld is the place abstraction produced by the local topology and local metric
* layers of the HSSH. The LocalPlaceOld provides a rich description of a place in the
* robot's environment, giving both a high resolution description of the place's structure
* and an abstract descriptor of the place's topology. Additionally, the gateways for the place
* provide the locations through which the robot could enter a place, making them special points
* to consider when evaluating further properties of the place.
*/
class LocalPlaceOld
{
public:

    /**
    * Default constructor for LocalPlaceOld.
    */
    LocalPlaceOld(void)
        : id(-1)
    {
    }

    /**
    * Constructor for LocalPlaceOld.
    *
    * \param    id          Unique identifier for the place
    * \param    gateways    Gateways bounding the place decisions
    * \param    star        SmallScaleStar describing the topology of the place
    * \param    lpm         LPM with the detailed metric description of the place
    * \param    extent      Description of the boundary of the place
    * \param    reference   Reference pose for the place, giving its center position
    */
    LocalPlaceOld(uint32_t                 id,
               const std::vector<Gateway>& gateways,
               const SmallScaleStar&       star,
               const LocalPerceptualMap&   lpm,
               const place_extent_t&       extent,
               const pose_t&        reference)
        : id(id)
        , gateways(gateways)
        , star(star)
        , lpm(lpm)
        , extent(extent)
        , reference(reference)
    {
    }

    /**
    * getId retrieves the id of the place.
    */
    uint32_t getId(void) const { return id; }

    /**
    * getGateways retrieves the gateways representing the path boundaries of the place.
    */
    std::vector<Gateway> getGateways(void) const { return gateways; }

    /**
    * getStar retrieves the small-scale star representing the topology of the place.
    */
    SmallScaleStar getStar(void) const { return star; }

    /**
    * getLPM retrieves the LPM representing the metric map of the place.
    */
    const LocalPerceptualMap& getLPM(void) const { return lpm; }

    /**
    * getExtent retrieves the extent of the place.
    */
    place_extent_t getExtent(void) const { return extent; }

    /**
    * getReferencePose retrieves the reference pose defining the metric location of the center of the place
    * in the global reference frame. The orientation of the pose provides the direction in which the x-axis
    * of the place points.
    */
    pose_t getReferencePose(void) const { return reference; }

private:

    uint32_t id;

    std::vector<Gateway> gateways;
    SmallScaleStar       star;
    LocalPerceptualMap   lpm;
    place_extent_t       extent;
    pose_t        reference;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_LOCAL_PLACE_H

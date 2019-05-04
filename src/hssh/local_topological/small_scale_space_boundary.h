/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     small_scale_space_boundary.h
* \author   Collin Johnson
*
* Declaration of SmallScaleSpaceBoundary interface and create_small_scale_space_boundary factory.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_SMALL_SCALE_SPACE_BOUNDARY_H
#define HSSH_LOCAL_TOPOLOGICAL_SMALL_SCALE_SPACE_BOUNDARY_H

#include <hssh/local_topological/event.h>
#include <math/geometry/rectangle.h>
#include <boost/optional.hpp>
#include <memory>
#include <string>

namespace vulcan
{
namespace hssh
{

class LocalPerceptualMap;
class LocalPose;
class LocalTopoMap;
class SmallScaleSpaceBoundary;

/**
* create_small_scale_space_boundary creates an appropriate instance of SmallScaleSpaceBoundary.
*
* \param    type        Subclass of SSSB to be created
* \pre  type is a valid type string descriptor of a subclass of SmallScaleSpaceBoundary.
* \return   An instance of the desired subclass of SmallScaleSpaceBoundary.
*/
std::unique_ptr<SmallScaleSpaceBoundary> create_small_scale_space_boundary(const std::string& type);


/**
* SmallScaleSpaceBoundary is an interface for controlling the size of small-scale space based on the local topology of
* the environment. The boundary is updated whenever a new LocalAreaEvent occurs.
*
* The boundary is computed using the current LocalTopoMap, LPM, and the LocalAreaEvents that have occurred.
*
* Not all events will generate a new boundary. Thus, the new boundary is optional when being computed.
*/
class SmallScaleSpaceBoundary
{
public:

    using MapBoundary = math::Rectangle<float>;

    /**
    * computeBoundary computes the current boundary of small-scale space based on new LocalTopoEvents that indicate
    * the robot has moved within the local topological structure of the environment.
    *
    * Not all events generate new boundaries, though the returned boundary is optional, depending on exactly what
    * events occurred.
    *
    * \param    events          Events that occurred
    * \param    topoMap         Local topological map of the current LPM
    * \param    pose            Pose of the robot in the LPM
    * \param    lpm             Current LPM
    * \return   An updated boundary for small-scale space, if a new boundary happens to be needed.
    */
    virtual boost::optional<MapBoundary> computeBoundary(const LocalAreaEventVec& events,
                                                         const LocalTopoMap& topoMap,
                                                         const LocalPose& pose,
                                                         const LocalPerceptualMap& lpm) = 0;

    virtual ~SmallScaleSpaceBoundary(void) { }
};


}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_SMALL_SCALE_SPACE_BOUNDARY_H

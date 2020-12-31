/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     fixed.h
* \author   Collin Johnson
*
* Declaration of FixedBoundary implementation of SmallScaleSpaceBoundary interface.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_BOUNDARY_FIXED_H
#define HSSH_LOCAL_TOPOLOGICAL_BOUNDARY_FIXED_H

#include "hssh/local_topological/small_scale_space_boundary.h"

namespace vulcan
{
namespace hssh
{

const std::string kFixedBoundaryType("fixed");

/**
* FixedBoundary is a boundary that does not change based on the local topology. The fixed boundary is simply whatever
* the current LPM boundary is. As a result, there's nothing to be done for the FixedBoundary. It's just a null
* operation.
*
* The FixedBoundary should be used whenever there is a ground-truth map being used. Otherwise, the map might be
* directed to be truncated, thereby breaking the ground-truthness of it.
*/
class FixedBoundary : public SmallScaleSpaceBoundary
{
public:

    // SmallScaleSpaceBoundary interface
    boost::optional<MapBoundary> computeBoundary(const LocalAreaEventVec& events,
                                                 const LocalTopoMap& topoMap,
                                                 const LocalPose& pose,
                                                 const LocalPerceptualMap& lpm) override;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_BOUNDARY_FIXED_H

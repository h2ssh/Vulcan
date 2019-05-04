/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     fixed.cpp
* \author   Collin Johnson
*
* Definition of FixedBoundary.
*/

#include <hssh/local_topological/boundary/fixed.h>

namespace vulcan
{
namespace hssh
{

boost::optional<SmallScaleSpaceBoundary::MapBoundary> FixedBoundary::computeBoundary(const LocalAreaEventVec& events,
                                                                                     const LocalTopoMap& topoMap,
                                                                                     const LocalPose& pose,
                                                                                     const LocalPerceptualMap& lpm)
{
    // The boundary should never be changed
    return boost::none;
}

}
}

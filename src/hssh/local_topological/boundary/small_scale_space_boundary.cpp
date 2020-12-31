/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     small_scale_space_boundary.cpp
* \author   Collin Johnson
*
* Definition of create_small_scale_space_boundary factory.
*/

#include "hssh/local_topological/small_scale_space_boundary.h"
#include "hssh/local_topological/boundary/fixed.h"
#include "hssh/local_topological/boundary/shrink_and_stretch.h"

namespace vulcan
{
namespace hssh
{

std::unique_ptr<SmallScaleSpaceBoundary> create_small_scale_space_boundary(const std::string& type)
{
    if(type == kFixedBoundaryType)
    {
        return std::unique_ptr<SmallScaleSpaceBoundary>(new FixedBoundary);
    }
    else if(type == kShrinkAndStretchBoundaryType)
    {
        return std::unique_ptr<SmallScaleSpaceBoundary>(new ShrinkAndStretchBoundary);
    }

    std::cerr << "ERROR: create_small_scale_space_boundary: Unknown boundary type:" << type << '\n';
    assert(false);
    return std::unique_ptr<SmallScaleSpaceBoundary>();
}

}
}

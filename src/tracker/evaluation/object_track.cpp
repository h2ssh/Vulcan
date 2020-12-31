/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object_track.cpp
* \author   Collin Johnson
*
* Definition of ObjectTrack.
*/

#include "tracker/evaluation/object_track.h"
#include <cassert>

namespace vulcan
{
namespace tracker
{

ObjectTrack::ObjectTrack(DynamicObject::ConstPtr object)
: currentEstimate_(object)
{
    assert(currentEstimate_);
}


void ObjectTrack::addEstimate(DynamicObject::ConstPtr estimate)
{
    currentEstimate_ = estimate;
}

} // namespace tracker
} // namespace vulcan

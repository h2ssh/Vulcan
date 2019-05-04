/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_VECTOR_H
#define LCMTYPES_VECTOR_H

#include <lcmtypes/vulcan_lcm_vector_t.h>
#include <core/vector.h>

namespace vulcan
{
namespace lcm
{

void allocate_lcm_vector(vulcan_lcm_vector_t& vector, int length);
void free_lcm_vector(vulcan_lcm_vector_t& vector);

void vulcan_vector_to_lcm_vector(const Vector& vulcanVector, vulcan_lcm_vector_t& lcmVector);
void lcm_vector_to_vulcan_vector(const vulcan_lcm_vector_t& lcmVector, Vector& vulcanVector);

}
}

#endif // LCMTYPES_VECTOR_H

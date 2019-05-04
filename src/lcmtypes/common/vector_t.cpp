/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <cassert>
#include <lcmtypes/common/vector_t.h>


void vulcan::lcm::allocate_lcm_vector(vulcan_lcm_vector_t& vector, int length)
{
    if(length > 0)
    {
        vector.length = length;
        vector.vector = new double[length];
    }
    else
    {
        vector.vector = 0;
        vector.length = 0;
    }
}


void vulcan::lcm::free_lcm_vector(vulcan_lcm_vector_t& vector)
{
    if(vector.vector)
    {
        delete [] vector.vector;
    }
}


void vulcan::lcm::vulcan_vector_to_lcm_vector(const Vector& vulcanVector, vulcan_lcm_vector_t& lcmVector)
{
    for(int i = lcmVector.length; --i >= 0;)
    {
        lcmVector.vector[i] = vulcanVector(i);
    }
}


void vulcan::lcm::lcm_vector_to_vulcan_vector(const vulcan_lcm_vector_t& lcmVector, Vector& vulcanVector)
{
    for(int i = lcmVector.length; --i >= 0;)
    {
        vulcanVector(i) = lcmVector.vector[i];
    }
}

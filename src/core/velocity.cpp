/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     velocity.cpp
* \author   Collin Johnson
*
* Definition of helper functions for velocity_t and acceleration_t.
*/

#include <core/velocity.h>
#include <iostream>

namespace vulcan
{

std::ostream& operator<<(std::ostream& out, const velocity_t& v)
{
    out<<'('<<v.linear<<','<<v.angular<<')';
    return out;
}


// Output operator: (linear, angular)
template <typename ostream>
std::ostream& operator<<(std::ostream& out, const acceleration_t& a)
{
    out<<'('<<a.linear<<','<<a.angular<<')';
    return out;
}

} // namespace vulcan

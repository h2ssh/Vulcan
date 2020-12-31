/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     id.cpp
 * \author   Collin Johnson
 *
 * Definition of load_next_id, save_next_id, next_id.
 */

#include "hssh/utils/id.h"
#include <fstream>

namespace vulcan
{
namespace hssh
{

static Id nextId = 0;


bool load_next_id(const std::string& name)
{
    std::ifstream in(name);

    if (in.is_open()) {
        in >> nextId;
    }

    return in.is_open();
}


bool save_next_id(const std::string& name)
{
    std::ofstream out(name);

    if (out.is_open()) {
        out << nextId;
    }

    return out.is_open();
}


Id next_id(void)
{
    return ++nextId;
}

}   // namespace hssh
}   // namespace vulcan
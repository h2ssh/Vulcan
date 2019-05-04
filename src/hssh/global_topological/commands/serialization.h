/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     serialization.h
* \author   Collin Johnson
* 
* Serialization support header for GlobalTopoCommand -- register all command types.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_COMMANDS_SERIALIZATION_H
#define HSSH_GLOBAL_TOPOLOGICAL_COMMANDS_SERIALIZATION_H

/////   Include all concrete type headers here   /////
#include <hssh/global_topological/commands/save_topo_slam_data.h>



// Serialization support for smart pointers
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

/////   Register each concrete command type here   /////
CEREAL_REGISTER_TYPE(vulcan::hssh::SaveTopoSlamDataCommand)

#endif // HSSH_GLOBAL_TOPOLOGICAL_COMMANDS_SERIALIZATION_H

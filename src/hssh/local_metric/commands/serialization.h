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
* A utility header file for programs that will be received LocalMetricCommands via a shared_ptr. All of the actual
* headers for the LocalMetricCommand messages must be explicitly included in the binary somewhere. This header contains
* all of the command headers to ensure that they are all properly included by only needing to include this one header.
*/

#ifndef HSSH_LOCAL_METRIC_COMMANDS_SERIALIZATION_H
#define HSSH_LOCAL_METRIC_COMMANDS_SERIALIZATION_H

#include <hssh/local_metric/commands/glass_evaluation.h>
#include <hssh/local_metric/commands/relocalize_in_lpm.h>
#include <hssh/local_metric/commands/rotate_lpm.h>
#include <hssh/local_metric/commands/truncate_lpm.h>
#include <hssh/local_metric/commands/set_map.h>
#include <hssh/local_metric/commands/set_slam_mode.h>
#include <hssh/local_metric/commands/save_metric_map.h>
#include <hssh/local_metric/commands/toggle_glass_mapping.h>

#endif // HSSH_LOCAL_METRIC_COMMANDS_SERIALIZATION_H

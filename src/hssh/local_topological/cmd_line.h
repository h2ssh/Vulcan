/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     cmd_line.h
 * \author   Collin Johnson
 *
 * Definition of all command-line argument names related to local_topo_hssh module.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_CMD_LINE_H
#define HSSH_LOCAL_TOPOLOGICAL_CMD_LINE_H

#include <string>

namespace vulcan
{
namespace hssh
{

const std::string kOneShotArg("one-shot");
const std::string kMapNameArg("map");
const std::string kDirectoryArg("directory");
const std::string kSaveEventsArg("save-events");
const std::string kSaveMapArg("save-map");
const std::string kConstraintLogProbArg("constraint-log-prob");
const std::string kRepeatLogProbArg("repeat-log-prob");
const std::string kMaxIterationsArg("mcmc-max-iterations");
const std::string kSamplesPerIterArg("mcmc-samples-per-iteration");

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_CMD_LINE_H

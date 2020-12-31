/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     decision_action.cpp
 * \author   Collin Johnson
 *
 * Definition of Decision.
 */

#include "planner/interface/decision.h"
#include "system/module_communicator.h"

namespace vulcan
{
namespace planner
{

Decision::Decision(DecisionDirection direction,
                   hssh::AreaType areaType,
                   Point<double> position,
                   double orientation,
                   bool isAbsolute)
: direction_(direction)
, type_(areaType)
, position_(position)
, orientation_(orientation)
, isAbsolute_(isAbsolute)
{
}

}   // namespace planner
}   // namespace vulcan

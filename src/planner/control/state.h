/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     state.h
 * \author   Collin Johnson
 *
 * Definition of ControlState
 */

#ifndef PLANNER_CONTROL_STATE_H
#define PLANNER_CONTROL_STATE_H

#include "hssh/local_metric/lpm.h"
#include "hssh/local_metric/pose.h"
#include "mpepc/metric_planner/messages.h"
#include <boost/optional.hpp>

namespace vulcan
{
namespace planner
{

/**
 * ControlState agglomerates all inputs and state needed for running a ControlTask.
 */
struct ControlState
{
    hssh::LocalPose pose;
    const hssh::LocalPerceptualMap* map;

    boost::optional<mpepc::metric_planner_status_message_t> metricPlannerStatus;

    // TODO: What additional state is needed?
};

}   // namespace planner
}   // namespace vulcan

#endif   // PLANNER_CONTROL_STATE_H

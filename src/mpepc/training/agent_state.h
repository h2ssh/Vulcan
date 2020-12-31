/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * @file
 * @author   Collin Johnson
 *
 * Declaration of agent_state_t, associated type aliases, and some simple save/load functions.
 */

#ifndef MPEPC_TRAINING_AGENT_STATE_H
#define MPEPC_TRAINING_AGENT_STATE_H

#include "mpepc/simulator/dynamic_object_trajectory.h"
#include <map>
#include <vector>

namespace vulcan
{
namespace mpepc
{

struct agent_state_t
{
    double x;
    double y;
    double xVel;
    double yVel;
    double normDist;

    agent_state_t(void) = default;
    agent_state_t(const dynamic_object_state_t& state, double normDist)
    : x(state.x)
    , y(state.y)
    , xVel(state.xVel)
    , yVel(state.yVel)
    , normDist(normDist)
    {
    }
};


using MotionObs = std::vector<agent_state_t>;
using MotionObsMap = std::map<int, MotionObs>;

/**
 * Save motion observations. The observations are saved in a file with the provided name.
 *
 * The format is a single agent_state_t per line with six values per line:
 *
 *   area_id x y xVel yVel normDist
 *
 * Area id is the area the agent was in for the particular measurement.
 *
 * \param    filename    Name of the file to save observations in
 * \param    obs         Observations to be saved
 */
void save_motion_observations(const std::string& filename, const MotionObsMap& obs);

/**
 * Load observations of agents from the saved file containing motion observations saved by save_motion_observations.
 *
 * \param    filename    Name of the file containing observations
 * \return   Observations loaded from the file. Will be empty if none are found.
 */
MotionObsMap load_motion_observations(const std::string& filename);

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_TRAINING_AGENT_STATE_H

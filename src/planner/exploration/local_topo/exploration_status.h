/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     exploration_status.h
* \author   Collin Johnson
* 
* Declaration of local_topo_exploration_status_t.
*/

#ifndef PLANNER_EXPLORATION_LOCAL_TOPO_EXPLORATION_STATUS_H
#define PLANNER_EXPLORATION_LOCAL_TOPO_EXPLORATION_STATUS_H

#include <planner/exploration/local_topo/exploration_map.h>
#include <mpepc/metric_planner/task/navigation.h>
#include <system/message_traits.h>

namespace vulcan
{
namespace planner
{

/**
* local_topo_exploration_status_t contains debugging information about the LocalTopoExplorer.
*/
struct local_topo_exploration_status_t
{
    LocalTopoExplorationMap explorationMap;
    hssh::LocalArea::Id currentArea;
    hssh::LocalArea::Id targetArea;
    std::shared_ptr<mpepc::NavigationTask> plannerTask;
};

template <class Archive>
void serialize(Archive& ar, local_topo_exploration_status_t& status)
{
    ar( status.explorationMap,
        status.currentArea,
        status.targetArea,
        status.plannerTask
    );
}

}
}

DEFINE_DEBUG_MESSAGE(planner::local_topo_exploration_status_t, ("DEBUG_LOCAL_TOPO_EXPLORATION_STATUS"))

#endif // PLANNER_EXPLORATION_LOCAL_TOPO_EXPLORATION_STATUS_H

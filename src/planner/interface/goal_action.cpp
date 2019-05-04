/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_action.cpp
* \author   Collin Johnson
* 
* Definition of GoalAction.
*/

#include <planner/interface/goal_action.h>
#include <planner/interface/goal.h>
#include <planner/utils/local_area_tasks.h>
#include <hssh/local_topological/local_topo_map.h>
#include <mpepc/metric_planner/task/navigation.h>
#include <system/system_communicator.h>
#include <utils/stub.h>
#include <cassert>

namespace vulcan
{
namespace planner
{
    
GoalAction::GoalAction(const Goal& goal)
: goal_(goal)
{
    assert(goal.globalPose());
    task_ = std::make_shared<mpepc::NavigationTask>(goal.globalPose().get());
}


GoalAction::GoalAction(const Goal& goal, const hssh::LocalTopoMap& topoMap)
: goal_(goal)
{
    assert(goal.localArea());

    if(auto goalArea = topoMap.areaWithId(goal.localArea().get_value_or(-1)))
    {
        task_ = create_navigation_task_for_local_area(*goalArea);
    }
    else
    {
        std::cerr << "ERROR: GoalAction: Trying to navigate to a localArea not in the current LocalTopoMap.\n";
        assert(topoMap.areaWithId(goal.localArea().get_value_or(-1)));
    }
}


void GoalAction::perform(system::SystemCommunicator& communicator) const
{
    communicator.sendSystem(task_);
}


bool GoalAction::isComplete(const mpepc::metric_planner_status_message_t& status) const
{
    PRINT_STUB("GoalAction::isComplete");
    return false;
}

}
}

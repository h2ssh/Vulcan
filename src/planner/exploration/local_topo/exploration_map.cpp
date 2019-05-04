/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     exploration_map.cpp
* \author   Collin Johnson
* 
* Definition of LocalTopoExplorationMap.
*/

#include <planner/exploration/local_topo/exploration_map.h>
#include <planner/exploration/local_topo/target_impl.h>
#include <hssh/local_topological/local_topo_map.h>
#include <cstdlib>
#include <ctime>

namespace vulcan 
{
namespace planner
{
    
LocalTopoExplorationMap::LocalTopoExplorationMap(const hssh::LocalTopoMap& topoMap)
{
    std::srand(std::time(0));
    
    // Create an area target for each area
    for(auto& area : topoMap)
    {
        std::unique_ptr<LocalAreaTarget> areaTarget(new LocalAreaTarget(*area));
        unvisitedTargets_.push_back(areaTarget.get());
        targets_.push_back(std::move(areaTarget));
    }
}


LocalTopoExplorationMap::~LocalTopoExplorationMap(void)
{
    // For std::unique_ptr
}


LocalAreaTarget* LocalTopoExplorationMap::selectRandomTarget(void)
{
    if(unvisitedTargets_.empty())
    {
        return nullptr;
    }
    
    // Select a random index to return amongst the unvisited targets
    int randIndex = std::rand() % unvisitedTargets_.size();
    return unvisitedTargets_[randIndex];
}


int LocalTopoExplorationMap::identifyVisitedTargets(const hssh::LocalAreaEventVec& events)
{
    int numNewlyVisited = 0;
    
    // Remove any unvisited targets were visited during these events
    for(auto& event : events)
    {
        auto lastIt = std::remove_if(unvisitedTargets_.begin(), 
                                     unvisitedTargets_.end(), 
                                     [&event](LocalAreaTarget* t) {
            return t->checkVisited(*event);
        });
            
        // Add them to the visited targets
        visitedTargets_.insert(visitedTargets_.end(), lastIt, unvisitedTargets_.end());
        
        numNewlyVisited += std::distance(lastIt, unvisitedTargets_.end());
            
        // Erase the unvisited targets
        unvisitedTargets_.erase(lastIt, unvisitedTargets_.end());
    }
    
    return numNewlyVisited;
}
    
} // namespace planner
} // namespace vulcan

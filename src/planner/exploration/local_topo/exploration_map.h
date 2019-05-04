/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     exploration_map.h
* \author   Collin Johnson
* 
* Declaration of LocalTopoExplorationMap.
*/

#ifndef PLANNER_EXPLORATION_LOCAL_TOPO_EXPLORATION_MAP_H
#define PLANNER_EXPLORATION_LOCAL_TOPO_EXPLORATION_MAP_H

#include <planner/exploration/local_topo/target_impl.h>
#include <hssh/local_topological/event.h>
#include <system/message_traits.h>
#include <cereal/access.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace hssh { class LocalTopoMap; }
struct pose_t;
namespace planner
{

/**
* LocalTopoExplorationMap maintains the list of all explored and unexplored areas in the environment. The map provides
* methods for selecting a random unvisited target, as well as iterating over all visited and unvisited targets 
* remaining in the map.
* 
* The exploration order used by LocalTopoExplorationMap is:
* 
*   1) Visit all areas.
* 
* As the robot drives, each topological event results in one or more targets being visited. A target doesn't have to be
* the goal of the robot's navigation in order to be considered visited, it just needs to be visited at some point while
* the robot is driving through the environment.
*/
class LocalTopoExplorationMap
{
public:
    
    using const_iterator = std::vector<LocalAreaTarget*>::const_iterator;

    /**
    * Default constructor for LocalTopoExplorationMap.
    *
    * Create an empty map.
    */
    LocalTopoExplorationMap(void) { }
    
    /**
    * Constructor for LocalTopoExplorationMap.
    * 
    * \param    topoMap         Map from which to extract the exploration map
    */
    LocalTopoExplorationMap(const hssh::LocalTopoMap& topoMap);

    /**
    * Destructor for LocalTopoExplorationMap.
    */
    ~LocalTopoExplorationMap(void);
    
    /**
    * selectRandomTarget selects a random unvisited target in the map.
    * 
    * \return   A randomly selected unvisited target. If no unvisited targets remain, then nullptr is returned.
    */
    LocalAreaTarget* selectRandomTarget(void);
    
    /**
    * identifyVisitedTargets identifies new targets that have been visited by the robot based on new topological events
    * that have been generated.
    * 
    * \param    events          Collection of topological events that occurred
    * \return   Number of new areas that were visited during these events.
    */
    int identifyVisitedTargets(const hssh::LocalAreaEventVec& events);
    
    // Iterator support for the map
    std::size_t sizeVisited(void) const { return visitedTargets_.size(); }
    const_iterator beginVisited(void) const { return visitedTargets_.begin(); }
    const_iterator endVisited(void) const { return visitedTargets_.end(); }
    
    std::size_t sizeUnvisited(void) const { return unvisitedTargets_.size(); }
    const_iterator beginUnvisited(void) const { return unvisitedTargets_.begin(); }
    const_iterator endUnvisited(void) const { return unvisitedTargets_.end(); }
    
private:
    
    std::vector<std::shared_ptr<LocalAreaTarget>> targets_;
    std::vector<LocalAreaTarget*> visitedTargets_;
    std::vector<LocalAreaTarget*> unvisitedTargets_;

    // Serialization support
    friend class cereal::access;
    
    // Split the load and save because need to repopulate the visited/unvisited vectors after loading
    template <class Archive>
    void save(Archive& ar) const
    {
        ar(targets_);
    }
    
    template <class Archive>
    void load(Archive& ar)
    {
        ar(targets_);
        
        for(auto& t : targets_)
        {
            if(t->wasVisited())
            {
                visitedTargets_.push_back(t.get());
            }
            else
            {
                unvisitedTargets_.push_back(t.get());
            }
        }
    }
};

} // namespace planner
} // namespace vulcan

DEFINE_DEBUG_MESSAGE(planner::LocalTopoExplorationMap, ("DEBUG_LOCAL_TOPO_EXPLORATION_MAP"))

#endif // PLANNER_EXPLORATION_LOCAL_TOPO_EXPLORATION_MAP_H

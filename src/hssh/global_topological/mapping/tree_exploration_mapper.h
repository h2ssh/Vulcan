/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     tree_exploration_mapper.h
* \author   Collin Johnson
*
* Definition of TreeExplorationMapper for simplest possible topological mapping.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_TREE_EXPLORATION_MAPPER_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_TREE_EXPLORATION_MAPPER_H

#include <string>
#include "hssh/global_topological/global_topo_data.h"
#include "hssh/global_topological/mapping/topological_mapper.h"

namespace vulcan
{
namespace hssh
{

class  TopologicalMapHypothesis;
struct entered_place_action_t;

const std::string TREE_EXPLORATION_MAPPER_TYPE("tree-exploration");

/**
* TreeExplorationMapper is the simplest topological mapping approach: always add a new
* place and assume the geometry is correct. The mapper assumes no loop closures
* occur, and therefore the geometry of the map is correct.
*/
class TreeExplorationMapper : public TopologicalMapper
{
public:

    /**
    * Constructor for TreeExplorationMapper.
    *
    * \param    manager         Place manager for handling the place models
    */
    TreeExplorationMapper(MetricMapCache& manager);

    /**
    * Destructor for TreeExplorationMapper.
    */
    virtual ~TreeExplorationMapper(void);

    // TopoSlamMapper interface
    void updateMap(const topology_action_t&       action,
                   const topology_measurements_t& measurements);

    TopoMapPtr getUsableHypothesis(void) const;

private:

    // The mapper only cares about place entered events, as that is the only time mapping is necessary
    void handlePlaceEntered(const entered_place_action_t&               action,
                            const topology_measurements_t&              measurements,
                            boost::shared_ptr<TopologicalMapHypothesis> hypothesis);
    void createInitialMap(const entered_place_action_t& action, const topology_measurements_t& measurements);
};

}
}

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_TREE_EXPLORATION_MAPPER_H

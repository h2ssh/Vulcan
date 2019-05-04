/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     map_explorer.cpp
* \author   Collin Johnson
* 
* Definition of create_map_explorer factory function.
*/

#include <planner/exploration/map_explorer.h>
#include <planner/exploration/local_topo/local_topo_explorer.h>
#include <hssh/local_topological/local_topo_map.h>
#include <hssh/local_topological/areas/serialization.h>
#include <utils/serialized_file_io.h>

namespace vulcan
{
namespace planner
{

std::unique_ptr<MapExplorer> create_map_explorer(const std::string& explorerType,
                                                 const std::string& map,
                                                 const utils::ConfigFile& config)
{
    if(explorerType == kLocalTopoExplorerType)
    {
        hssh::LocalTopoMap topoMap;
        if(utils::load_serializable_from_file(map, topoMap))
        {
            local_topo_explorer_params_t params(config);
            return std::unique_ptr<MapExplorer>(new LocalTopoExplorer(topoMap, params));
        }
        else
        {
            std::cerr << "ERROR: create_map_explorer: Unable to open file: " << map << " containing a LocalTopoMap.\n";
        }
    }

    return std::unique_ptr<MapExplorer>();
}

} // namespace planner
} // namespace vulcan

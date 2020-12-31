/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     metric_place_manager.cpp
* \author   Collin Johnson
*
* Definition of MetricMapCache and load_metric_place_manager_params().
*/

#include "hssh/global_topological/utils/metric_map_cache.h"
#include "hssh/local_topological/areas/decision_point.h"
#include "hssh/local_topological/areas/destination.h"
#include "hssh/local_topological/areas/path_segment.h"
#include "hssh/local_topological/events/area_transition.h"
#include "utils/serialized_file_io.h"
#include <boost/filesystem.hpp>

namespace vulcan
{
namespace hssh
{

namespace bfs = boost::filesystem;


Id id_from_directory_entry(const bfs::path& file);


/////   CachedMap implementation   /////
CachedMap::CachedMap(const std::shared_ptr<LocalArea>& area)
: area_(area)
{
}


/////   MetricMapCache implementation   /////
void MetricMapCache::addMap(Id id, CachedMap map)
{
    // Overwrite any previously stored map under the assumption that a new map will be better than old maps
    maps_[id] = std::move(map);
}


const CachedMap* MetricMapCache::loadMap(Id id) const
{
    auto mapIt = maps_.find(id);

    if(mapIt != maps_.end())
    {
        return &mapIt->second;
    }
    else
    {
        return nullptr;
    }
}


bool MetricMapCache::saveMapsToDisk(const std::string& directory, const std::string& basename)
{
    // Ensure the directory in which to save the maps exists
    if(!bfs::create_directories(directory))     // create_directories is like calling mkdir -p
    {
        std::cerr << "ERROR: MetricMapCache: Unable to create directory for storing maps:" << directory << '\n';
        return false;
    }

    for(auto& idToMap : maps_)
    {
        std::ostringstream filename;
        filename << directory << '/' << basename << '_' << idToMap.first << ".lpm";

        if(!utils::save_serializable_to_file(filename.str(), idToMap.second))
        {
            std::cerr << "ERROR: MetricMapCache: Unable to save LPM to file:" << filename.str() << '\n';
            return false;
        }
    }

    return true;
}


int MetricMapCache::loadMapsFromDisk(const std::string& directory, const std::string& basename)
{
    if(!bfs::is_directory(directory))
    {
        std::cerr << "ERROR: MetricMapCache: Cannot load maps from " << directory << ". It isn't a directory!\n";
        return -1;
    }

    int numLoaded = 0;
    CachedMap map;

    for(bfs::directory_entry& file : bfs::directory_iterator(directory))
    {
        auto path = file.path();

        if(!utils::load_serializable_from_file(path.native(), map))
        {
            std::cerr << "ERROR: MetricMapCache: Failed to load file " << path << '\n';
        }
        else
        {
            // Extract the id from the directory entry and save the LPM in the cache
            Id id = id_from_directory_entry(path);
            maps_[id] = std::move(map);
            ++numLoaded;
        }

    }

    return numLoaded;
}


/////   CacheEventVisitor implementation   /////
CacheEventVisitor::CacheEventVisitor(MetricMapCache& cache)
: cache_(cache)
{
}
    
    
void CacheEventVisitor::visitAreaTransition(const AreaTransitionEvent & event)
{
    auto entered = event.enteredArea();
    if(entered)
    {
        cache_.addMap(entered->id(), CachedMap(entered));
    }
    
    auto exited = event.exitedArea();
    if(exited)
    {
        cache_.addMap(exited->id(), CachedMap(exited));
    }
}
    
    
void CacheEventVisitor::visitTurnAround(const TurnAroundEvent & event) 
{
}


Id id_from_directory_entry(const bfs::path& file)
{
    // The stem is the entry minus the extension. Reverse search it for the '_' which divides the basename from
    // the string version of the id.
    std::string stem = file.stem().string();

    auto pos = stem.rfind('_');
    return std::stoll(stem.substr(pos + 1));
}

} // namespace hssh
} // namespace vulcan

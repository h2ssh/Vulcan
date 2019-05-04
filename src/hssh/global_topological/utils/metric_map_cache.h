/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     metric_map_cache.h
* \author   Collin Johnson
*
* Declaration of MetricMapCache, CachedMap, and CacheEventVisitor.
*/

#ifndef HSSH_UTILS_METRIC_MAP_CACHE_H
#define HSSH_UTILS_METRIC_MAP_CACHE_H

#include <hssh/local_topological/area.h>
#include <hssh/local_topological/event_visitor.h>
#include <hssh/utils/id.h>
#include <math/geometry/polygon.h>
#include <cereal/access.hpp>
#include <cereal/types/unordered_map.hpp>
#include <string>

namespace vulcan
{
namespace hssh
{
    
/**
* CachedMap stores information about a cached map:
*   - the LPM representation
*   - the axis-aligned rectangle boundary
*   - the convex hull of the walls in the LPM
* 
* The boundaries are in coordinates relative to the reference frame of the LPM.
*/
class CachedMap
{
public:
    
    CachedMap(void) = default;
    
    /**
    * Constructor for CachedMap.
    * 
    * \param    area            Local area that is being cached
    */
    CachedMap(const std::shared_ptr<LocalArea>& area);
    
    /**
    * rectangleBoundary retrieves the stored rectangle boundary.
    */
    math::Rectangle<double> rectangleBoundary(void) const { return area_->extent().rectangleBoundary(); }
    
    /**
    * polygonBoundary retrieves the stored polygon boundary.
    */
    math::Polygon<double> polygonBoundary(void) const { return area_->extent().polygonBoundary(); }
    
    /**
    * localArea retrieves the local area stored in the map. Use this if you need access to specific features of the
    * underlying area that was saved.
    */
    const LocalArea* localArea(void) const { return area_.get(); }
    
    /**
    * extent retrieves the extent of the cached area.
    */
    const AreaExtent& extent(void) const { return area_->extent(); }
    
private:
    
    std::shared_ptr<LocalArea> area_;
    
    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar( area_ );
    }
};

/**
* MetricMapCache handles loading/storing LocalPerceptualMaps associated with LocalAreas. The map cache allows LPMs
* to be saved to/loaded from somewhere on the disk. It additionally caches the LPMs in memory to allow fast access.
*
* The intended use for the map cache is to store the maps of LocalAreas built by the robot for later recall. Maps are
* cached away in memory for quick access. Maps can be periodically saved to disk as well.
*/
class MetricMapCache
{
public:

    /**
    * addMap adds a new map to the cache.
    *
    * \param    id          Id associated with the map
    * \param    map         Map to be saved.
    */
    void addMap(Id id, CachedMap map);

    /**
    * loadMap attempts to load a map from the cache. If no map exists with the specified id, then a nullptr is
    * returned to indicate this.
    *
    * \param    id          Id of the map to load
    * \return   A pointer to the loaded map. nullptr if no map is associated with the specified id.
    */
    const CachedMap* loadMap(Id id) const;

    /**
    * saveMapsToDisk saves all maps in the cache to disk. The directory and basename for the maps is passed in.
    *
    * The filename for the maps is:
    *
    *   directory/basename_id.lpm
    *
    * If the directory doesn't exist, it will be created.
    *
    * \param    directory           Directory in which to save the maps
    * \param    basename            Basename of the individual LPM files
    * \return   True if all maps were saved successfully.
    */
    bool saveMapsToDisk(const std::string& directory, const std::string& basename);

    /**
    * loadMapsFromDisk loads all maps saved in the specified directory that start with the provided basename.
    *
    * All files in the directory of the format:
    *
    *   directory/basename_id.lpm
    *
    * will be loaded.
    *
    * \param    directory           Directory containing the maps to load
    * \param    basename            Basename of the individual LPM files
    * \return   Number of maps loaded from the disk. -1 on error.
    */
    int loadMapsFromDisk(const std::string& directory, const std::string& basename);

private:

    using IdToMapMap = std::unordered_map<Id, CachedMap>;

    IdToMapMap maps_;
    
    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(maps_);
    }
};


/*
* CacheEventVisitor is a functor for stashing areas in the MetricMapCache.
*/
struct CacheEventVisitor : public LocalAreaEventVisitor
{
    MetricMapCache& cache_;    
    CacheEventVisitor(MetricMapCache& cache);

    /////   LocalAreaEventVisitor interface   /////
    void visitAreaTransition(const AreaTransitionEvent & event);
    void visitTurnAround(const TurnAroundEvent & event);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_UTILS_METRIC_MAP_CACHE_H

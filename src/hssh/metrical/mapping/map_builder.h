/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     map_builder.h
* \author   Collin Johnson
*
* Definition of MapBuilder abstract base class that handles common operations
* for all algorithms for building scrolling local metric maps.
*/

#ifndef HSSH_LOCAL_METRIC_MAPPING_MAP_BUILDER_H
#define HSSH_LOCAL_METRIC_MAPPING_MAP_BUILDER_H

#include <cstdint>

namespace vulcan
{
namespace math  { template <typename T> class Rectangle; }
struct pose_t;
namespace robot { struct velocity_t; }
namespace utils { template <typename T, int N> class TiledCellGrid; }
namespace laser { class MovingLaserScan; }

namespace hssh
{

struct metric_slam_data_t;

typedef utils::TiledCellGrid<int8_t, 8> laser_scan_raster_t;

/**
* map_update_data_t contains all data needed for MapBuilders to update a map.
* If a new MapBuilder needs additional data, just add it here.
*/
struct map_update_data_t
{
    int64_t timestamp;                          ///< Time scan for this update was taken
    bool    initialUpdate;                      ///< Flag indicating if this is the first update for this laser

    const pose_t&          pose;         ///< Pose of the robot when the scan was measured
    const laser::MovingLaserScan& scan;         ///< Polar coordinate version of the laser scan in ROBOT coordinates
    const laser_scan_raster_t&    scanRaster;   ///< A raster of the scan into a discretized grid that is synchronized 
                                                ///< with all map grids

    map_update_data_t(int64_t                       timestamp,
                      bool                          initial,
                      const pose_t&          pose,
                      const laser::MovingLaserScan& scan,
                      const laser_scan_raster_t&    scanRaster)
    : timestamp(timestamp)
    , initialUpdate(initial)
    , pose(pose)
    , scan(scan)
    , scanRaster(scanRaster)
    {
    }
};

/**
* MapBuilder is an abstract base class that handles all common operations for scrolling local metric maps.
* These operations deal with moving and resetting the map, as will occur in the natural course of mapping.
*
* The Map template parameter must support the following methods:
*
*   - reset                : return all map cells to a default state
*   - changeMapBoundary    : shift the metric boundary of the map, adjusting the corresponding map cells as appropriate
*   - setId                : set an identifier for the map, which will be incremented when the map is updated
*
* A MapBuilder subclass must implement the following methods:
*
*   - reset           : reset any necessary internal state when a map is reset
*   - boundaryChanged : indicates the boundary for the map changed, take any necessary action
*   - update          : update the map with new sensor data
*/
template <class Map>
class MapBuilder
{
public:

    MapBuilder(void)
        : mapId(0)
    {
    }

    // No value semantics
    MapBuilder(const MapBuilder& copy)    = delete;
    void operator=(const MapBuilder& rhs) = delete;

    virtual ~MapBuilder(void) { }

    // Accessors
    Map&       getMap(void)       { return getMapInstance(); }
    const Map& getMap(void) const { return getMapInstance(); }

    /**
    * resetMap resets the contents of the map to its default values.
    */
    void resetMap(void)
    {
        getMapInstance().reset();
        reset();
    }
    
    /**
    * setMap sets the map being built to the provided map.
    * 
    * \param    map             Map to be used
    */
    void setMap(const Map& map)
    {
        getMapInstance() = map;
    }

    /**
    * changeMapBoundary shifts the boundary of the map to contain only cell within the
    * new boundary. Any shifted cells not inside the boundary are discarded. Cells not previously
    * in the boundary will be initialized to their default value
    *
    * \param    boundary        Boundary in meters defining the portion of the map
    */
    void changeMapBoundary(const math::Rectangle<float>& boundary)
    {
        getMapInstance().changeBoundary(boundary);
        boundaryChanged(boundary);
    }

    /**
    * updateMap updates the map with new sensor information. See the map_update_data_t description above
    * to see the exact contents of the provided data.
    *
    * After updateMap completes, the fresh map can be accessed via getMap(). updateMap will increment
    * the mapId.
    *
    * \param    data            Data for doing the map update
    */
    void updateMap(const map_update_data_t& data)
    {
        update(data);
        getMapInstance().setId(mapId++);
    }
    
    /**
    * rotateMap rotates the map around its center by the specified number of radians.
    * 
    * \param    radians         Radians to rotate the map
    */
    void rotateMap(float radians)
    {
        rotate(radians);
    }

protected:

    /**
    * reset is called whenever the map state is reset. If any additional state is maintained by
    * the builder, then it should be reset to default values here.
    */
    virtual void reset(void) = 0;

    /**
    * boundaryChanged is called when the map boundary is changed. Any additional book keeping related to
    * the map boundary should be updated accordingly.
    *
    * \param    boundary            New boundary for the map
    */
    virtual void boundaryChanged(const math::Rectangle<float>& boundary) = 0;

    /**
    * update performs the actual map update.
    *
    * \param    data            Data for the map update
    */
    virtual void update(const map_update_data_t& data) = 0;
    
    /**
    * rotate rotates the map by the specified number of radians.
    * 
    * \param    radians         Radians to rotate the map
    */
    virtual void rotate(float radians) = 0;

    /**
    * getMap retrieves the map being built by the MapBuilder subclass.
    */
    virtual Map& getMapInstance(void) = 0;
    virtual const Map& getMapInstance(void) const = 0;

private:

    int32_t mapId;
};

}
}

#endif // HSSH_LOCAL_METRIC_MAPPING_MAP_BUILDER_H

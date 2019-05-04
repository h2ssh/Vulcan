/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     location.h
* \author   Collin Johnson
*
* Declaration of LocalLocation.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_LOCATION_H
#define HSSH_LOCAL_TOPOLOGICAL_LOCATION_H

#include <hssh/local_topological/gateway.h>
#include <system/message_traits.h>
#include <cereal/access.hpp>

namespace vulcan
{
namespace hssh
{

/**
* LocalLocation represents the robot's current location with the LocalTopoMap. The location of the robot is described
* by the id of the area it is in and the gateway it crossed to enter the area (if there is one). From this location
* information, further state can be acquired by querying the relevant area in the LocalTopoMap.
*/
class LocalLocation
{
public:
    
    /**
    * Constructor for LocalLocation.
    * 
    * The usual location for the robot, where it has entered from some other area.
    * 
    * \param    mapId           Id of the map with which this map is associated
    * \param    areaId          Id of the area entered
    * \param    entryGateway    Gateway through which the robot entered the area
    */
    LocalLocation(int32_t mapId, int32_t areaId, const Gateway& entryGateway)
    : mapId_(mapId)
    , areaId_(areaId)
    , entry_(entryGateway)
    , isInitialArea_(false)
    {
    }
    
    /**
    * Constructor for LocalLocation.
    * 
    * The special location associated with the robot's initial area.
    * 
    * \param    mapId           Id of the map with which this map is associated
    * \param    areaId          Id of the area entered
    */
    LocalLocation(int32_t mapId, int32_t areaId)
    : mapId_(mapId)
    , areaId_(areaId)
    , isInitialArea_(true)
    {
    }
    
    /**
    * Default constructor for LocalLocation.
    * 
    * Creates a location not associated with any map.
    */
    LocalLocation(void)
    : mapId_(-1)
    , areaId_(-1)
    {
    }
    
    // Accessors for the LocalLocation
    /**
    * mapId retrieves the id of the map with which this location is associated.
    */
    int32_t mapId(void) const { return mapId_; }
    
    /**
    * areaId retrieves the id of the area within that map that the robot is currently in.
    */
    int32_t areaId(void) const { return areaId_; }
    
    /**
    * entryGateway retrieves the gateway through which the robot entered its current area location.
    */
    Gateway entryGateway(void) const { return entry_; }
    
    /**
    * isInitialArea checks if this location corresponds to the robot's intial area encountered in the environment. If 
    * so, then there is no valid entry gateway.
    */
    bool isInitialArea(void) const { return isInitialArea_; }

    // Operators
    /**
    * Two LocalLocations are the same if they belong to the same map, have the same area id, and the same entry gateway.
    */
    bool operator==(const LocalLocation& rhs) const
    {
        return (mapId_ == rhs.mapId_) && (areaId_ == rhs.areaId_)
            && ((entry_ == rhs.entry_) || (isInitialArea_ && rhs.isInitialArea_));
    }

    bool operator!=(const LocalLocation& rhs) const
    {
        return !(*this == rhs);
    }

private:
    
    int32_t mapId_;
    int32_t areaId_;
    Gateway entry_;
    bool    isInitialArea_;

    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( mapId_,
            areaId_,
            entry_,
            isInitialArea_);
    }
};

} // namespace hssh
} // namespace vulcan

DEFINE_SYSTEM_MESSAGE(hssh::LocalLocation, ("LOCAL_TOPO_LOCATION"))

#endif // HSSH_LOCAL_TOPOLOGICAL_LOCATION_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     map_explorer.h
* \author   Collin Johnson
* 
* Declaration of MapExplorer interface and create_map_explorer factory function.
*/

#ifndef PLANNER_EXPLORATION_MAP_EXPLORER_H
#define PLANNER_EXPLORATION_MAP_EXPLORER_H

#include <memory>
#include <string>

namespace vulcan
{
namespace system { class ModuleCommunicator; }
namespace utils { class ConfigFile; }
namespace planner
{
  
class MapExplorer;
    
/**
* create_map_explorer creates a new MapExplorer of the given type for the given map using parameters specified in the
* config file.
*
* \param    explorerType    Type of explorer to create
* \param    map             Map to be explored (depends on explorer type)
* \param    config          Config file with additional parameters
* \return   An instance of MapExplorer.
*/
std::unique_ptr<MapExplorer> create_map_explorer(const std::string& explorerType,
                                                 const std::string& map,
                                                 const utils::ConfigFile& config);

/**
* MapExplorer is the interface for different exploration approaches. An instance of MapExplorer will have the following
* usage pattern:
* 
*   - On creation:
*       * subscribeToData : establish necessary incoming messages
* 
*   - On start:
*       * startExploring : handle any necessary startup
* 
*   - During exploration:
*       * hasNewData          : check if the data needed for updating the exploration has arrived
*       * isFinishedExploring : first see if exploration is completed
*       * continueExploring   : if not done, then issue whatever commands are needed to get closer to completion
*       * stopExploring       : map exploration is halting, so cleanup the exploration state
* 
*   - On destruction:
*       * unsubscribeFromData : unsubscribe from all messages
*/
class MapExplorer
{
public:

    /**
    * subscribeToData subscribes the MapExplorer to any messages that will be needed for performing the exploration.
    * 
    * \param    communicator        Communicator instance that will deliver the data
    */
    virtual void subscribeToData(system::ModuleCommunicator& communicator) = 0;
    
    /**
    * unsubscribeFromData unsubscribes the MapExplorer from all messages subscribed to in subscribeToData
    * 
    * \param    communicator        Communicator instance that was delivering messages
    */
    virtual void unsubscribeFromData(system::ModuleCommunicator& communicator) = 0;
    
    /**
    * hasNewData checks if new data has arrived and an new update step can proceed.
    * 
    * \return   True if data has arrived and an exploration round can proceed.
    */
    virtual bool hasNewData(void) = 0;
    
    /**
    * isFinishedExploring checks if the exploration of the map has been completed.
    */
    virtual bool isFinishedExploring(void) const = 0;
    
    /**
    * startExploring initializes the exploration process for the map.
    * 
    * \param    communicator            Communicator to use for sending out the necessary messages
    */
    virtual void startExploring(system::ModuleCommunicator& communicator) = 0;
    
    /**
    * continueExploring performs an update of the exploration using newly arrived exploration data.
    * 
    * \param    communicator            Communicator to use for sending out the necessary messages
    */
    virtual void continueExploring(system::ModuleCommunicator& communicator) = 0;
    
    /**
    * stopExploring stops the exploration. When exploration is finished this method is called. It will also be called
    * if the explorer is still running when the module is shutdown.
    * 
    * \param    communicator            Communicator to use for sending out the necessary messages
    */
    virtual void stopExploring(system::ModuleCommunicator& communicator) = 0;
};

} // namespace planner
} // namespace vulcan

#endif // PLANNER_EXPLORATION_MAP_EXPLORER_H

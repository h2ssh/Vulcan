/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topological_map_editor.h
* \author   Collin Johnson
*
* Declaration of TopologicalMapEditor.
*/

#ifndef HSSH_UTILS_TOPOLGOICAL_MAP_EDITOR_H
#define HSSH_UTILS_TOPOLGOICAL_MAP_EDITOR_H

#include <string>
#include <vector>
#include <hssh/global_topological/topological_map_hypothesis.h>

namespace vulcan
{
namespace hssh
{

class LocalPlace;
class MetricMapCache;

/**
* TopologicalMapEditor provides a means of incrementally constructing a topological
* map independent of the other layers of the HSSH. The editor is intended to be used
* for building maps from a set of local places in the real world to create more complex
* synthetic environments for testing large-scale mapping without having to create an
* hours-long dataset.
*
* Constructing a topological map consists of performing the following operations in
* some order:
*
*   0) Either create a new map or load a previously saved map.
*   1) Add new places to the map.
*   2) Connect places in the map.
*   3) Save the map to disk for later use.
*
* New places are not immediately added to the topological map being built because they
* first need to be connected to an existing place in the map.
*
* The map editor supports undoing or redoing some number of map building steps as specified
* in the constructor.
*
* Editing functionality is currently limited to adding new places to the map or connecting
* places within the map. Places cannot be unconnected once in the map. Places that have been
* added but not connected can, however, be deleted. Perhaps in the future, full editing
* of an existing map will be supported.
*/
class TopologicalMapEditor
{
public:

    /**
    * Constructor for TopologicalMapEditor.
    *
    * \param    manager         Manager for handling the LocalPlaces
    * \param    undoSteps       Number of undo steps to maintain
    */
    TopologicalMapEditor(MetricMapCache& manager, size_t undoSteps);

    /**
    * loadInitialMap loads the specified map from a file. If no map with the specified filename
    * exists, then the an empty map will be created.
    *
    * \param    filename            Name of the file from which to load the map
    * \return   True if the map was loaded. False if no such map exists.
    */
    bool loadInitialMap(const std::string& filename);

    /**
    * createNewMap creates a new, empty topological map.
    */
    void createNewMap(void);

    /**
    * saveMap saves the map to the specified file.
    *
    * \param    filename            Name of file in which to save the map
    * \return   True if able to save map. False if the operation could not be completed.
    */
    bool saveMap(const std::string& filename);

    /**
    * addPlace adds a new place to the map. The place is specified with the transform to apply
    * to the map center when adding to the map. Thus, the center of the map will exist at
    * (transform.x, transform.y). All components of the map will be rotated by transform.theta.
    *
    * \param    place           Place to be added
    * \param    transform       Transform to apply to the place
    * \return   Id assigned to the newly created place.
    */
    uint32_t addPlace(const LocalPlace& place, const pose_t& transform);

    /**
    * deletePlace deletes a place added to the map as long as it has not been connected.
    *
    * Deleting a place is not an operation that can currently be undone.
    *
    * \param    id              Id of the place to be deleted
    * \return   True if the place could be deleted. False otherwise.
    */
    bool deletePlace(uint32_t id);

    /**
    * connectPlaces connects two places in the map. To connect two places, the global place id
    * and the global path fragment of each place are specified. The two places are then connected
    * by creating a GlobalPathSegment between the places and adding it to the GlobalPath.
    *
    * \param    placeId             Id of one of the places to be connected
    * \param    otherPlaceId        Id of the other place to be connected
    * \param    fragmentId          Id of the fragment associated with placeId
    * \param    otherFragmentId     Id of the fragment associated with otherPlaceId
    * \return   True if the places could be connected. False if otherwise, i.e. the fragments weren't frontiers.
    */
    bool connectPlaces(uint32_t placeId, uint32_t otherPlaceId, uint32_t fragmentId, uint32_t otherFragmentId);

    /**
    * undo reverts the last place connection made in the map. undo only deals with connections. To delete
    * an unconnected place, call deletePlace().
    *
    * \return   True if able to undo. False if there is no more history to undo.
    */
    bool undo(void);

    /**
    * redo reverts the previous undo operation. A redo only works if no actions have been performed since the last undo.
    *
    * \return   True if the redo operation was successful. False if there was nothing to redo.
    */
    bool redo(void);

    /**
    * getPlaces retrieves all places in the map, both those connected and unconnected.
    *
    * \return   Vector of places in the map.
    */
    const std::vector<GlobalPlace>& getPlaces(void) const;

    /**
    * getPaths retrieves all paths in the map.
    *
    * \return   Paths in the map.
    */
    const std::vector<GlobalPath>& getPaths(void) const;

private:

    std::map<uint32_t, GlobalPlace> unattachedPlaces;            // Places not yet connected into a map

    std::deque<TopoMapPtr> mapHistory;
    size_t                 mapIndex;
    size_t                 maxHistorySize;

    uint32_t nextPlaceId;
    uint32_t nextPathId;

    MetricMapCache& manager;
};

}
}

#endif // HSSH_UTILS_TOPOLGOICAL_MAP_EDITOR_H

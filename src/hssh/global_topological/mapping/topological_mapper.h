/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topological_mapper.h
* \author   Collin Johnson
*
* Declaration of TopologicalMapper interface and create_topological_mapper() factory function.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_TOPOLOGICAL_MAPPER_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_TOPOLOGICAL_MAPPER_H

#include <memory>
#include <string>
#include "hssh/global_topological/mapping/tree_of_maps.h"

namespace vulcan
{
namespace hssh
{

class  LocalPlace;
class  MetricMapCache;
class  TopologicalMapper;
struct correct_global_topo_map_message_t;
struct set_global_location_message_t;
struct topology_action_t;
struct topology_measurements_t;
struct topological_mapper_params_t;

/**
* create_topological_mapper is a factory for creating instances of TopoSlamMap. The
* mapperType specifies the detailed type to be created. The params is a generic
* mapper params struct parsed to contain the necessary specific parameters.
*
* NOTE: If mapperType is invalid, create_topological_mapper() will assert and fail immediately.
*
* \param    mapperType          Type of mapper to be created
* \param    params              Parameters to use for the mapper
* \param    manager             Place manager for loading LocalPlaces stored in the map
* \return   Instance of TopologicalMapper.
*/
std::unique_ptr<TopologicalMapper> create_topological_mapper(const std::string&                 mapperType,
                                                             const topological_mapper_params_t& params,
                                                             MetricMapCache&                manager);

/**
* TopologicalMapper is an abstract base class for implementations of topological mapping. The mapper
* is responsible for adding new places to the map, as determined when a current map hypothesis
* location is a new map. The mapper handles pruning the map when geometric constraints fail.
*/
class TopologicalMapper
{
public:

    /**
    * Constructor for TopologicalMapper.
    *
    * \param    manager     Place manager that stores the LocalPlace representations
    */
    TopologicalMapper(MetricMapCache& manager);

    virtual ~TopologicalMapper(void) {}

    /**
    * setCorrectMap sets the correct map consistent with current data in the tree of maps.
    *
    * \param    id              Id of the correct map hypothesis
    */
    virtual void setCorrectMap(uint32_t id);

    /**
    * setCorrectMap sets the correct map to be used for mapping as received from some
    * external source, rather than one of the maps from the current exploration.
    *
    * \param    map             Map to set as the correct map
    */
    virtual void setCorrectMap(TopoMapPtr map);

    /**
    * setGlobalLocationInMap sets the global location of the robot within a map hypothesis.
    *
    * \param    message         Message containing information about the location and map
    */
    void setGlobalLocationInMap(const set_global_location_message_t& message);

    /**
    * updateMap updates the topological map based on a new topology action and measurement.
    *
    * \param    data            Data to be used for the update
    * \param    map             Map to be modified
    */
    virtual void updateMap(const topology_action_t&       action,
                           const topology_measurements_t& measurements) = 0;

    /**
    * getUsableHypothesis retrieves a map hypothesis suitable for further use in planning and navigation.
    * This hypothesis should represent a hypothesis that will hopefully be consistent over time, given
    * the current actions and measurements gathered by the robot.
    *
    * \return   A TopologicalMapHypothesis that can be used by other modules.
    */
    virtual TopoMapPtr getUsableHypothesis(void) const = 0;

    /**
    * getTreeOfMaps retrieves the currently maintained TreeOfMaps with all generated hypotheses.
    *
    * \return   TreeOfMaps being built by the mapper.
    */
    const TreeOfMaps& getTreeOfMaps(void) const { return tree; }

protected:

    /**
    * atFrontierPlace uses the GlobalLocation of the provided map hypothesis to determine if the
    * robot is currently at a frontier place or a known place.
    *
    * \param    hypothesis          Hypothesis to check for frontier
    * \return   True if at a frontier. False if on a path or at a known place.
    */
    bool atFrontierPlace(const TopoMapPtr& hypothesis);

    /**
    * validLoopClosure determines if a proposed loop closure, where the path goes from start to end
    * is valid. A valid loop closure involves two places or two different path fragments within the
    * same place.
    *
    *   (start.placeId != end.placeId) || (start.entryFragment != end.entryFragment)
    *
    * \param    start       Start place of the path segment creating a loop closure
    * \param    end         End place of the path segment for the loop closure
    * \return   True if valid. False otherwise.
    */
    bool validLoopClosure(const place_connection_t& start, const place_connection_t& end);

    /**
    * initializeTree initializes the tree of maps with the initially detected place.
    *
    * \param    action          Initial place action
    * \param    measurements    Initial place measurements
    */
    void initializeTree(const topology_action_t& action, const topology_measurements_t& measurements);

    /**
    * moveAlongKnownPath has the robot move along a known path to a known destination. The lambda
    * measurement will be updated for the traversed segment.
    *
    * \param    star            LargeScaleStar of the new place
    * \param    lambda          Measured lambda between the previous place and this new place
    * \param    localPlaceId    Id of the LocalPlace for which a GlobalPlace is being constructed
    * \param    hypothesis      Parent hypothesis that is being extended with the new place
    * \return   The new map hypothesis with the path segment updated and the robot at the next place on the path.
    */
    TopoMapPtr moveAlongKnownPath(const LargeScaleStar& star,
                                  const Lambda&         lambda,
                                  uint32_t              localPlaceId,
                                  TopoMapPtr            hypothesis);

    /**
    * createNewPlaceHypothesis extends the provided map hypothesis with the hypothesis that
    * the robot has just arrived at a new place. A new hypothesis will be created where the
    * new place with the given properties will be the current location of the robot.
    *
    * The returned hypothesis will have its placeState updated to reflect being at the new place.
    *
    * \param    star            LargeScaleStar of the new place
    * \param    lambda          Measured lambda between the previous place and this new place
    * \param    localPlaceId    Id of the LocalPlace for which a GlobalPlace is being constructed
    * \param    hypothesis      Parent hypothesis that is being extended with the new place
    * \return   The new map hypothesis with the place added.
    */
    virtual TopoMapPtr createNewPlaceHypothesis(const LargeScaleStar& star,
                                                const Lambda&         lambda,
                                                uint32_t              localPlaceId,
                                                TopoMapPtr            hypothesis);

    /**
    * createLoopClosureHypotheses creates all possible loop closure hypotheses between two existing
    * places in a map hypothesis based on the calculated topology of the most recently entered place.
    *
    * A vector of generated map hypotheses will be output, a new hypothesis for each loop closure.
    *
    * \param    star            LargeScaleStar of the observed place
    * \param    lambda          Measured lambda from the previous place
    * \param    localPlaceId    Id of the LocalPlace which will be matched to the stored place for each loop closure hypothesis
    * \param    hypothesis      Map hypothesis to be extended with new loop closures
    * \return   A vector of map hypotheses representing all of discovered possible loop closures.
    */
    virtual std::vector<TopoMapPtr> createLoopClosureHypotheses(const LargeScaleStar& star,
                                                                const Lambda&         lambda,
                                                                uint32_t              localPlaceId,
                                                                TopoMapPtr            hypothesis);

    TreeOfMaps          tree;
    MetricMapCache& manager;

private:

    pose_t findTransformBetweenPlaces(const GlobalPlace& currentPlace, const GlobalPlace& referencePlace, int referenceId);
};

}
}

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_TOPOLOGICAL_MAPPER_H

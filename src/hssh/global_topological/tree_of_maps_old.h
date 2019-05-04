/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     tree_of_maps.h
* \author   Collin Johnson
*
* Declaration of TreeOfMaps.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_TREE_OF_MAPS_H
#define HSSH_GLOBAL_TOPOLOGICAL_TREE_OF_MAPS_H

#include <hssh/global_topological/topological_map_hypothesis.h>
#include <boost/shared_ptr.hpp>
#include <set>

namespace vulcan
{
namespace hssh
{

class  HypothesisTree;
struct hypothesis_tree_node_t;

/**
* TreeOfMaps maintains the current set of valid hypotheses for the topological map of
* the environment. The valid hypotheses are each described by a TopologicalMapHypothesis.
*
* The hypotheses within the TreeOfMaps can be accessed in two ways: through the leaves or by
* depth in the tree. The tree maintains the current set of leaves across all depths of the tree.
* Each hypotheses for each depth of the tree are also maintained.
*
* The tree can be modified in the following ways:
*
*   - createInitialHypothesis : create the root hypothesis for the tree.
*   - addNewPlace             : add a new place to the given map, generating a new child hypothesis
*   - connectPlaces           : assert two places in a map to be equal, creating a new child hypothesis
*                               with the given loop closure hypothesis.
*   - copyMapToNextDepth      : if navigating in a known part of the map, no places are added or loop
*                               closures created, but new measurements are made. Thus, the map needs to
*                               be updated to the next depth of the tree to indicate which actions and
*                               measurements it is current with.
*   - prune                   : remove an inconsistent hypothesis from the tree. Pruning will continue
*                               recursively until a parent hypothesis with more than the single pruned
*                               child exists.
*/
class TreeOfMapsOld
{
public:

    /**
    * Constructor for TreeOfMaps.
    */
    TreeOfMaps(void);

    /**
    * setRootHypothesis resets the tree by setting the root to be the map with the provided id. All other hypotheses
    * will be discarded. If the map with id is not a leaf, even its children will be tossed aside. How rude!
    *
    * \param    id          Id of the map to be the root hypothesis
    * \return   True if a map with this id exists. False otherwise.
    */
    bool setRootHypothesis(uint32_t id);

    /**
    * setRootHypothesis resets the tree by specifying a new TopologicalMapHypothesis to serve as the root. All
    * hypotheses in the tree are discarded and only this single map exists.
    *
    * \param    root          Hypothesis to use as the root of the modified tree.
    */
    void setRootHypothesis(TopoMapPtr root);

    /**
    * getHeight retrieves the height of the tree. The tree height is equal to the number of
    * place events that have occurred during the mapping process.
    *
    * An uninitialized tree has a height of 0.
    *
    * \return   Height of the tree.
    */
    uint32_t getHeight(void) const { return depths.size(); }

    /**
    * getMapFromId retrieves the map with the specified id. If no such map exists, a null pointer is returned.
    *
    * \param    id          Id of the map to retrieve
    * \return   Map with the desired id or a null pointer if no map exists.
    */
    TopoMapPtr getMapFromId(uint32_t id) const;

    /**
    * getLeaves retrieves the current leaves of the tree across all depths of the tree.
    *
    * \return   The leaves of the tree.
    */
    const std::set<TopoMapPtr>& getLeaves(void) const;// { return leaves; }

    /**
    * getCompleteHypotheses retrieves all hypotheses that are considered complete, i.e. the leaves
    * at the very bottom of the tree.
    *
    * \pre      getHeight() > 0 -- the tree has been initialized
    * \return   The set of complete hypotheses.
    */
    const std::set<TopoMapPtr>& getCompleteHypotheses(void) const { return depths[getHeight()-1]; }

    /**
    * getMapsAtDepth retrieves all the maps that exist at the specified depth of the tree.
    *
    * The depth of the root is 0. The most recent hypotheses are depth height-1.
    *
    * \pre      depth < height of the tree
    * \param    depth           Depth of tree to retrieve
    * \return   Map hypotheses consistent with the given depth of the tree.
    */
    const std::set<TopoMapPtr>& getMapsAtDepth(uint32_t depth) const { return depths[depth]; }

    /**
    * getHypothesisTree creates a HypothesisTree representing the current set of nodes in the tree of maps.
    * See the description of HypothesisTree for details on the contents of the tree.
    *
    * \return   HypothesisTree representation of the TreeOfMaps.
    */
    HypothesisTree getHypothesisTree(void) const;

    /**
    * setMeasurementLikelihoodDescriptions sets a text description of the measurement likelihood terms used in the probability
    * of the TopologicalMapHypotheses. The text description will be attached to HypothesisTrees generated from the TreeOfMaps.
    *
    * \param    descriptions            Descriptions of the measurement likelihoods
    */
    void setMeasurementLikelihoodDescriptions(const std::vector<std::string>& descriptions);

    /**
    * createInitialHypothesis creates a new hypothesis with a single place.
    *
    * \pre      The tree is empty. getHeight == 0.
    * \param    initialPlace            Initial place with which to initialize the map
    */
    void createInitialHypothesis(const GlobalPlace& initialPlace);

    /**
    * addNewPlace adds a new place to a map hypothesis.
    *
    * \param    map                     Map hypothesis to be augmented with a new place
    * \param    newPlace                New place to be added (id of the place is modified to reflect assignment in the returned map hypothesis)
    * \param    lambda                  Lambda value giving measured displacement from previous place
    * \return   New hypothesis map generated by adding a new place to a previous hypothesis.
    */
    TopoMapPtr addNewPlace(const TopoMapPtr& map,
                           GlobalPlace&      newPlace,
                           int               entrySegmentId,
                           const Lambda&     lambda);

    /**
    * connectPlaces connects two places within a map hypotheses.
    *
    * \param    map                     Map hypothesis in which to connect the two places
    * \param    previousPlace           Place previously visited by robot
    * \param    currentPlace            Place in which the robot is currently located
    * \param    lambda                  Lambda value giving measured displacement between the places
    * \param    transform               Measured transformation of local frame into the currentPlace reference frame
    * \return   New hypothesis map generated by connecting the previous and current places.
    */
    TopoMapPtr connectPlaces(const TopoMapPtr&         map,
                             const place_connection_t& previousPlace,
                             const place_connection_t& currentPlace,
                             const Lambda&             lambda,
                             const pose_t&      transform);

    /**
    * copyMapToNextDepth takes a current map hypothesis and copies it on to the next depth in the tree.
    * Copying a map is intended for navigation within a known part of the tree. New measurements are
    * incorporated into the hypothesis, but otherwise it will remain the same. Right now, a deep copy
    * of the hypothesis is made. Perhaps simply propagating the hypothesis along is a better choice though.
    *
    * \param    map                     Map hypothesis to be copied
    * \param    lambda                  Measured lambda to the next place
    * \return   New map hypothesis at the next depth in the tree.
    */
    TopoMapPtr copyMapToNextDepth(const TopoMapPtr& map, const Lambda& lambda);

    /**
    * prune removes the hypothesis from the set of map hypotheses.
    *
    * \param    toPrune                 Map to be pruned from the set of valid hypotheses
    * \return   Lowest depth in the tree in which a hypothesis was pruned.
    */
    uint32_t prune(TopoMapPtr toPrune);

private:

    TreeOfMaps(const TreeOfMaps& map)            = delete;
    TreeOfMaps& operator=(const TreeOfMaps& map) = delete;

    void buildHypothesisTree(const TopoMapPtr& hypothesis, std::vector<hypothesis_tree_node_t>& hypNodes) const;

    bool isValidParent(const boost::shared_ptr<TopologicalMapHypothesis>& parent);
    void addChildToMap(const TopoMapPtr& map, const TopoMapPtr& child);

    void validateHypothesis(TopoMapPtr& map);

    std::set<TopoMapPtr>               leaves;          // current leaves across all depths
    std::vector<std::set<TopoMapPtr>> depths;          // set of nodes for each depth of the tree. index in vector = depth
    std::map<uint32_t, TopoMapPtr>     idToMap;         // map the id->map for easy access

    std::vector<std::string> probabilityDescriptions;

    uint32_t nextHypothesisId;
};

}
}

#endif // HSSH_GLOBAL_TOPOLOGICAL_TREE_OF_MAPS_H

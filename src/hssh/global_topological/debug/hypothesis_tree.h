/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_tree.h
* \author   Collin Johnson
*
* Declaration of HypothesisTree.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_DEBUG_HYPOTHESIS_TREE_H
#define HSSH_GLOBAL_TOPOLOGICAL_DEBUG_HYPOTHESIS_TREE_H

#include "hssh/global_topological/map_probability.h"
#include "hssh/utils/id.h"
#include "system/message_traits.h"
#include <cereal/types/string.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace hssh
{

/**
* hypothesis_tree_node_t represents a single node within the hypothesis tree.
* A node contains:
*
*   - the id of the corresponding TopologicalMapHypothesis
*   - a list of probabilities calculated for the node (see HypothesisTree for how to distinguish them)
*   - a list of child hypotheses
*/
struct hypothesis_tree_node_t
{
    Id id;
    int depth;
    std::vector<Id> children;
    TopoMapProbability probability;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar (id,
            depth,
            children,
            probability
        );
    }
};

/**
* HypothesisTree provides an inheritance tree for the TopologicalMapHypotheses. The tree contains a node for
* each TopologicalMapHypothesis in the tree of maps. Thus, the hypothesis tree can be used to see where the tree
* is being expanded. Each node contains the full set of calculated probabilities as well. Thus, the development
* of probabilities within the tree can be found as well. The nodes only contain a list of the probabilities. The
* hypothesis tree contains a text description for each of these probabilities.
*/
class HypothesisTree
{
public:

    /**
    * Default constructor for HypothesisTree.
    *
    * Create an empty tree.
    */
    HypothesisTree(void) = default;

    /**
    * Constructor for HypothesisTree.
    *
    * Create a HypothesisTree from the descriptions and nodes.
    *
    * \param    nodes               Nodes for the tree
    * \param    rootId              Id of the root node
    * \param    numLeaves           Number of leaf nodes in the tree
    * \param    numComplete         Number of complete nodes in the tree
    */
    HypothesisTree(const std::vector<hypothesis_tree_node_t>& nodes,
                   Id rootId,
                   std::size_t numLeaves,
                   std::size_t numComplete)
    : nodes_(nodes)
    , root_(rootId)
    , numLeafNodes_(numLeaves)
    , numCompleteNodes_(numComplete)
    {
        for(std::size_t n = 0; n < nodes_.size(); ++n)
        {
            idToIndex_[nodes_[n].id] = n;
        }
    }

    /**
    * treeSize retrieves the number of nodes in the tree.
    */
    std::size_t treeSize(void) const { return nodes_.size(); }

    /**
    * numLeafNodes retrieves the number of nodes in the HypothesisTree that are leaves
    * of the the tree of maps.
    */
    std::size_t numLeafNodes(void) const { return numLeafNodes_; }

    /**
    * numCompleteNodes retrieves the number of complete nodes in the HypothesisTree. Complete nodes are those with
    * depth == depth of the overall tree.
    */
    std::size_t numCompleteNodes(void) const { return numCompleteNodes_; }

    /**
    * getRootNode retrieves the root of the tree.
    *
    * \pre  tree size > 0
    * \return   The root node of the tree. nullptr if an empty tree.
    */
    const hypothesis_tree_node_t* getRootNode(void) const { return getNode(root_); }

    /**
    * getNode retrieves the node with the specified id.
    *
    * \param    id          Id of the node in the tree to retrieve
    * \return   The requested node if it exists. nullptr if no such node exists.
    */
    const hypothesis_tree_node_t* getNode(Id id) const
    {
        auto indexIt = idToIndex_.find(id);
        if(indexIt != idToIndex_.end())
        {
            return &nodes_[indexIt->second];
        }

        return nullptr;
    }

    /**
    * getAllNodes retrieves all the nodes contained in the HypothesisTree.
    *
    * \return   The nodes in the tree.
    */
    const std::vector<hypothesis_tree_node_t>& getAllNodes(void) const { return nodes_; }

private:

    std::vector<hypothesis_tree_node_t> nodes_;
    Id root_;
    std::size_t numLeafNodes_;
    std::size_t numCompleteNodes_;
    std::unordered_map<Id, std::size_t> idToIndex_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar (nodes_,
            root_,
            numLeafNodes_,
            numCompleteNodes_,
            idToIndex_
        );
    }
};

} // namespace hssh
} // namespace vulcan

DEFINE_DEBUG_MESSAGE(hssh::HypothesisTree, ("DEBUG_GLOBAL_TOPO_HYPOTHESIS_TREE"))

#endif // HSSH_GLOBAL_TOPOLOGICAL_DEBUG_HYPOTHESIS_TREE_H

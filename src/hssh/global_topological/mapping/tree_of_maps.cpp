/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     tree_of_maps.cpp
* \author   Collin Johnson
*
* Definition of TreeOfMaps.
*/

#include "hssh/global_topological/mapping/tree_of_maps.h"
#include "hssh/global_topological/mapping/probability_heuristics.h"
#include "hssh/global_topological/debug/hypothesis_tree.h"
#include "utils/algorithm_ext.h"
#include "utils/stub.h"
#include <boost/range/adaptor/map.hpp>

namespace vulcan
{
namespace hssh
{

bool TreeOfMaps::addRootState(StatePtr&& root)
{
    // Can't change the root if any states exist
    if(!states_.empty())
    {
        return false;
    }

    auto rootId = root->id;
    auto success = states_.insert(std::make_pair(rootId, TreeNode(rootId, 0, -1, std::move(root))));

    // If the state was inserted, then increment the id and create a new depth.
    if(success.second)
    {
        depth_ = 0;
        rootId_ = rootId;

        const auto& rootNode = success.first->second;

        // The root starts as complete and a leaf
        leaves_.push_back(rootNode.state.get());
        complete_.push_back(rootNode.state.get());
    }
    else
    {
        std::cerr << "ERROR: TreeOfMaps: Failed to add the root state!\n";
    }

    return success.second;
}


bool TreeOfMaps::addChildState(Id parentId, StatePtr&& child)
{
    // Find the parent node
    auto node = nodeWithId(parentId);

    // If the state doesn't exist, fail away
    if(!node)
    {
        std::cout << "Failed to find parent!\n";
        return false;
    }

    // Create a node for the child state
    auto childId = child->id;
    auto success = states_.insert(std::make_pair(childId, TreeNode(childId,
                                                                   node->depth + 1,
                                                                   parentId,
                                                                   std::move(child))));

    // If the state couldn't be created, then fail.
    if(!success.second)
    {
        std::cout << "Failed to create child tree node.\n";
        return false;
    }

    const auto& childNode = success.first->second;

    // If the parent is no longer a leaf, then remove it from the set of leaves
    if(node->childIds.empty() && !node->hadChildren)
    {
        int numRemoved = utils::erase_remove(leaves_, node->state.get());
        assert(numRemoved == 1);    // the node must be a leaf, so ensure the variant held
    }

    // Save the parent->child link
    node->childIds.push_back(childId);
    node->hadChildren = true;

    leaves_.push_back(childNode.state.get());

    // If this child is deeper than previous complete nodes, update the depth.
    if(depth_ < childNode.depth)
    {
        depth_ = childNode.depth;
        // All existing complete hypotheses are no longer complete
        complete_.clear();
        // The new node is now the only complete node
        complete_.push_back(childNode.state.get());
    }
    else if(childNode.depth == depth_)
    {
        complete_.push_back(childNode.state.get());
    }

    return true;
}


bool TreeOfMaps::changeRootState(Id id)
{
    // Find the new root state
    // If the desired root doesn't exist, then fail
    // Copy all children of the new root to a new states_ map
    // Set the parent of the new root to invalid id.
    // Reassign the existing states.
    // Iterate through the remaining states to identify the leaves and complete hypotheses
    // Adjust the depth of all nodes in the new tree.

    auto newRoot = nodeWithId(id);

    if(!newRoot)
    {
        return false;
    }

    States newStates;
    preorderTraversal(newRoot, [&newStates](TreeNode& node) {
        newStates[node.id] = std::move(node);   // WARNING: states_ is invalid as soon as this line runs!
    });

    states_ = std::move(newStates);
    newRoot = nodeWithId(id);   // newRoot invalidated when constructing newStates, so reassign
    assert(newRoot);
    newRoot->parentId = kInvalidId;
    rootId_ = newRoot->id;

    int depthChange = newRoot->depth;
    depth_ -= depthChange;

    for(auto& node : boost::adaptors::values(states_))
    {
        node.depth -= depthChange;
    }

    findLeaves();
    return true;
}


int TreeOfMaps::numChildren(Id id) const
{
    auto node = nodeWithId(id);
    return node ? node->childIds.size() : 0;
}


int TreeOfMaps::pruneState(Id id)
{
    if(id == rootId_)
    {
        std::cerr << "WARNING: TreeOfMaps: Attempted to prune the root state!\n";
        return 0;
    }

    // If the node doesn't exist, then return without doing anything
    // Otherwise, run a preorder traversal where each node is removed.
    // Also remove from complete and leaves as needed
    auto node = nodeWithId(id);

    if(!node)
    {
        return 0;
    }

    Id parentId = node->parentId;

    if(!removeChild(parentId, id))
    {
        return 0;
    }

    int numRemoved = 1;     // remove the child at this point
    preorderTraversal(node, [&numRemoved,this](TreeNode& node) {
        numRemoved += 1;
        utils::erase_remove(complete_, node.state.get());
        utils::erase_remove(leaves_, node.state.get());
        states_.erase(node.id);     // WARNING: node reference invalidated at this point
    });

    // If no complete nodes exist, then the depth has been reduced by 1 and new complete nodes must be found
    if(complete_.empty())
    {
        std::cout << "Erased all complete maps. Rebuilding leaves.\n";
        --depth_;
        findLeaves();
    }

    // If our parent no longer has children prune it too
    if(states_[parentId].childIds.empty())
    {
        numRemoved += pruneState(parentId);
    }

    return numRemoved;
}


void TreeOfMaps::pruneIncompleteLeaves(void)
{
    std::vector<Id> toErase;
    for(auto& leaf : leaves_)
    {
        if(states_[leaf->id].childIds.empty())
        {
            toErase.push_back(leaf->id);
        }
    }

    for(auto& id : toErase)
    {
        pruneState(id);
    }
}


int TreeOfMaps::pruneLowProbability(double probabilityCutoff)
{
    PRINT_PRETTY_STUB();
    return 0;
}


HypothesisTree TreeOfMaps::toHypothesisTree(const ProbabilityHeuristics& heuristics)
{
    // Make sure leaves are valid when the tree is generated
    findLeaves();

    std::vector<hypothesis_tree_node_t> nodes;
    for(auto& n : boost::adaptors::values(states_))
    {
        hypothesis_tree_node_t node;
        node.id = n.id;
        node.depth = n.depth;
        node.children = n.childIds;
        node.probability = n.state->probability;
        node.probability.estimatedLogPrior = heuristics.logPriorHeuristic(n.depth);
        node.probability.estimatedLogLikelihood = heuristics.logLikelihoodHeuristic(n.depth);
        nodes.push_back(node);
    }

    return HypothesisTree(nodes, rootId_, leaves_.size(), complete_.size());
}


const TopologicalState* TreeOfMaps::stateWithId(Id id) const
{
    auto stateIt = states_.find(id);
    return (stateIt == states_.end()) ? nullptr : stateIt->second.state.get();
}


TreeOfMaps::TreeNode* TreeOfMaps::nodeWithId(Id id)
{
    auto nodeIt = states_.find(id);
    return nodeIt == states_.end() ? nullptr : &(nodeIt->second);
}


const TreeOfMaps::TreeNode* TreeOfMaps::nodeWithId(Id id) const
{
    auto nodeIt = states_.find(id);
    return nodeIt == states_.end() ? nullptr : &(nodeIt->second);
}


bool TreeOfMaps::removeChild(Id parent, Id child)
{
    // If there is a parent node, then remove this reference to the child
    auto parentNode = nodeWithId(parent);
    if(parentNode)
    {
        return utils::erase_remove(parentNode->childIds, child) > 0;
    }

    return false;
}


void TreeOfMaps::findLeaves(void)
{
    leaves_.clear();
    complete_.clear();

    for(auto& node : boost::adaptors::values(states_))
    {
        if(is_leaf(node))
        {
            leaves_.push_back(node.state.get());
        }

        if(node.depth == depth_)
        {
//             assert(is_leaf(node));
            complete_.push_back(node.state.get());
        }
    }

    std::cout << "Found " << leaves_.size() << " leaves and " << complete_.size() << " complete maps at depth "
        << depth_ << '\n';
}

} // namespace hssh
} // namespace vulcan

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

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_TREE_OF_MAPS_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_TREE_OF_MAPS_H

#include <hssh/global_topological/state.h>
#include <system/message_traits.h>
#include <cereal/access.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/unordered_map.hpp>
#include <vector>
#include <cstdint>

namespace vulcan
{
namespace hssh
{

class HypothesisTree;
class ProbabilityHeuristics;

/**
* TreeOfMaps maintains ownership of the current set of valid topological map hypotheses for the environment. The tree
* of maps maintains the hierarchy of topological states as they evolve as the robot explores the environment. The tree
* allows for iteration across complete states (those whose depth == depth of the tree) and leaf state (states with no
* child states). Other states are maintained internally to the tree and should not be accessed. For debugging purposes,
* a HypothesisTree + stateWithId methods can be used to access these, but they can't be used to actually change the
* tree in any way.
*
* A TreeOfMaps is noncopyable, but can be moved.
*/
class TreeOfMaps
{
public:

    using StateIter = std::vector<TopologicalState*>::const_iterator;

    TreeOfMaps(void) = default;
    TreeOfMaps(TreeOfMaps&&) = default;
    TreeOfMaps& operator=(TreeOfMaps&&) = default;
    TreeOfMaps(const TreeOfMaps&) = delete;
    TreeOfMaps& operator=(const TreeOfMaps&) = delete;

    /**
    * depth retrieves the current depth of the tree.
    *
    * The depth of the tree is automatically maintained. Whenever a complete state has a child node added, the depth
    * of the tree increases.
    *
    * Until the root is set, the depth is -1. A tree with a single root node has depth 0.
    */
    int depth(void) const { return depth_; }

    /**
    * addRootState adds the initial root state for the tree. This method can only be called on an empty TreeOfMaps.
    * Once the root state is set, the only way to change the root is via the setRootState method.
    *
    * \param    root        Initial state for the TreeOfMaps
    * \return   True if the state could be set as the root. False if the tree has already been initialized and thus
    *   the root can't be changed in this fashion.
    */
    bool addRootState(std::unique_ptr<TopologicalState>&& root);

    /**
    * addChildState adds a new child state for a parent state in the tree. If the parent state was previously a
    * leaf, it will no longer be one in the new tree.
    *
    * WARNING: Adding a child state invalidates leaf and complete state iterators.
    *
    * \param    parentId    Id of the parent state
    * \param    child       Full child state to add to the parent
    * \return   True if the parent actually existed, so the child could be added.
    */
    bool addChildState(Id parentId, std::unique_ptr<TopologicalState>&& child);

    /**
    * changeRootState changes the root of the tree. All nodes that are not descendents of this root state are discarded.
    * Changing the root state amounts to asserting some absolutely true condition about the environment. It can
    * dramatically reduce the size of the tree, even down to a single node.
    *
    * WARNING: Changing the root state invalidates leaf and complete state iterators.
    *
    * \param    id          Id of the new root state
    * \return   True if the root state was changed. False if no state with id exists.
    */
    bool changeRootState(Id id);

    /**
    * numChildren checks how many child states a state has.
    *
    * \param    id          Id to check for children
    * \return   Number of valid child states a parent state has.
    */
    int numChildren(Id id) const;

    /**
    * pruneState prunes a single state from the tree, marking it as inconsistent. The pruned state is removed from the
    * tree and from its parent's list of child nodes.
    *
    * NOTE: If a parent node is pruned, then all its children will be pruned as well.
    *
    * WARNING: Pruning invalidates leaf and complete state iterators.
    *
    * \param    id          Id of the state to be pruned from the tree
    * \return   Total number of states pruned from the tree.
    */
    int pruneState(Id id);

    /**
    * Prune all leaves with no children. In some modes, incomplete leaves should never exist and are a sign that a
    * leaf is bad.
    */
    void pruneIncompleteLeaves(void);

    /**
    * pruneLowProbability prunes all low-probability states from the tree. The posterior probabilty threshold is
    * provided the determines what low probability means.
    *
    * WARNING: Pruning low-probability states invalidates leaf and complete state iterators.
    *
    * \param    probabilityCutoff           The posterior probability cutoff below which a state is discarded
    * \return   The number of low-probability states pruned from the tree.
    */
    int pruneLowProbability(double probabilityCutoff);

    /////   Iterator access for all leaves in the tree   /////
    std::size_t sizeLeaves(void) const { return leaves_.size(); }
    StateIter beginLeaves(void) const { return leaves_.begin(); }
    StateIter endLeaves(void) const { return leaves_.end(); }
    TopologicalState* leafAt(int index) const { return leaves_.at(index); }

    /////   Iterator access for all complete states in the tree   /////
    std::size_t sizeComplete(void) const { return complete_.size(); }
    StateIter beginComplete(void) const { return complete_.begin(); }
    StateIter endComplete(void) const { return complete_.end(); }
    TopologicalState* completeAt(int index) const { return complete_.at(index); }

    /**
    * size retrieves the size of the full tree, including all internal states.
    */
    std::size_t size(void) const { return states_.size(); }
    bool empty(void) const { return states_.empty(); }

    /////   Debugging support   /////
    /**
    * toHypothesisTree converts the TreeOfMaps into a simpler HypothesisTree that is suitable for visualization.
    * The TreeOfMaps itself isn't suitable because it hides the internal state of non-complete, non-leaf nodes.
    *
    * \param    heuristics          Heuristics for the state, so they can be saved in the tree
    * \return   HypothesisTree representation of the TreeOfMaps.
    */
    HypothesisTree toHypothesisTree(const ProbabilityHeuristics& heuristics);

    /**
    * stateWithId retrieves a TopologicalState with the given id. If no state with this id exists, then a nullptr
    * is returned. This method is intended for debugging to allow visualizing the internal state of the tree in
    * conjunction with a HypothesisTree.
    *
    * \param    id          Id of the state to retrieve
    * \return   A pointer to the desired state. nullptr if no state with id exists in the tree.
    */
    const TopologicalState* stateWithId(Id id) const;

private:

    using StatePtr = std::unique_ptr<TopologicalState>;

    // TreeNode maintains all internal information about a state and its children and parent
    struct TreeNode
    {
        Id id = kInvalidId;
        int depth = 0;
        Id parentId = kInvalidId;
        std::vector<Id> childIds;
        bool hadChildren = false;       // flag indicating if childIds has been pruned back to empty, so
                                        // a node doesn't reappear as a leaf
        StatePtr state;

        TreeNode(void) = default;

        TreeNode(Id id, int depth, Id parentId, StatePtr&& state)
        : id(id)
        , depth(depth)
        , parentId(parentId)
        , state(std::move(state))
        {
        }

        // Serialization support
        template <class Archive>
        void serialize(Archive& ar, const unsigned int version)
        {
            ar (id,
                depth,
                parentId,
                childIds,
                hadChildren,
                state
            );
        }
    };

    // INVARIANT: A single copy of the TopologicalState exists in the states_ map.
    using States = std::unordered_map<Id, TreeNode>;
    using Leaves = std::vector<TopologicalState*>;

    States states_;         // all states that exist in the hypothesis space
    Leaves complete_;       // leaves with depth == depth
    Leaves leaves_;         // all leaves in the tree, regardless of depth

    int depth_ = -1;        // depth of complete nodes in the tree
    Id rootId_ = -1;        // id of root node for easy access, otherwise requires searching states_

    TreeNode* nodeWithId(Id id);
    const TreeNode* nodeWithId(Id id) const;
    bool removeChild(Id parent, Id child);
    bool is_leaf(const TreeNode& node) { return node.childIds.empty() && !node.hadChildren; }
    void findLeaves(void);

    // Op is unary function Op(TreeNode& node)
    template <class Op>
    void preorderTraversal(TreeNode* node, Op o)
    {
        for(auto& c : node->childIds)
        {
            preorderTraversal(&states_[c], o);
        }

        o(*node);
    }

    // Serialization support -- can't save raw pointers, so need to separate save/load and reconstruct the
    // leaves_ and complete_ vectors upon loading
    friend class cereal::access;

    template <class Archive>
    void save(Archive& ar, const unsigned int version) const
    {
        ar (states_,
            depth_
        );
    }

    template <class Archive>
    void load(Archive& ar, const unsigned int version)
    {
        ar (states_,
            depth_
        );

        // Go through all the states. Any state without children gets added to leaves_ and those with depth==depth_
        // will get added to complete_
        for(auto& s : states_)
        {
            const TreeNode& node = s.second;
            if(node.depth == depth_)
            {
                complete_.push_back(node.state.get());
            }
            if(node.childIds.empty() && !node.hadChildren)
            {
                leaves_.push_back(node.state.get());
            }
        }
    }
};

} // namespace hssh
} // namespace vulcan

DEFINE_DEBUG_MESSAGE(hssh::TreeOfMaps, ("DEBUG_GLOBAL_TOPO_TREE_OF_MAPS"))

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_TREE_OF_MAPS_H

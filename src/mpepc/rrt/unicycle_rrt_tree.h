/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     unicycle_rrt_tree.h
* \author   Jong Jin Park
* 
* Declaration of UnicycleRRTTree class, which encapsulates the rapidly exploring
* tree of poses for unicycle-type vehicles.
*/

#ifndef UNICYCLE_RRT_TREE_H
#define UNICYCLE_RRT_TREE_H

#include "mpepc/rrt/unicycle_rrt_node.h"
#include "mpepc/math/unicycle_lyapunov_distance.h"
#include <list>

namespace vulcan
{
namespace mpepc
{

class UnicycleRRTTree
{
public:
    
    using nodePtr = unicycle_rrt_node_t*;
    
    /**
    * Constructor for UnicycleRRTTree 
    * 
    * \param    startPose       start pose for the path. Forms the root node.
    * \param    distanceParams  parameters for UnicycleLyapunovDistance
    */
    UnicycleRRTTree(const pose_t& startPose, const unicycle_lyapunov_distance_params_t& distanceParams);
    
    UnicycleRRTTree(void) {};
    
    // mutators
    /**
    * initializeTree initializes the rrt tree with the goal node
    *
    * \param    initialPose     pose of the goal
    */
    void initializeTree(const pose_t& startPose);
    
    /**
    * insertNode creates a node, insert it to the tree, and connect it to a parent node
    *
    * \param    newSample               sampled pose to be added
    * \param    costFromParent          cost from the parent
    * \param    isGoal                  sample is the goal
    * \param    isMovingBackward        sample should be approached with backward motion
    * \param    parentNodePtr           parent node to be connected (will be modified)
    * \return   pointer to the inserted node
    */
    nodePtr insertNode(const pose_t& newSample, float costFromParent, bool isGoal, bool isMovingBackward, nodePtr parentNodePtr);
    
    /**
    * deleteChildFromParent attetmpts to delete a child node from a parent node.
    * 
    * \param    childNodeToDelete       pointer to the child node to be deleted
    * \param    parentNode              pointer to the parent node
    * 
    * \return   success/failure
    */
    bool deleteChildFromParent(const unicycle_rrt_node_t* childToDelete, nodePtr parentNode);
    
    
    // find operations
    
    /**
    * findNearbyNodesFromPose finds a set of nodes whose directed distance *from*
    * a pose is less than specified threshold.
    * 
    * \param    pose                    a pose from which the distance is measured.
    * \param    distanceThreshold       threshold for the non-holonomic distance
    * \param    isMovingBackward        approach direction indicator (optional. default = forward)
    * 
    * \return   a vector of pointers to nearby nodes
    */
    std::vector<nodePtr> findNearbyNodesFromPose(const pose_t& pose, float distanceThreshold, bool  isMovingBackward = false);
    
    /**
    * findNearbyNodesToPose finds a set of nodes whose directed distance *to*
    * a pose is less than specified threshold.
    * 
    * \param    pose                    a pose to which the distance is measured.
    * \param    distanceThreshold       threshold for the non-holonomic distance
    * \param    isMovingBackward        approach direction indicator (optional. default = forward)
    * 
    * \return   a vector of pointers to nearby nodes
    */
    std::vector<nodePtr> findNearbyNodesToPose(const pose_t& pose, float distanceThreshold, bool  isMovingBackward = false);
    
    /**
    * findNeaestNodeFromPose finds the nearest node *from* a pose.
    * 
    * \param    pose                    a pose from which the distance is measured.
    * \param    isMovingBackward        approach direction indicator (optional. default = forward)
    * 
    * \return   pointer to the nearest node
    */
    nodePtr findNearestNodeFromPose(const pose_t& pose, double* nonHolonomicDistance, bool isMovingBackward = false);
    
    /**
    * findNearestNodesToPose finds the nearest node *to* a pose, *excluding the goal.*
    * 
    * \param    pose                    a pose to which the distance is measured.
    * \param    isMovingBackward        approach direction indicator (optional. default = forward)
    * 
    * \return   pointer to the nearest node
    */
    nodePtr findNearestNodeToPose(const pose_t& pose, double* nonHolonomicDistance, bool isMovingBackward = false);
    
    std::list<unicycle_rrt_node_t> tree_; // WARNING: using vector here can invalidate pointers.
    
    unicycle_lyapunov_distance_params_t distanceParams_;
    
private:
    
    friend class UnicycleRRTStar;
};

} // mpepc
} // vulcan

#endif
/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     unicycle_rrt_tree.cpp
* \author   Jong Jin Park
* 
* Definition of UnicycleRRTTree class, which encapsulates the rapidly exploring
* tree of poses for unicycle-type vehicles.
*/

#include <mpepc/rrt/unicycle_rrt_tree.h>

namespace vulcan
{
namespace mpepc
{

UnicycleRRTTree::UnicycleRRTTree(const pose_t& startPose, const unicycle_lyapunov_distance_params_t& distanceParams)
: distanceParams_(distanceParams)
{
    initializeTree(startPose);
};


void UnicycleRRTTree::initializeTree(const pose_t& startPose)
{
    // create root node
    unicycle_rrt_node_t rootNode(0, startPose, 0.0, 0, 0); // ID, pose, cost, isGoal, isMovingBackward
    
    rootNode.parent = nullptr;
    rootNode.children.clear();
    
    // initialize the tree and add a node
    tree_.clear();
    tree_.push_back(rootNode);
}


unicycle_rrt_node_t* UnicycleRRTTree::insertNode(const pose_t& newSample, float costFromParent, bool isGoal, bool isMovingBackward, unicycle_rrt_node_t* parentNodePtr)
{
    assert(parentNodePtr != nullptr);
    
    // create a new node
    int newNodeID = tree_.back().nodeID + 1; // this depends on the monotonically increasing IDs in the tree, which is guaranteed by construction.
    unicycle_rrt_node_t newNode(newNodeID, newSample, parentNodePtr->cost + costFromParent, isGoal, isMovingBackward);
    newNode.parent = parentNodePtr; // set parent
    
    // add to tree
    tree_.push_back(newNode);
    
    // add a child to the parent node
    unicycle_rrt_node_t* insertedNode = &(tree_.back());
    parentNodePtr->children.push_back(insertedNode);
    
    return insertedNode;
}


bool UnicycleRRTTree::deleteChildFromParent(const unicycle_rrt_node_t* childToDelete, unicycle_rrt_node_t* parentNode)
{
    bool isDeleted = false;
    
    for(auto childIt = parentNode->children.begin(), childEnd = parentNode->children.end(); childIt != childEnd; childIt++)
    {
        if(*childIt == childToDelete)
        {
            isDeleted = true;
            parentNode->children.erase(childIt); // this invalidates the iterator! has to exit now.
            break; // assume no duplicate child
        }
    }
    
    return isDeleted; // report successful deletion
}


std::vector<unicycle_rrt_node_t*> UnicycleRRTTree::findNearbyNodesFromPose(const pose_t& pose, float distanceThreshold, bool isMovingBackward)
{
    std::vector<unicycle_rrt_node_t*> neighbors;
    
    // compute distance *from* a pose ...
    pose_t fromPose = isMovingBackward ? pose.flip() : pose;
    
    for(auto nodeIt = tree_.begin(), nodeEnd = tree_.end(); nodeIt != nodeEnd; nodeIt++)
    {
        // *to* a node
        pose_t toPose  = isMovingBackward ? nodeIt->pose.flip() : pose;
        UnicycleLyapunovDistance lyap(toPose, distanceParams_); // Lyapunov distance function around each node
        double distanceToNode = lyap.distanceFromPose(fromPose);
        
        // add to neighbors if the distance is less than threshold
        if(distanceToNode < distanceThreshold)
        {
            neighbors.push_back(&(*nodeIt));
        }
    }
    
    return neighbors;
}


std::vector<unicycle_rrt_node_t*> UnicycleRRTTree::findNearbyNodesToPose(const pose_t& pose, float distanceThreshold, bool isMovingBackward)
{
    std::vector<unicycle_rrt_node_t*> neighbors;
    
    // compute distance *to* a pose ...
    pose_t toPose   = isMovingBackward ? pose.flip() : pose;
    UnicycleLyapunovDistance lyap(toPose, distanceParams_); // Lyapunov distance function around the pose
    
    for(auto nodeIt = tree_.begin(), nodeEnd = tree_.end(); nodeIt != nodeEnd; nodeIt++)
    {
        // *from* a node
        pose_t fromPose = isMovingBackward ? nodeIt->pose.flip() : nodeIt->pose;
        double distanceToNode  = lyap.distanceFromPose(fromPose);
        
        // add to neighbors if the distance is less than threshold
        if(distanceToNode < distanceThreshold)
        {
            neighbors.push_back(&(*nodeIt));
        }
    }
    
    return neighbors;
};


unicycle_rrt_node_t* UnicycleRRTTree::findNearestNodeFromPose(const pose_t& pose, double* nonHolonomicDistance, bool isMovingBackward)
{
    unicycle_rrt_node_t* nearestNode = nullptr;
    double  minDistance = 1000000.0; // some large number
    
    // compute distance *from* a pose
    pose_t fromPose = isMovingBackward ? pose.flip() : pose;
    
    for(auto nodeIt = tree_.begin(), nodeEnd = tree_.end(); nodeIt != nodeEnd; nodeIt++)
    {
        // *to* a node
        pose_t toPose  = isMovingBackward ? nodeIt->pose.flip(): nodeIt->pose;
        UnicycleLyapunovDistance lyap(toPose, distanceParams_);
        double distanceToNode = lyap.distanceFromPose(fromPose);
        
        if(minDistance > distanceToNode)
        {
            nearestNode = &(*nodeIt);
            minDistance = distanceToNode;
        }
    }
    
    if(nonHolonomicDistance)
    {
        *nonHolonomicDistance = minDistance;
    }
    
    return nearestNode;
};


unicycle_rrt_node_t* UnicycleRRTTree::findNearestNodeToPose(const pose_t& pose, double* nonHolonomicDistance, bool isMovingBackward)
{
    unicycle_rrt_node_t* nearestNode = nullptr;
    double  minDistance = 1000000.0; // some large number;
    
    // compute distance *to* a pose
    pose_t toPose = isMovingBackward ? pose.flip() : pose;
    UnicycleLyapunovDistance lyap(toPose, distanceParams_);
    
    for(auto nodeIt = tree_.begin(), nodeEnd = tree_.end(); nodeIt != nodeEnd; nodeIt++)
    {
        if(!nodeIt->isGoal) // need to exclude the goal pose here!
        {
            pose_t fromPose  = isMovingBackward ? nodeIt->pose.flip() : nodeIt->pose;
            double distanceFromNode = lyap.distanceFromPose(fromPose);
            
            if(minDistance > distanceFromNode)
            {
                nearestNode = &(*nodeIt);
                minDistance = distanceFromNode;
            }
        }
    }
    
    if(nonHolonomicDistance)
    {
        *nonHolonomicDistance = minDistance;
    }
    
    return nearestNode;
};

} // mpepc
} // vulcan

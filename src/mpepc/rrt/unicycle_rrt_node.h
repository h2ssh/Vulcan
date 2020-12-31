/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     unicycle_rrt_node.h
* \author   Jong Jin Park
* 
* Declaration of unicycle_rrt_node_t.
*/

#ifndef UNICYCLE_RRT_NODE_H
#define UNICYCLE_RRT_NODE_H

#include "core/pose.h"
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{
namespace mpepc
{

/*
* unicycle_rrt_node_t stores data for a node in a tree (pose graph).
*/
struct unicycle_rrt_node_t
{
    int                               nodeID;   // node ID
    pose_t                     pose;     // pose of the waypoint associated with the node
    unicycle_rrt_node_t*              parent;   // pointer to parent. (This is nullptr for the root node)
    std::vector<unicycle_rrt_node_t*> children; // pointers to children nodes
    double                            cost;     // cost from the root node
    bool                              isGoal;   // indicator for reaching the goal
    bool                              shouldBeApproachedBackward; // approach direction indicator
    
    unicycle_rrt_node_t(void) {};
    unicycle_rrt_node_t(int nodeID, const pose_t& pose, double cost, bool isGoal, bool shouldBeApproachedBackward)
    : nodeID(nodeID)
    , pose(pose)
    , cost(cost)
    , isGoal(isGoal)
    , shouldBeApproachedBackward(shouldBeApproachedBackward)
    {
        // pointers needs to be set manually
    };
};


/*
* unicycle_rrt_node_t stores read/writable data associated with node (without pointers)
* NOTE: edge data (e.g. points on a path) is not stored as they can be
*       reconstructed easily via steering. Edge data may get added if needed.
*/
struct unicycle_rrt_node_data_t
{
    int                        nodeID;
    pose_t              pose;
    std::vector<int>           parentID;   // can be empty
    std::vector<pose_t> parentPose; // can be empty
//     std::vector<int>           childIDs;   // can be empty
//     std::vector<pose_t> childPoses; // can be empty
    double                     cost;
    bool                       isGoal;
    bool                       shouldBeApproachedBackward;
    
    unicycle_rrt_node_data_t(void) {};
    unicycle_rrt_node_data_t(const unicycle_rrt_node_t& node)
    : nodeID(node.nodeID)
    , pose(node.pose)
    , cost(node.cost)
    , isGoal(node.isGoal)
    , shouldBeApproachedBackward(node.shouldBeApproachedBackward)
    {
        if(node.parent)
        {
            parentID.push_back(node.parent->nodeID);
            parentPose.push_back(node.parent->pose);
        };
        
//         for(auto childNodeIt = node.children.begin(); childNodeIt != node.children.end(); childNodeIt++)
//         {
//             childIDs.push_back((*childNodeIt)->nodeID);
//             childPoses.push_back((*childNodeIt)->pose);
//         };
    };
};


// Serialization support
template <class Archive>
void serialize(Archive& ar, unicycle_rrt_node_data_t& data)
{
    ar (data.nodeID,
        data.pose,
        data.parentID,
        data.parentPose,
//         data.childIDs,
//         data.childPoses,
        data.cost,
        data.isGoal,
        data.shouldBeApproachedBackward);
}

} // mpepc
} // vulcan

#endif // UNICYCLE_RRT_NODE_H
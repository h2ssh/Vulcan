/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     unicycle_rrt_star.cpp
 * \author   Jong Jin Park
 *
 * Definition of  UnicycleRRTStar class, which implements non-holonomic RRT* for
 * unicycle-type vehicles, as introduced in [Park 15].
 */

#include "mpepc/rrt/unicycle_rrt_star.h"
#include "utils/timestamp.h"

namespace vulcan
{
namespace mpepc
{

// Constructor
UnicycleRRTStar::UnicycleRRTStar(const pose_t& startPose,
                                 const pose_t& goalPose,
                                 const ObstacleDistanceGrid& grid,
                                 const unicycle_rrt_star_params_t& params)
: startPose_(startPose)
, goalPose_(goalPose)
, grid_(grid)
, numSamples_(0)
, totalMilliSecondsSpent_(0.0)
, averageMilliSecondPerSample_(0.0)
, poseSampler_(params.samplerParams)
, steering_(params.steeringParams)
, RRT_(startPose, params.distanceParams)
, haveFoundPath_(false)
, params_(params)
{
}


void UnicycleRRTStar::resetObstacleGridMap(const ObstacleDistanceGrid& grid)
{
    grid_ = grid;

    bool condition = false;   // TODO: what is the desired condition here?
    if (condition) {
        clearRRTStats();
        clearFoundPaths();

        RRT_.initializeTree(startPose_);
    }
}


// Path-building methods
bool UnicycleRRTStar::findNewPath(size_t maxNumSamples, bool canTerminateEarly)
{

    std::cout << "UnicycleRRTStarInfo: Initializing New Tree... \n";

    numSamples_ = 0;
    minimumCostToGoal_ = 10000;   // some large number
    minimumCostLeaf_ = nullptr;

    // improvePath() updates numSamples, minimumCostToGoal, and minimumCostLeaf internally
    haveFoundPath_ = improvePath(maxNumSamples, canTerminateEarly);

    return haveFoundPath_;
}


bool UnicycleRRTStar::improvePath(size_t maxAdditionalSamples, bool canTerminateEarly)
{
    size_t numNewSamples = 0;
    bool haveImprovedPath = false;

    int64_t tic = utils::system_time_us();

    while (numNewSamples <= maxAdditionalSamples) {
        numNewSamples++;

        bool isGoal = false;
        bool isMovingBackward = false;

        // sample a pose
        pose_t sampledPose = rrtSample(&isGoal, &isMovingBackward);
        std::cout << "NumNewSamples:" << numNewSamples << "    maxAdditionalSamples:" << maxAdditionalSamples << '\n';

        // find the nearest node

        std::cout << "Finding the nearest node from the sample ... and is moving backward?: " << isMovingBackward
                  << '\n';
        unicycle_rrt_node_t* nearestNode = nodeNearestTo(sampledPose, isMovingBackward);
        std::cout << "Nearest node found. Extending to the nearest node ... \n";
        unicycle_path_segment_t edgePath = extendTo(*nearestNode, sampledPose, params_.maxExtensionLength);

        std::cout << "NumNewSamples:" << numNewSamples << "    maxAdditionalSamples:" << maxAdditionalSamples
                  << "    Nearest Node: " << nearestNode->pose << '\n';

        if (obstacleFree(edgePath.steps, grid_)) {
            std::cout << "Path to the nearest node is collision free : " << nearestNode->pose << '\n';

            // find nearby nodes
            pose_t newSample = edgePath.steps.back();
            std::vector<unicycle_rrt_node_t*> nearbyNodes = nodesNearTo(sampledPose, isMovingBackward);
            std::cout << "Found nearby nodes. Finding parent node...\n";

            // choose parent by choosing the minimum-cost route from root
            unicycle_rrt_node_t* parentNode = rrtChooseParent(nearbyNodes, nearestNode, newSample, isMovingBackward);
            std::cout << "Found parent node. Inserting node to the tree...\n";

            // insert node to the tree
            unicycle_rrt_node_t* newNode = rrtInsertNode(parentNode, newSample, isGoal, isMovingBackward);
            std::cout << "Node inserted. Finding nodes near from the inserted node...\n";

            // find nearby nodes from the sample (this will include itself, which is the last sample in the tree)
            nearbyNodes = nodesNearFrom(newSample, isMovingBackward);
            std::cout << "Found nodes near from the inserted node. Rewiring...\n";

            // rewire neargy nodes to the new node, if it decreases overall cost.
            rrtRewire(nearbyNodes, newNode);
            std::cout << "Rewiring complete.\n";

            if (isGoal) {
                if (newNode->cost < minimumCostToGoal_) {
                    minimumCostToGoal_ = newNode->cost;
                    minimumCostLeaf_ = newNode;
                }

                haveImprovedPath = true;
            }
        }

        if (haveImprovedPath && canTerminateEarly) {
            break;
        }
    }

    int64_t toc = utils::system_time_us();

    // compute and update stats
    double milliSecondsSpent = static_cast<double>(toc - tic) / 1000.0;
    numSamples_ += numNewSamples;
    totalMilliSecondsSpent_ += milliSecondsSpent;
    if (numSamples_ != 0) {
        averageMilliSecondPerSample_ = totalMilliSecondsSpent_ / numSamples_;
    }

    std::cout << "UnicycleRRTStarInfo: NumSamples : " << numSamples_
              << "    TotalTimeSpent (ms): " << totalMilliSecondsSpent_ << "\n";

    //
    if (haveImprovedPath) {
        // update path information
        retrievePathFromLeaf(minimumCostLeaf_);
    }

    return haveImprovedPath;
}


bool UnicycleRRTStar::addSamples(size_t numSamplesToAdd)
{
    if (numSamples_ == 0) {
        return findNewPath(numSamplesToAdd, true);
    } else {
        return improvePath(numSamplesToAdd, false);
    }
}


std::list<unicycle_rrt_node_data_t> UnicycleRRTStar::getGraphInfo(void)
{
    std::list<unicycle_rrt_node_data_t> info;

    for (auto nodeIt = RRT_.tree_.begin(), nodeEnd = RRT_.tree_.end(); nodeIt != nodeEnd; nodeIt++) {
        info.push_back(unicycle_rrt_node_data_t(*nodeIt));
    }

    return info;
}


// Sampling
pose_t UnicycleRRTStar::rrtSample(bool* isGoal, bool* isMovingBackward) const
{
    if (isMovingBackward != nullptr) {
        *isMovingBackward = drand48() < params_.backwardMotionSamplingRate;
    }

    // using goal-biased sampling
    return poseSampler_.getGoalBiasedSample(goalPose_, grid_, isGoal);
}


// Steering
unicycle_path_segment_t UnicycleRRTStar::extendTo(const unicycle_rrt_node_t& rrtNode,
                                                  const pose_t& sampledPose,
                                                  double maxExtension,
                                                  bool isMovingBackward) const
{
    pose_t poseFrom = isMovingBackward ? rrtNode.pose.flip() : rrtNode.pose;
    pose_t poseTo = isMovingBackward ? sampledPose.flip() : sampledPose;

    UnicycleLyapunovDistance lyap(poseTo, params_.distanceParams);
    return steering_.extend(poseFrom, lyap, maxExtension);
}


unicycle_path_segment_t UnicycleRRTStar::steerTo(const pose_t& sampledPose, const unicycle_rrt_node_t& rrtNode) const
{
    bool isMovingBackward = rrtNode.shouldBeApproachedBackward;
    pose_t poseFrom = isMovingBackward ? sampledPose.flip() : sampledPose;
    pose_t poseTo = isMovingBackward ? rrtNode.pose.flip() : rrtNode.pose;

    UnicycleLyapunovDistance lyap(poseTo, params_.distanceParams);
    return steering_.steer(poseFrom, lyap);
}


bool UnicycleRRTStar::obstacleFree(const std::vector<pose_t>& steps, const ObstacleDistanceGrid& grid) const
{
    float kDistanceThreshlod = 0.35;

    bool isCollisionFree = true;
    for (auto stepIt = steps.begin(), stepEnd = steps.end(); stepIt != stepEnd; stepIt++) {
        if (grid.getObstacleDistance((*stepIt).toPoint()) < kDistanceThreshlod) {
            isCollisionFree = false;
            break;
        }
    }

    return isCollisionFree;
}


// Neighbor searches
double UnicycleRRTStar::neighborhoodRadius(bool isMovingBackward)
{
    double kSearchSpaceDimension = 3.0;
    double numNodes = static_cast<double>(RRT_.tree_.size());
    double distanceMultiplier = pow(log(numNodes) / numNodes, 1.0 / kSearchSpaceDimension);

    if (isMovingBackward) {
        distanceMultiplier /= params_.backwardDistanceWeight;
    }

    return distanceMultiplier * params_.nearnessParamGamma;
}


unicycle_rrt_node_t* UnicycleRRTStar::nodeNearestTo(const pose_t& sampledPose, bool isMovingBackward)
{
    return RRT_.findNearestNodeToPose(sampledPose, nullptr, isMovingBackward);
}


std::vector<unicycle_rrt_node_t*> UnicycleRRTStar::nodesNearTo(const pose_t& sampledPose, bool isMovingBackward)
{
    return RRT_.findNearbyNodesToPose(sampledPose, neighborhoodRadius(isMovingBackward), isMovingBackward);
}


std::vector<unicycle_rrt_node_t*> UnicycleRRTStar::nodesNearFrom(const pose_t& sampledPose, bool isMovingBackward)
{
    return RRT_.findNearbyNodesFromPose(sampledPose, neighborhoodRadius(isMovingBackward), isMovingBackward);
}


// Cost definitions

double UnicycleRRTStar::rrtCostFromNodeToPose(const unicycle_rrt_node_t& fromNode,
                                              const pose_t& toPose,
                                              bool isMovingBackward)
{
    UnicycleLyapunovDistance lyap(isMovingBackward ? toPose.flip() : toPose, params_.distanceParams);
    return lyap.distanceFromPose(isMovingBackward ? fromNode.pose : fromNode.pose);
}


double UnicycleRRTStar::rrtCostFromPoseToNode(const pose_t& fromPose, const unicycle_rrt_node_t& toNode)
{
    UnicycleLyapunovDistance lyap(toNode.shouldBeApproachedBackward ? toNode.pose.flip() : toNode.pose,
                                  params_.distanceParams);
    return lyap.distanceFromPose(toNode.shouldBeApproachedBackward ? fromPose.flip() : fromPose);
}


double UnicycleRRTStar::rrtCostToGoViaNode(const pose_t& robotPose, const unicycle_rrt_node_t& node)
{
    if (haveFoundPath_) {
        return pathNodes_.back().cost - node.cost + rrtCostFromPoseToNode(robotPose, node);
    } else {
        return 1000.0;   // some large number
    }
}


// RRT-Star methods
unicycle_rrt_node_t* UnicycleRRTStar::rrtChooseParent(const std::vector<unicycle_rrt_node_t*>& nodesNearTo,
                                                      unicycle_rrt_node_t* nearestNode,
                                                      const pose_t& samplePose,
                                                      bool isMovingBackward)
{
    // control-laypunov distance function to target pose
    UnicycleLyapunovDistance lyap(samplePose, params_.distanceParams);

    // initialize with the nearest node
    unicycle_rrt_node_t* parentNode = nearestNode;
    double minCostFromRoot = parentNode->cost + rrtCostFromNodeToPose(*parentNode, samplePose, isMovingBackward);
    ;

    // iterate among nearby nodes to find the best node
    for (auto nodeIt = nodesNearTo.begin(), nodeEnd = nodesNearTo.end(); nodeIt != nodeEnd; nodeIt++) {
        // generate path to the node
        unicycle_path_segment_t pathToPose = steering_.steer((*nodeIt)->pose, lyap);

        // collision check
        bool poseCanBeReachedFromNode = obstacleFree(pathToPose.steps, grid_);

        // update, if collision-free
        if (poseCanBeReachedFromNode) {
            // check cost
            double costFromRoot = (*nodeIt)->cost + rrtCostFromNodeToPose(**nodeIt, samplePose, isMovingBackward);

            // choose the node with the minimum cost from the root as the parent
            if (costFromRoot < minCostFromRoot) {
                parentNode = *nodeIt;
                minCostFromRoot = costFromRoot;
            }
        }
    }

    return parentNode;
}


unicycle_rrt_node_t* UnicycleRRTStar::rrtInsertNode(unicycle_rrt_node_t* parentNode,
                                                    const pose_t& childPose,
                                                    bool isGoal,
                                                    bool isMovingBackward)
{
    double costFromParent = rrtCostFromNodeToPose(*parentNode, childPose, isMovingBackward);
    unicycle_rrt_node_t* insertedNode =
      RRT_.insertNode(childPose, costFromParent, isGoal, isMovingBackward, parentNode);

    return insertedNode;
}


void UnicycleRRTStar::rrtRewire(const std::vector<unicycle_rrt_node_t*>& nearbyNodes, unicycle_rrt_node_t* rewireNode)
{
    for (auto nodeIt = nearbyNodes.begin(), nodeEnd = nearbyNodes.end(); nodeIt != nodeEnd; nodeIt++) {
        // if a node nearby is not the parent or itself
        if ((*nodeIt) != rewireNode->parent && (*nodeIt) != rewireNode) {
            // find path to the node
            unicycle_path_segment_t pathToNode = steerTo(rewireNode->pose, **nodeIt);

            // check collision
            bool nodeCanBeReachedFromPose = obstacleFree(pathToNode.steps, grid_);

            if (nodeCanBeReachedFromPose) {
                // compute cost via rewire node
                double rewiredEdgeCost = rrtCostFromPoseToNode(rewireNode->pose, **nodeIt);
                double rewiredCostToRoot = rewireNode->cost + rewiredEdgeCost;

                // if the cost can be improved
                if (rewiredCostToRoot < (*nodeIt)->cost) {
                    // reconnect edges and propagate cost to leaves
                    rrtReconnect(rewireNode, *nodeIt, rewiredEdgeCost);
                }
            }
        }
    }
}


void UnicycleRRTStar::rrtReconnect(unicycle_rrt_node_t* newParent,
                                   unicycle_rrt_node_t* childToRewire,
                                   double rewiredEdgeCost)
{
    // delete connection to old parent
    RRT_.deleteChildFromParent(childToRewire, childToRewire->parent);

    // rewire to the new node
    newParent->children.push_back(childToRewire);   // set child to parent
    childToRewire->parent = newParent;              // set parent to child
    childToRewire->cost = newParent->cost + rewiredEdgeCost;

    // propagate cost down the line
    rrtPropagateCostToChildren(childToRewire);
}


void UnicycleRRTStar::rrtPropagateCostToChildren(unicycle_rrt_node_t* parentNode)
{
    for (auto childIt = parentNode->children.begin(), childEnd = parentNode->children.end(); childIt != childEnd;
         childIt++) {
        // compute cost between edge
        double costToChild =
          rrtCostFromNodeToPose(*parentNode, (*childIt)->pose, (*childIt)->shouldBeApproachedBackward);
        (*childIt)->cost = parentNode->cost + costToChild;

        // propagate until reached leaves, where there are no more edges to propagate
        rrtPropagateCostToChildren(*childIt);
    }
}


bool UnicycleRRTStar::branchAndBound(int* numNodesPruned)
{
    return false;   // TODO: Implement this
}


bool UnicycleRRTStar::retrievePathFromLeaf(unicycle_rrt_node_t* leafNode)
{
    if (minimumCostLeaf_ == nullptr) {
        return false;
    } else {
        // retrieve path data
        unicycle_rrt_node_t* nodePtr = leafNode;
        while (nodePtr != nullptr) {
            pathNodes_.push_back(unicycle_rrt_node_data_t(*nodePtr));
            nodePtr = nodePtr->parent;
        }

        // construct other path representations
        if (!pathNodes_.empty()) {
            std::reverse(pathNodes_.begin(), pathNodes_.end());

            for (auto pathNodeIt = pathNodes_.begin(), pathNodeEnd = pathNodes_.end(); pathNodeIt != pathNodeEnd;
                 pathNodeIt++) {
                // path info
                if (pathNodeIt != pathNodes_.begin()) {
                    pose_t robotPose = pathNodeIt->parentPose.front();
                    pose_t targetPose = pathNodeIt->pose;

                    if (pathNodeIt->shouldBeApproachedBackward) {
                        robotPose = robotPose.flip();
                        targetPose = targetPose.flip();
                    }

                    UnicycleLyapunovDistance lyap(targetPose, params_.distanceParams);
                    unicycle_path_segment_t pathSegment = steering_.steer(robotPose, lyap);

                    plannedPath_.insert(plannedPath_.begin(),
                                        pathSegment.path.begin(),
                                        pathSegment.path.end());   // NOTE: Is steps sufficient?
                }

                // waypoint info
                plannedWaypoints_.push_back(std::make_pair(pathNodeIt->pose, pathNodeIt->shouldBeApproachedBackward));
            }
        }

        return true;
    }
}


void UnicycleRRTStar::clearRRTStats(void)
{
    numSamples_ = 0;
    totalMilliSecondsSpent_ = 0.0;
    averageMilliSecondPerSample_ = 0.0;
}


void UnicycleRRTStar::clearFoundPaths(void)
{
    haveFoundPath_ = false;
    minimumCostToGoal_ = 65535.0;   // some large number
    minimumCostLeaf_ = nullptr;

    pathNodes_.clear();
    plannedPath_.clear();
    plannedWaypoints_.clear();
}

}   // namespace mpepc
}   // namespace vulcan

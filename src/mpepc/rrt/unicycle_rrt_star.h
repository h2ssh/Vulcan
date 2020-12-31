/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     unicycle_rrt_star.h
 * \author   Jong Jin Park
 *
 * Declaration of UnicycleRRTStar class, which implements non-holonomic RRT* for
 * unicycle-type vehicles, as introduced in [Park 15].
 */

#ifndef UNICYCLE_RRT_STAR_H
    #define UNICYCLE_RRT_STAR_H

    #include "mpepc/grid/obstacle_distance_grid.h"
    #include "mpepc/math/unicycle_lyapunov_distance.h"
    #include "mpepc/rrt/unicycle_lyapunov_steering.h"
    #include "mpepc/rrt/unicycle_pose_sampler.h"
    #include "mpepc/rrt/unicycle_rrt_tree.h"

namespace vulcan
{
namespace mpepc
{

struct unicycle_rrt_star_params_t
{
    std::size_t defaultNumSamples;
    double backwardMotionSamplingRate = 0.2;   // 0.0 ~ 1.0
    double backwardDistanceWeight = 3.0;       // > 1.0
    double nearnessParamGamma = 3.0;
    double maxExtensionLength = 3.0;
    unicycle_lyapunov_distance_params_t distanceParams;
    unicycle_lyapunov_steering_params_t steeringParams;
    unicycle_pose_sampler_params_t samplerParams;
};


struct unicycle_rrt_star_found_path_info_t
{
    bool haveFoundPath;
    bool numPosesSampled;
    std::vector<Point<float>> path;
    std::vector<std::pair<pose_t, bool>> waypoints;
    std::vector<std::pair<UnicycleLyapunovDistance, bool>> manifold;
};


class UnicycleRRTStar
{
public:
    using unicyclePath = std::vector<Point<float>>;
    using unicycleWaypoints = std::vector<std::pair<pose_t, bool>>;

    UnicycleRRTStar(void){};

    /**
     * Constructor for UnnicycleRRTStar
     *
     * \param    startPose       pose for the root node
     * \param    goalPose        goal pose
     * \param    grid            obstacle distance grid
     * \param    params          relevant parameters
     */
    UnicycleRRTStar(const pose_t& startPose,
                    const pose_t& goalPose,
                    const ObstacleDistanceGrid& grid,
                    const unicycle_rrt_star_params_t& params);

    //     void resetStartPose(const pose_t& startPose);
    //     void resetGoalPose (const pose_t& goalPose);
    void resetObstacleGridMap(const ObstacleDistanceGrid& grid);

    // Path-building methods
    // attempt to find a path within max allowed number of samples. Terminates as soon as a path is found.
    bool findNewPath(size_t maxNumSamples, bool canTerminateEarly = true);

    // attempt to improve path with max allowed number of additional samples. Terminates as soon as a path is found.
    bool improvePath(size_t maxAdditionalSamples, bool canTerminateEarly = true);

    // attempt to imporve path with fixed number of samples. return true if the path has been improved.
    bool addSamples(size_t numSamplesToAdd);

    // getters for retrieving various description of the planned path. Will return empty vector if no path has been
    // found.
    bool haveFoundPath(void) const { return haveFoundPath_; };
    unicyclePath getPlannedPath(void) const { return plannedPath_; };
    unicycleWaypoints getPlannedWaypoints(void) const { return plannedWaypoints_; };

    // getters for planner info
    std::size_t getNumNodes(void) { return RRT_.tree_.size(); };
    std::list<unicycle_rrt_node_data_t> getGraphInfo(void);

    // chage planner params
    void sampleBackwardMotion(bool tf) { isSamplingBackwardMotion_ = tf; };

private:
    // Sampling
    // sample a pose within a map
    pose_t rrtSample(bool* isGoal, bool* isMovingBackward) const;


    // Steering
    // extend path from a node to a sample, up to specified length. used for rrt-exploration
    unicycle_path_segment_t extendTo(const unicycle_rrt_node_t& rrtNode,
                                     const pose_t& sampledPose,
                                     double maxExtension,
                                     bool isMovingBackward = false) const;

    // find path from sampled pose to a given node. used for rrt-star-rewire
    unicycle_path_segment_t steerTo(const pose_t& sampledPose, const unicycle_rrt_node_t& rrtNode) const;

    // check if a given path is obstacle-free in a given map
    bool obstacleFree(const std::vector<pose_t>& steps, const ObstacleDistanceGrid& map) const;
    //     bool obstacleFree(const std::vector<pose_t>& steps, const ObstacleDistanceGrid& grid, const
    //     RobotCollisionModel& robotShape); // to be implemented


    // Neighbor searches
    // computing neighborhood radius
    double neighborhoodRadius(bool isMovingBackward);

    // find a node nearest *to* a given sample, excluding goal nodes.
    unicycle_rrt_node_t* nodeNearestTo(const pose_t& sampledPose, bool isMovingBackward = false);

    // find nodes near *to* a given pose
    // internal: neighborhoodRadius
    std::vector<unicycle_rrt_node_t*> nodesNearTo(const pose_t& sampledPose, bool isMovingBackward = false);

    // find nodes near *from* a given pose
    // internal: neighborhoodRadius
    std::vector<unicycle_rrt_node_t*> nodesNearFrom(const pose_t& sampledPose, bool isMovingBackward = false);


    // Cost definitions
    double
      rrtCostFromNodeToPose(const unicycle_rrt_node_t& fromNode, const pose_t& toPose, bool isMovingBackward = false);
    double rrtCostFromPoseToNode(const pose_t& fromPose, const unicycle_rrt_node_t& toNode);

    double rrtCostToGoViaNode(const pose_t& robotPose,
                              const unicycle_rrt_node_t& node);   // works only if a path has been found

    // to implement ?
    //     double rrtCostFromNodeToPose(const unicycle_path_segment_t& directedEdge);
    //     double rrtCostFromPoseToNode(const unicycle_path_segment_t& directedEdge);


    // RRT-Star methods (mutators)
    // choose parent from the set of pose nearby, using the initial guess
    unicycle_rrt_node_t* rrtChooseParent(const std::vector<unicycle_rrt_node_t*>& nodesNearTo,
                                         unicycle_rrt_node_t* nearestNode,
                                         const pose_t& samplePose,
                                         bool isMovingBackward = false);

    // insert node and connect to the parent. returns pointer to the inserted node.
    unicycle_rrt_node_t* rrtInsertNode(unicycle_rrt_node_t* parentNode,
                                       const pose_t& childPose,
                                       bool isGoal,
                                       bool isMovingBackward = false);

    // rewire graph, considering all nearby nodes
    void rrtRewire(const std::vector<unicycle_rrt_node_t*>& nearbyNodes, unicycle_rrt_node_t* rewireNode);

    // re-connect a child to a parent
    void rrtReconnect(unicycle_rrt_node_t* newParent, unicycle_rrt_node_t* childToRewire, double rewiredEdgeCost);

    // propagte cost from a parent to all children down the graph
    void rrtPropagateCostToChildren(unicycle_rrt_node_t* parentNode);


    // Tree manager
    // branch-and-bound to streamline the graph
    bool branchAndBound(int* numNodesPruned);   // TODO: implement this!

    bool retrievePathFromLeaf(unicycle_rrt_node_t* leafNode);

    // clear data
    void clearRRTStats(void);
    void clearFoundPaths(void);

    // base information required for planning
    pose_t startPose_;
    pose_t goalPose_;
    ObstacleDistanceGrid grid_;   // currently assume fixed map. will implement efficient reuse of information in case
                                  // of partial map change later.

    // sampler and performance indicators
    size_t numSamples_;
    double totalMilliSecondsSpent_;
    double averageMilliSecondPerSample_;   // running average
    UnicyclePoseSampler poseSampler_;

    // steering
    UnicycleLyapunovSteering steering_;

    // rrt-star graph
    UnicycleRRTTree RRT_;

    // path indicators
    bool haveFoundPath_;
    double minimumCostToGoal_;
    unicycle_rrt_node_t* minimumCostLeaf_;
    std::vector<unicycle_rrt_node_t*> goalLeaves_;

    // various representation of the planned path (i.e. the best path found)
    std::vector<unicycle_rrt_node_data_t> pathNodes_;   // a series of rrt nodes leading to the goal
    std::vector<Point<float>> plannedPath_;             // a series of 2D points which represents a path on a plane.
    std::vector<std::pair<pose_t, bool>>
      plannedWaypoints_;   // a series of waypoints and approach direction. i.e. <pose, isBackward> from start to goal
    //     std::vector<std::pair<UnicycleLyapunovDistance, bool>> plannedManifold_;  // a series of lyapunov functions
    //     anchored around each waypoint on the path.

    // other options
    bool isSamplingBackwardMotion_;

    // parameters
    unicycle_rrt_star_params_t params_;
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // UNICYCLE_RRT_STAR_H

//
//
//
//     unicycle_lyapunov_distance_params_t distanceParams_;
//     unicycle_lyapunov_steering_params_t steeringParams_;
//
//     std::list<unicycle_rrt_node_t> tree_;
// };

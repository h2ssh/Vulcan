/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


// /**
// * \file     rrt_renderer.h
// * \author   Collin Johnson
// *
// * Declaration of RRTRenderer.
// */
// 
// #ifndef UI_COMPONENTS_RRT_RENDERER_H
// #define UI_COMPONENTS_RRT_RENDERER_H
// 
// #include <vector>
// #include "ui/common/ui_color.h"
// #include "ui/components/robot_trajectory_renderer.h"
// 
// namespace vulcan
// {
// namespace math { template <typename T> class Rectangle; }
// pose_t
// namespace mpepc
// {
//     class  DebugRRT;
//     struct rrt_info_t;
//     struct debug_rrt_node_t;
//     struct debug_rrt_edge_t;
// }
// 
// namespace ui
// {
// 
// /**
// * RRTRenderer handles the task of rendering the RRT and associated information. The edges of
// * the RRT are drawn as curved lines following the simulated trajectories. The nodes are triangles
// * pointing in the direction of the pose at the node. The goal region is a translucent rectangle
// * The start and target are drawn larger and in different colors. The final path is drawn in a
// * separate color from the rest of the tree.
// */
// class RRTRenderer
// {
// public:
// 
//     /**
//     * Constructor for RRTRenderer.
//     *
//     * Defaults to rendering everything.
//     */
//     RRTRenderer(void);
// 
//     /**
//     * setRenderColors sets the colors to use for rendering the RRT.
//     */
//     void setRenderColors(const GLColor& edgeColor,
//                          const GLColor& nodeColor,
//                          const GLColor& startColor,
//                          const GLColor& targetColor,
//                          const GLColor& goalPathColor,
//                          const GLColor& goalRegionColor);
// 
//     // Flags for turning on/off the various pieces of rendered information
//     void showRRT           (bool show) { shouldDrawRRT        = show; }
//     void showNodes         (bool show) { shouldDrawNodes      = show; }
//     void showEdges         (bool show) { shouldDrawEdges      = show; }
//     void showStartAndTarget(bool show) { shouldDrawStart      = show; }
//     void showGoalPath      (bool show) { shouldDrawPath       = show; }
//     void showGoalRegion    (bool show) { shouldDrawGoalRegion = show; }
// 
//     // Do the actual drawing based on the configured
//     void renderRRT(const mpepc::rrt_info_t& rrtInfo);
// 
// private:
// 
//     void drawRRT           (const mpepc::DebugRRT& rrt);
//     void drawNodes         (const std::vector<mpepc::debug_rrt_node_t>& nodes);
//     void drawEdges         (const std::vector<mpepc::debug_rrt_edge_t>& edges);
//     void drawStartAndTarget(const pose_t& start, const pose_t& target);
//     void drawGoalPath      (const std::vector<pose_t>& path);
//     void drawGoalRegion    (const math::Rectangle<float>& region);
// 
//     RobotTrajectoryRenderer trajectoryRenderer;
// 
//     bool shouldDrawRRT;
//     bool shouldDrawNodes;
//     bool shouldDrawEdges;
//     bool shouldDrawStart;
//     bool shouldDrawPath;
//     bool shouldDrawGoalRegion;
// 
//     GLColor edgeColor;
//     GLColor nodeColor;
//     GLColor startColor;
//     GLColor targetColor;
//     GLColor goalPathColor;
//     GLColor goalRegionColor;
// };
// 
// }
// }
// 
// #endif // UI_COMPONENTS_RRT_RENDERER_H

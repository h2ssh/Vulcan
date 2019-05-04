/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


// /**
// * \file     rrt_renderer.cpp
// * \author   Collin Johnson
// *
// * Definition of RRTRenderer.
// */
// 
// #include <ui/components/rrt_renderer.h>
// #include <math/geometry/rectangle.h>
// #include <GL/gl.h>
// #include <ui/common/gl_shapes.h>
// #include <mpepc/rrt/rrt.h>
// 
// namespace vulcan
// {
// namespace ui
// {
// 
// const float NODE_WIDTH   = 0.2f;
// const float NODE_HEIGHT  = 0.3f;
// const float START_WIDTH  = 0.5f;
// const float START_HEIGHT = 0.75f;
// 
// 
// RRTRenderer::RRTRenderer(void)
// : shouldDrawRRT(true)
// , shouldDrawNodes(true)
// , shouldDrawEdges(true)
// , shouldDrawStart(true)
// , shouldDrawPath(true)
// , shouldDrawGoalRegion(true)
// {
// }
// 
// 
// void RRTRenderer::setRenderColors(const GLColor& edgeColor,
//                                   const GLColor& nodeColor,
//                                   const GLColor& startColor,
//                                   const GLColor& targetColor,
//                                   const GLColor& goalPathColor,
//                                   const GLColor& goalRegionColor)
// {
//     this->edgeColor       = edgeColor;
//     this->nodeColor       = nodeColor;
//     this->startColor      = startColor;
//     this->targetColor     = targetColor;
//     this->goalPathColor   = goalPathColor;
//     this->goalRegionColor = goalRegionColor;
// }
// 
// 
// void RRTRenderer::renderRRT(const mpepc::rrt_info_t& rrtInfo)
// {
//     // Order here chosen for a reason, leave it this way
//     if(shouldDrawGoalRegion)
//     {
// //         drawGoalRegion(rrtInfo.goalRegion);
//     }
// 
//     if(shouldDrawRRT)
//     {
// //         drawRRT(rrtInfo.rrt);
//     }
// 
//     if(shouldDrawStart)
//     {
// //         drawStartAndTarget(rrtInfo.start, rrtInfo.target);
//     }
// 
//     if(shouldDrawPath)
//     {
// //         drawGoalPath(rrtInfo.pathTrajectory);
//     }
// }
// 
// 
// void RRTRenderer::drawRRT(const mpepc::DebugRRT& rrt)
// {
// //     // Draw edges first so the nodes will be on top and thus easier to see
// //     if(shouldDrawEdges)
// //     {
// //         drawEdges(rrt.getEdges());
// //     }
// // 
// //     if(shouldDrawNodes)
// //     {
// //         drawNodes(rrt.getNodes());
// //     }
// }
// 
// 
// void RRTRenderer::drawNodes(const std::vector<mpepc::debug_rrt_node_t>& nodes)
// {
// //     nodeColor.set();
// // 
// //     for(auto nodeIt = nodes.begin(), nodeEnd = nodes.end(); nodeIt != nodeEnd; ++nodeIt)
// //     {
// //         gl_draw_filled_triangle(nodeIt->pose.toPoint(), NODE_WIDTH, NODE_HEIGHT, nodeIt->pose.theta);
// //     }
// }
// 
// 
// void RRTRenderer::drawEdges(const std::vector<mpepc::debug_rrt_edge_t>& edges)
// {
// //     // Set all the colors the same because different colors for collisions or goals don't matter here
// //     trajectoryRenderer.setRenderColors(edgeColor, edgeColor, edgeColor);
// //     
// //     for(auto edgeIt = edges.begin(), edgeEnd = edges.end(); edgeIt != edgeEnd; ++edgeIt)
// //     {
// //         trajectoryRenderer.renderTrajectory(edgeIt->trajectory);
// //     }
// }
// 
// 
// void RRTRenderer::drawStartAndTarget(const pose_t& start, const pose_t& target)
// {
// //     startColor.set();
// //     gl_draw_filled_triangle(start.toPoint(), START_WIDTH, START_HEIGHT, start.theta);
// //     gl_draw_line_triangle(start.toPoint(), START_WIDTH, START_HEIGHT, start.theta, 2.0f);
// // 
// //     targetColor.set();
// //     gl_draw_filled_triangle(target.toPoint(), START_WIDTH, START_HEIGHT, target.theta);
// //     gl_draw_line_triangle(target.toPoint(), START_WIDTH, START_HEIGHT, target.theta, 2.0f);
// }
// 
// 
// void RRTRenderer::drawGoalPath(const std::vector<pose_t>& path)
// {
// //     // Set all the colors the same because different colors for collisions or goals don't matter here
// //     trajectoryRenderer.setRenderColors(goalPathColor, goalPathColor, goalPathColor);
// //     trajectoryRenderer.renderTrajectory(path);
// }
// 
// 
// void RRTRenderer::drawGoalRegion(const math::Rectangle<float>& region)
// {
// //     goalRegionColor.set();
// // 
// //     gl_draw_filled_rectangle(region);
// //     gl_draw_line_rectangle(region, 2.0f);
// }
// 
// } // namespace ui
// } // namespace vulcan

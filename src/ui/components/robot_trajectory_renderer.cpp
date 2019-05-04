/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     robot_trajectory_renderer.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of RobotTrajectoryRenderer.
*/

#include <ui/components/robot_trajectory_renderer.h>
#include <ui/components/robot_renderer.h>
#include <ui/common/gl_shapes.h>
#include <core/pose.h>
#include <mpepc/control/control_law_coordinates.h>
#include <cassert>
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{    


RobotTrajectoryRenderer::RobotTrajectoryRenderer(void)
: poseVertexArray_(nullptr)
, costVertexArray_(nullptr)
, costColorArray_ (nullptr)
, numPoseVertices_(0)
, numCostVertices_(0)
{    
}


void RobotTrajectoryRenderer::setRenderColor(const GLColor& defaultColor)
{
    this->defaultColor_ = defaultColor;
}


void RobotTrajectoryRenderer::resetRenderer(void)
{
    // Get rid of the things we aren't using for efficiency sake.
    // These client states are disabled by default, but this is for 
    // when something else enabled it and did not disable them.
    glDisableClientState(GL_EDGE_FLAG_ARRAY);
    glDisableClientState(GL_INDEX_ARRAY);
    glDisableClientState(GL_SECONDARY_COLOR_ARRAY);
    glDisableClientState(GL_FOG_COORDINATE_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    
//     // Clear memory. (I probably don't need this.)
//     delete [] poseVertexArray_;
//     delete [] costVertexArray_;
//     delete [] costColorArray_;
//     
//     numPoseVertices_ = 0;
//     numCostVertices_ = 0;
}


void RobotTrajectoryRenderer::renderTrajectory(const std::vector<pose_t>& trajectory)
{
    renderTrajectory(trajectory, defaultColor_);
}


void RobotTrajectoryRenderer::renderTrajectory(const std::vector<pose_t>& trajectory, const GLColor& color)
{
    renderTrajectory(trajectory, color, 2.0);
}


void RobotTrajectoryRenderer::renderTrajectory(const std::vector<pose_t>& trajectory, const GLColor& color, float lineWidth)
{
    // allocate vertex array if needed
    allocatePoseArrayIfNeeded(trajectory.size());
    
    // populate vertex array
    for(size_t n = 0; n < trajectory.size(); ++n)
    {
        poseVertexArray_[2*n]     = trajectory[n].x;
        poseVertexArray_[2*n + 1] = trajectory[n].y;
    }
    
    // set width and color
    glLineWidth(lineWidth);
    color.set();
    
    // drawing the trajectory 
    glEnableClientState(GL_VERTEX_ARRAY);
    
    glVertexPointer(2, GL_FLOAT, 0, poseVertexArray_);
    glDrawArrays(GL_LINE_STRIP, 0, trajectory.size());
    
    glDisableClientState(GL_VERTEX_ARRAY);
}


void RobotTrajectoryRenderer::renderPath(const std::vector<Point<float>>& path, const GLColor& pathColor, float lineWidth)
{
    // allocate vertex array if needed
    allocatePointArrayIfNeeded(path.size());
    
    // populate vertex array
    for(size_t n = 0; n < path.size(); ++n)
    {
        pointVertexArray_[2*n]     = path[n].x;
        pointVertexArray_[2*n + 1] = path[n].y;
    }
    
    // set width and color
    glLineWidth(lineWidth);
    pathColor.set();
    
    // drawing the path 
    glEnableClientState(GL_VERTEX_ARRAY);
    
    glVertexPointer(2, GL_FLOAT, 0, pointVertexArray_);
    glDrawArrays(GL_LINE_STRIP, 0, path.size());
    
    glDisableClientState(GL_VERTEX_ARRAY);
}


void RobotTrajectoryRenderer::renderPiecewiseCosts(const std::vector<pose_t>& trajectory, const std::vector<float>& piecewiseCosts)
{
    size_t numPoints   = trajectory.size();
    size_t numSegments = piecewiseCosts.size();
    
    assert(numSegments < numPoints);
    
    size_t interval = numPoints/numSegments;
    
    allocateCostArrayIfNeeded(numSegments + 1);
    
    for(size_t segmentIndex = 0, pointIndex = 0; segmentIndex <= numSegments; ++segmentIndex, pointIndex += interval)
    {
        costVertexArray_[2*segmentIndex]     = trajectory[pointIndex].x;
        costVertexArray_[2*segmentIndex + 1] = trajectory[pointIndex].y;
        
        // selecting trajectory colors. Low values will be displayed in green, and high values will be in red.
        if(segmentIndex == 0)
        {
            // for the first vertex copy the value from the 2nd vertex,
            // which is the first segment endpoint where the costs are computed.
            costColorArray_[4*segmentIndex]     = piecewiseCosts[segmentIndex];        // red
            costColorArray_[4*segmentIndex + 1] = 1.0f - piecewiseCosts[segmentIndex]; // green
            costColorArray_[4*segmentIndex + 2] = 0.0f;                                // blue
            costColorArray_[4*segmentIndex + 3] = 100.0f/255.0f;                       // alpha
        }
        else
        {
            // normal computation from the second vertex till the end.
            costColorArray_[4*segmentIndex]     = piecewiseCosts[segmentIndex-1];         // red
            costColorArray_[4*segmentIndex + 1] = 1.0f - piecewiseCosts[segmentIndex-1];  // green
            costColorArray_[4*segmentIndex + 2] = 0.0f;                                   // blue
            costColorArray_[4*segmentIndex + 3] = 100.0f/255.0f;                          // alpha
        }
    }
    
    glLineWidth(2.0);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(2, GL_FLOAT, 0, costVertexArray_);
    glColorPointer (4, GL_FLOAT, 0, costColorArray_);

    glDrawArrays(GL_LINE_STRIP, 0, numSegments + 1);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

}


void RobotTrajectoryRenderer::renderRobotsOverTrajectory(const std::vector<pose_t>& trajectory, const RobotRenderer& robotRenderer, uint numRobots) const
{
    if(numRobots <= 0)
    {
        return; // do nothing if there is nothing to draw
    }
    
    size_t numPoses = trajectory.size();
        
    if(numPoses == 0)
    {
        return; // do nothing if there is no trajectory to draw over
    }
    
    size_t interval = numPoses/numRobots;
    
    // draw robots over the trajectory, from the endpoint of the trajectory
    // toward the beginning, and not drawing at the first element.
    for(size_t i = numPoses - 1; i > 0; i -= interval)
    {
        robotRenderer.renderBoundary(trajectory[i]); // using default color
    }
}


void RobotTrajectoryRenderer::renderMotionTarget(const mpepc::motion_target_t& motionTarget) const
{
    // this will use whatever color that was set previously.
    gl_draw_filled_triangle(motionTarget.pose.toPoint(), 0.2, 0.3, motionTarget.pose.theta);
}


void RobotTrajectoryRenderer::renderMotionTarget(const mpepc::motion_target_t& motionTarget, const GLColor& motionTargetColor) const
{
    renderTriangle(motionTarget.pose.toPoint(), motionTarget.pose.theta, motionTargetColor);
}


void RobotTrajectoryRenderer::renderTriangle(const Point<float>& point, float orientation, const GLColor& color) const
{
    // this will use whatever color that was set previously.
    color.set();
    gl_draw_filled_triangle(point, 0.2, 0.3, orientation);
}


void RobotTrajectoryRenderer::allocatePointArrayIfNeeded(int size)
{
    if(size != numPointVertices_)
    {
        delete [] pointVertexArray_;

        numPointVertices_ = size;
        pointVertexArray_ = new float[numPointVertices_*2];
    }
}


void RobotTrajectoryRenderer::allocatePoseArrayIfNeeded(int size)
{
    if(size != numPoseVertices_)
    {
        delete [] poseVertexArray_;

        numPoseVertices_ = size;
        poseVertexArray_ = new float[numPoseVertices_*2];
    }
}


void RobotTrajectoryRenderer::allocateCostArrayIfNeeded(int size)
{
    if(size != numCostVertices_)
    {
        delete [] costVertexArray_;
        delete [] costColorArray_;

        numCostVertices_  = size;
        costVertexArray_  = new float[numCostVertices_*2];
        costColorArray_   = new float[numCostVertices_*4];
    }
}


}
}

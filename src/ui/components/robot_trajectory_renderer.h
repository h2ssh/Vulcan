/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     robot_trajectory_renderer.h
* \author   Collin Johnson and Jong Jin Park
*
* Declaration of RobotTrajectoryRenderer.
*/

#ifndef UI_COMPONENTS_ROBOT_TRAJECTORY_RENDERER_H
#define UI_COMPONENTS_ROBOT_TRAJECTORY_RENDERER_H

#include "ui/common/ui_color.h"
#include "core/point.h"
#include <vector>

namespace vulcan
{
struct pose_t;

namespace mpepc { struct motion_target_t; }

namespace ui
{
    
class RobotRenderer;

/**
* RobotTrajectoryRenderer renders a trajectory from a vector of poses. The trajectory
* is rendered as a line strip. The various properties of a trajectory will adjust its
* rendered color.
*/
class RobotTrajectoryRenderer
{
public:

    /**
    * Constructor for RobotTrajectoryRenderer.
    */
    RobotTrajectoryRenderer(void);
    
    ~RobotTrajectoryRenderer(void) {};
    
    /**
    * setRenderColor sets the default color to use for rendering the trajectory.
    *
    * \param    defaultColor       Predefined default color profile
    */
    void setRenderColor(const GLColor& defaultColor);
    
    void resetRenderer(void);
    
    void renderTrajectory(const std::vector<pose_t>& trajectory);
    void renderTrajectory(const std::vector<pose_t>& trajectory, const GLColor& trajectoryColor);
    void renderTrajectory(const std::vector<pose_t>& trajectory, const GLColor& trajectoryColor, float lineWidith);
    
    void renderPath(const std::vector<Point<float>>& path, const GLColor& pathColor, float lineWidth = 1.5);
    
    void renderPiecewiseCosts(const std::vector<pose_t>& trajectory, const std::vector<float>& piecewiseCosts);
    
    void renderRobotsOverTrajectory(const std::vector<pose_t>& trajectory, const RobotRenderer& robotRenderer, uint numRobots = 5) const;
    
    void renderMotionTarget(const mpepc::motion_target_t& motionTarget) const;
    void renderMotionTarget(const mpepc::motion_target_t& motionTarget, const GLColor& motionTargetColor) const;
    
    void renderTriangle(const Point<float>& point, float orientation, const GLColor& triangleColor) const;
    
private:
    
    void allocatePointArrayIfNeeded(int size);
    void allocatePoseArrayIfNeeded(int size);
    void allocateCostArrayIfNeeded(int size);
    
    GLColor defaultColor_;
    
    float* pointVertexArray_;
    float* poseVertexArray_;
    float* costVertexArray_;
    float* costColorArray_;
    
    int numPointVertices_;
    int numPoseVertices_;
    int numCostVertices_;
};

}
}

#endif // UI_COMPONENTS_ROBOT_TRAJECTORY_RENDERER_H

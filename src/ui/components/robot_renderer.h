/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     robot_renderer.h
* \author   Collin Johnson and Jong Jin Park
*
* Declaration of RobotRenderer.
*/

#ifndef UI_COMPONENTS_ROBOT_RENDERER_H
#define UI_COMPONENTS_ROBOT_RENDERER_H

#include <ui/common/ui_color.h>
#include <core/point.h>
#include <math/geometry/circle.h>
#include <math/geometry/rectangle.h>
#include <math/geometry/polygon.h>
#include <vector>

namespace vulcan
{
struct pose_t;
namespace robot{ struct collision_model_params_t; }

namespace ui
{

/**
* RobotRenderer draws a robot on the screen at the given position as a rectangle with fixed size.
* Can also draw bounding boxes of various size around the robot when specified.
*/
class RobotRenderer
{
public:

    RobotRenderer(void);

    void setRobotColor(const GLColor& defaultColor);
    void setRobotShape(const robot::collision_model_params_t& params);

    void renderRobot(const pose_t& robotPose) const;
    void renderRobot(const pose_t& robotPose, const GLColor& robotColor) const;

    void renderBoundary(const pose_t& robotPose) const;
    void renderBoundary(const pose_t& robotPose, const GLColor& edgeColor) const;

    void renderBoundingBox(const pose_t& robotPose, float boxOffset) const;
    void renderBoundingBox(const pose_t& robotPose, float boxOffset, const GLColor& boxColor) const;

private:

    enum model_t
    {
        rectangle,
        polygon,
        circle,
    };

    // defalut robot color
    GLColor defaultColor_;

    // other robot shapes from parameter file
    model_t modelToDraw_;
    math::Circle<float> circleModel_;
    math::Rectangle<float> rectangleModel;
    math::Polygon<float>   convexPolygonModel;
};

}
}

#endif // UI_COMPONENTS_ROBOT_RENDERER_H

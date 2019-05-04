/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     robot_renderer.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of RobotRenderer.
*/

#include <ui/components/robot_renderer.h>
#include <ui/common/gl_shapes.h>
#include <ui/common/default_colors.h>
#include <robot/model/params.h>
#include <core/pose.h>
#include <core/point.h>
#include <utils/config_file.h>
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

RobotRenderer::RobotRenderer(void)
: defaultColor_(robot_color())
{
    utils::ConfigFile robotModelConfig(robot::kRobotModelConfigFilename);
    robot::collision_model_params_t params(robotModelConfig);
    setRobotShape(params);
}

void RobotRenderer::setRobotColor(const GLColor& defaultColor)
{
    defaultColor_ = defaultColor;
}

void RobotRenderer::setRobotShape(const robot::collision_model_params_t& params)
{
    if(params.type == "polygon")
    {
        modelToDraw_ = polygon;
        convexPolygonModel = params.convexPolygonModel;
    }
    else if(params.type == "rectangle")
    {
        modelToDraw_ = rectangle;
        rectangleModel = params.rectangleModel;
    }
    else if(params.type == "circle")
    {
        modelToDraw_ = circle;
        circleModel_ = math::Circle<float>(params.circleModelRadius);
    }
    else
    {
        std::cout<<"DEBUG: RobotRenderer: Robot Shape: "<<params.type<<'\n';
        std::cout<<"ERROR!!: RobotRenderer: Unsupported robot shape!! Check config file. Using default values...\n";

        modelToDraw_ = rectangle;
        rectangleModel = math::Rectangle<float>(Point<float>(-0.53,-0.32), Point<float>(0.55,0.32));
    }
}


void RobotRenderer::renderRobot(const pose_t& robotPose) const
{
    renderRobot(robotPose, defaultColor_);
}


void RobotRenderer::renderRobot(const pose_t& robotPose, const GLColor& robotColor) const
{
    // set robot color
    robotColor.set();
    
    float arrowLength = 1.0; // default length for the line indicating the front of the robot
    
    glPushMatrix();
    
    // set robot center
    glTranslatef(robotPose.x, robotPose.y, 0.0);
    glRotatef(robotPose.theta*180.0f/M_PI, 0.0, 0.0, 1.0);
    
    // draw body
    switch(modelToDraw_)
    {
    case polygon:
        gl_draw_filled_polygon(convexPolygonModel.vertices());
        arrowLength = convexPolygonModel.begin()->x; // this assumes the first vertex points to the front.
        break;
    case rectangle:
        gl_draw_filled_rectangle(rectangleModel);
        arrowLength = rectangleModel.topRight.x;
        break;
    case circle:
        gl_draw_filled_circle(circleModel_);
        arrowLength = circleModel_.radius();
    }
    
    // draw heading indicator
    glLineWidth(3.0);
    glColor4f(0.0, 0.0, 0.0, robotColor.alpha());
    
    glBegin(GL_LINES);
    glVertex2f(0.0f, 0.0f);
    glVertex2f(arrowLength, 0.0f);
    glEnd();

    glPopMatrix();
}


void RobotRenderer::renderBoundary(const pose_t& robotPose) const
{
    renderBoundary(robotPose, defaultColor_);
}


void RobotRenderer::renderBoundary(const pose_t& robotPose, const GLColor& edgeColor) const
{
    // set edge color and width
    edgeColor.set();
    float lineWidth = 2.0;
    
    glPushMatrix();
    
    // set robot center
    glTranslatef(robotPose.x, robotPose.y, 0.0);
    glRotatef(robotPose.theta*180.0f/M_PI, 0.0, 0.0, 1.0);
        
    // draw boundary
    switch(modelToDraw_)
    {
    case polygon:
        gl_draw_line_polygon(convexPolygonModel.vertices(), lineWidth);
        break;
    case rectangle:
        gl_draw_line_rectangle(rectangleModel, lineWidth);
        break;
    case circle:
        gl_draw_line_circle(circleModel_, lineWidth);
    }
    
    glPopMatrix();
}


void RobotRenderer::renderBoundingBox(const pose_t& robotPose, float boxOffset) const
{
    renderBoundingBox(robotPose, boxOffset, defaultColor_);
}


void RobotRenderer::renderBoundingBox(const pose_t& robotPose, float boxOffset, const GLColor& boxColor) const
{
    const float kLineWidth = 2.0f;

    boxColor.set();

    glPushMatrix();
    
    // set robot pose
    glTranslatef(robotPose.x, robotPose.y, 0.0);
    glRotatef(robotPose.theta*180.0f/M_PI, 0.0, 0.0, 1.0);
    
    switch(modelToDraw_)
    {
    case polygon:
        glLineWidth(kLineWidth);
        glBegin(GL_LINE_LOOP);
        for(auto vertexIt = convexPolygonModel.begin(); vertexIt != convexPolygonModel.end(); vertexIt++)
        {
            // FIXME: This is computation is wrong. To do it properly, for every line segment the normal would
            // need to be computed and and the vertices needs to be pushed in that direction, thus creating
            // twice as many vertices as it started out with. But it would still be incorrect though,
            // since it needs to diplay arcs between new vertices which is the distance from the corner.
            // (All of this assumes convex polygon.)
            
            // below (kindof) works in that it gives you correct answers for lines colinear with x- and y- axes,
            // and otherwise gives you much larger bounding box. I might fix this if I find extra free time.
            float newVertexX = (fabs(vertexIt->x) <= 0.01) ? vertexIt->x : vertexIt->x + copysign(boxOffset, vertexIt->x);
            glVertex2f(newVertexX, vertexIt->y + copysign(boxOffset, vertexIt->y));
        }
        glEnd();
        break;
    case rectangle:
        {
            math::Rectangle<float> expanded(Point<float>(rectangleModel.bottomLeft.x - boxOffset, rectangleModel.bottomLeft.y - boxOffset),
                                            Point<float>(rectangleModel.topRight.x   + boxOffset, rectangleModel.topRight.y   + boxOffset));
            gl_draw_line_rectangle(expanded, kLineWidth);
        }
        break;

    case circle:
        gl_draw_line_circle(math::Circle<float>(circleModel_.radius() + boxOffset));
        break;
    }

    glPopMatrix();
}

} // namespace ui
} // namespace vulcan

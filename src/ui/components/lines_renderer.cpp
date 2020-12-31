/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <GL/gl.h>
#include <GL/glu.h>
#include "core/pose.h"
#include "ui/components/lines_renderer.h"


using vulcan::ui::LinesRenderer;


void LinesRenderer::setRenderColor(const GLColor& color)
{
    lineColor = color;
}


void LinesRenderer::renderLines(const std::vector<Line<float>>& lines, const pose_t& globalRobotPose, float lineWidth)
{
    glPushMatrix();
    
    glTranslatef(globalRobotPose.x, globalRobotPose.y, 0.0);
    glRotatef(globalRobotPose.theta*180.0f/M_PI, 0.0, 0.0, 1.0);
    
    glLineWidth(lineWidth);
    glBegin(GL_LINES);
    
    lineColor.set();
    
    for(size_t i = 0; i < lines.size(); ++i)
    {
        glVertex3f(lines[i].a.x, lines[i].a.y, 0);
        glVertex3f(lines[i].b.x, lines[i].b.y, 0);
    }
    
    glEnd();
    
    glPopMatrix();
}

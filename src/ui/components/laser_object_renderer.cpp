/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     laser_object_renderer.cpp
* \author   Collin Johnson
*
* Definition of LaserObjectRenderer.
*/

#include "ui/components/laser_object_renderer.h"
#include "ui/components/object_boundary_renderer.h"
#include "ui/common/color_generator.h"
#include "ui/common/gl_shapes.h"
#include "tracker/laser_object_collection.h"
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

const float kLineWidth = 2.0f;


void LaserObjectRenderer::renderObjects(const tracker::LaserObjectCollection& objects, 
                                        tracker::BoundaryType boundary,
                                        int options)
{
    auto objectColors = generate_colors(objects.laserId()+1, 0.8, false);
    
    for(auto& o : objects)
    {
        renderObject(o, boundary, objectColors[objects.laserId()], options);
    }
}


void LaserObjectRenderer::renderObject(const tracker::LaserObject& object, 
                                       tracker::BoundaryType boundary,
                                       const GLColor& color, 
                                       int options)
{
    activeColor = color;

    if(options & kShowUncertainty)
    {
        drawUncertainty(object);
    }

    drawBoundary(object.boundaryWithType(boundary), options);

    if(options & kShowPoints)
    {
        drawPoints(object);
    }
}


void LaserObjectRenderer::drawBoundary(const tracker::ObjectBoundary& boundary, int options)
{
    // Only fill in the circle if not already drawing the uncertainty ellipse -- doing both
    // would be confusing
    if(~options & kShowUncertainty)
    {
        boundary.visitShape(ObjectBoundaryRenderer(activeColor));
    }
    else
    {
        boundary.visitShape(OutlineObjectBoundaryRenderer(activeColor));
    }
}


void LaserObjectRenderer::drawUncertainty(const tracker::LaserObject& object)
{
    gl_draw_gaussian_distribution(object.centerWithUncertainty(), 1.0, activeColor, kLineWidth);
}


void LaserObjectRenderer::drawPoints(const tracker::LaserObject& object)
{
    activeColor.set();
    glPointSize(2.0);

    glBegin(GL_POINTS);
    for(auto point : object)
    {
        glVertex2f(point.x, point.y);
    }
    glEnd();
}
    
} // namespace ui
} // namespace vulcan

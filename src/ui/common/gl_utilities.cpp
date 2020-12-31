/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "ui/common/gl_utilities.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <cmath>

namespace vulcan
{
namespace ui
{


void setup_opengl_context(void)
{
    // set the viewport for doing the proper display
    glShadeModel(GL_SMOOTH);   // use flat shading for now
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}


void set_gl_viewport(std::size_t widthInPixels, std::size_t heightInPixels)
{
    glClearColor(1.0, 1.0, 1.0, 1.0);   // clear to white screen
    glShadeModel(GL_SMOOTH);            // use flat shading for now

    glViewport(0, 0, (GLsizei)widthInPixels, (GLsizei)heightInPixels);
}


void set_projection(std::size_t widthInPixels, std::size_t heightInPixels, float objectSize, float distance, float zoom)
{
    float fov = 2.0 * std::atan2(zoom * objectSize / 2.0, distance) * (180.0 / M_PI);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (GLfloat)widthInPixels / (GLfloat)heightInPixels, 1.0, 10.0);
}


void set_camera_position(const math::Point3D<float>& cameraFocus)
{
    gluLookAt(cameraFocus.x, cameraFocus.y, cameraFocus.z, cameraFocus.x, cameraFocus.y, 0.0, 0.0, 1.0, 0.0);
}


Point<float> convert_screen_to_world_coordinates(const Point<int>& screenPosition,
                                                 const math::Point3D<float>& cameraPosition)
{
    GLdouble modelview[16];
    GLdouble projection[16];
    GLint viewport[16];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    math::Point3D<double> near;
    math::Point3D<double> far;
    math::Point3D<double> ray;
    Point<float> goal;

    gluUnProject(screenPosition.x, screenPosition.y, 0, modelview, projection, viewport, &near.x, &near.y, &near.z);
    gluUnProject(screenPosition.x, screenPosition.y, 1, modelview, projection, viewport, &far.x, &far.y, &far.z);

    ray.x = far.x - near.x;
    ray.y = far.y - near.y;
    ray.z = far.z - near.z;

    // Now find out where this ray intersects with the plane z = 0. The start of the ray is cameraPos. Intersection
    // parametric: cameraPos.z / ray.z. Then just do cameraPos.x - ray.x * t cameraPos.y - ray.y * t to get the actual
    // intersection
    double t = cameraPosition.z / ray.z;

    goal.x = cameraPosition.x - ray.x * t;
    goal.y = cameraPosition.y - ray.y * t;

    return goal;
}

}   // namespace ui
}   // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/*
* gl_utilities.h contains a number of common OpenGL functions that appear throughout the code.
*/

#ifndef UI_COMMON_GL_UTILITIES_H
#define UI_COMMON_GL_UTILITIES_H

#include <core/point.h>
#include <math/point3d.h>

namespace vulcan
{
namespace ui
{

void setup_opengl_context(void);
void set_gl_viewport(std::size_t widthInPixels, std::size_t heightInPixels);
void set_projection(std::size_t widthInPixels, std::size_t heightInPixels, float objectSize, float distance, float zoom);
void set_camera_position(const math::Point3D<float>& cameraFocus);

Point<float> convert_screen_to_world_coordinates(const Point<int>& screenPosition, const math::Point3D<float>& cameraPosition);

}
}

#endif // UI_COMMON_GL_UTILITIES_H

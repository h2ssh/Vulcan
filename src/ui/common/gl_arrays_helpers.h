/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gl_arrays_helpers.h
 * \author   Collin Johnson
 *
 * Declaration of helper functions for dealing with GL vertex arrays.
 */

#ifndef UI_COMMON_GL_ARRAYS_HELPERS_H
#define UI_COMMON_GL_ARRAYS_HELPERS_H

#include <GL/gl.h>
#include <cstdint>

namespace vulcan
{
namespace ui
{

// Vertex size = number of points per vertex -- either 2 or 3
void draw_gl_array(GLfloat* vertexArray, int vertexSize, GLfloat* scanColors, std::size_t arraySize, GLenum mode);
void draw_gl_array(GLfloat* vertexArray, int vertexSize, std::size_t arraySize, GLenum mode);

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMMON_GL_ARRAYS_HELPERS_H

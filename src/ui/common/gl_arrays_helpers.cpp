/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gl_arrays_helpers.cpp
 * \author   Collin Johnson
 *
 * Definition of helper functions for dealing with vertex arrays.
 */

#include "ui/common/gl_arrays_helpers.h"
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

void draw_gl_array(GLfloat* vertexArray, int vertexSize, GLfloat* vertexColors, std::size_t arraySize, GLenum mode)
{
    // The main bit of lidar data exists in two vector lists for the colors and vertices
    glDisableClientState(GL_EDGE_FLAG_ARRAY);   // get rid of the things we aren't using for efficiency sake
    glDisableClientState(GL_INDEX_ARRAY);
    glDisableClientState(GL_SECONDARY_COLOR_ARRAY);
    glDisableClientState(GL_FOG_COORDINATE_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(vertexSize, GL_FLOAT, 0, vertexArray);
    glColorPointer(4, GL_FLOAT, 0, vertexColors);

    // Transform is to rotate the points and then translate to appropriate global position

    glDrawArrays(mode, 0, arraySize);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    glEnd();
}


void draw_gl_array(GLfloat* vertexArray, int vertexSize, std::size_t arraySize, GLenum mode)
{
    // The main bit of lidar data exists in two vector lists for the colors and vertices
    glDisableClientState(GL_EDGE_FLAG_ARRAY);   // get rid of the things we aren't using for efficiency sake
    glDisableClientState(GL_INDEX_ARRAY);
    glDisableClientState(GL_SECONDARY_COLOR_ARRAY);
    glDisableClientState(GL_FOG_COORDINATE_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(vertexSize, GL_FLOAT, 0, vertexArray);
    glDrawArrays(mode, 0, arraySize);
    glDisableClientState(GL_VERTEX_ARRAY);

    glEnd();
}

}   // namespace ui
}   // namespace vulcan

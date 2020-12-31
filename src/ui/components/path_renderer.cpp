/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     path_renderer.cpp
* \author   Collin Johnson
*
* Definition of PathRenderer.
*/

#include "ui/components/path_renderer.h"
#include "ui/common/default_colors.h"
#include "ui/common/ui_color.h"
#include "mpepc/grid/navigation_grid_utils.h"
#include <boost/range/iterator_range.hpp>
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

void PathRenderer::render(const mpepc::GridPath& path, double metersPerCell)
{
    vertices_.clear();
    vertices_.reserve(path.size() * 4);

    for(auto& pos : boost::make_iterator_range(path.beginPosition(), path.endPosition()))
    {
        vertices_.push_back(pos.x);
        vertices_.push_back(pos.y);

        vertices_.push_back(pos.x + metersPerCell);
        vertices_.push_back(pos.y);

        vertices_.push_back(pos.x + metersPerCell);
        vertices_.push_back(pos.y + metersPerCell);

        vertices_.push_back(pos.x);
        vertices_.push_back(pos.y + metersPerCell);
    }

    dynamic_color().set(0.8);

    glDisableClientState(GL_EDGE_FLAG_ARRAY);  // get rid of the things we aren't using for efficiency sake
    glDisableClientState(GL_INDEX_ARRAY);
    glDisableClientState(GL_SECONDARY_COLOR_ARRAY);
    glDisableClientState(GL_FOG_COORDINATE_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    glEnableClientState(GL_VERTEX_ARRAY);

    glVertexPointer(2, GL_FLOAT, 0, vertices_.data());
    glDrawArrays(GL_QUADS, 0, vertices_.size() / 2);

    glDisableClientState(GL_VERTEX_ARRAY);
    glEnd();
}

} // namespace ui
} // namespace vulcan

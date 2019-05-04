/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     path_renderer.h
* \author   Collin Johnson
*
* Declaration of PathRenderer.
*/

#ifndef UI_COMPONENTS_PATH_RENDERER_H
#define UI_COMPONENTS_PATH_RENDERER_H

#include <vector>

namespace vulcan
{
namespace mpepc { class GridPath; }
namespace ui
{

/**
* PathRenderer draws various types of paths. Currently supported are:
*
*   - GridPath : a path containing a sequence of grid cells
*/
class PathRenderer
{
public:

    /**
    * render draws the provided path.
    *
    * \param    path            Path to render
    * \param    metersPerCell   Scale of cells in the grid
    */
    void render(const mpepc::GridPath& path, double metersPerCell);

private:

    std::vector<float> vertices_;
};

} // namespace ui
} // namespace vulcan

#endif // UI_COMPONENTS_PATH_RENDERER_H

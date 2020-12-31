/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     frontiers_renderer.h
* \author   Collin Johnson
*
* Declaration of FrontiersRenderer.
*/

#ifndef UI_COMPONENTS_FRONTIERS_RENDERER_H
#define UI_COMPONENTS_FRONTIERS_RENDERER_H

#include <vector>
#include "ui/common/ui_color.h"

namespace vulcan
{
namespace hssh { struct Frontier; }

namespace ui
{

/**
* FrontiersRenderer renders the frontiers found in the map onto the screen. Frontiers are drawn as a line
* indicating the boundary of the frontier, and an arrow indicating the direction of the path bounded by
* the frontier.
*/
class FrontiersRenderer
{
public:

    /**
    * setRenderColor sets the color to render the frontiers.
    *
    * \param    frontierColor           Color to use for drawing the frontier
    */
    void setRenderColor(const GLColor& frontierColor);

    /**
    * render renders a collection of frontiers as line segments with direction arrows.
    *
    * \param    frontiers       Frontiers to be rendered
    */
    void render(const std::vector<hssh::Frontier>& frontiers);

private:

    GLColor frontierColor;
};

}
}

#endif // UI_COMPONENTS_FRONTIERS_RENDERER_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_graph_renderer.h
* \author   Collin Johnson
*
* Declaration of AreaGraphRenderer.
*/

#ifndef UI_COMPONENTS_AREA_GRAPH_RENDERER_H
#define UI_COMPONENTS_AREA_GRAPH_RENDERER_H
#include "ui/common/ui_color.h"

namespace vulcan
{
namespace hssh { class AreaGraph; }
namespace hssh { class AreaNode; }
namespace hssh { class AreaEdge; }

namespace ui
{

/**
* AreaGraphRenderer renders an AreaGraph on the screen. The AreaGraph is drawn as circle for
* the nodes with lines connecting them that represent the edges. The nodes are drawn as circles
* with their color representing the type of the node as returned from the node->getType() method.
*/
class AreaGraphRenderer
{
public:

    void setRenderColors(const GLColor& gatewayColor,
                         const GLColor& junctionColor,
                         const GLColor& frontierColor,
                         const GLColor& deadEndColor);

    void renderGraph(const hssh::AreaGraph& graph);

private:

    void drawNode(const hssh::AreaNode& node);
    void drawEdge(const hssh::AreaEdge& edge);

    void setColorFromType(char type, double prob, bool isBorder);

    GLColor gatewayColor;
    GLColor junctionColor;
    GLColor frontierColor;
    GLColor deadEndColor;
};

}
}

#endif // UI_COMPONENTS_AREA_GRAPH_RENDERER_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topological_graph_renderer.h
* \author   Collin Johnson
*
* Declaration of TopologicalGraphRenderer.
*/

#ifndef UI_COMPONENTS_TOPOLOGICAL_GRAPH_RENDERER_H
#define UI_COMPONENTS_TOPOLOGICAL_GRAPH_RENDERER_H

#include <ui/common/ui_color.h>
#include <hssh/global_topological/graph.h>  // not using forward-declaration because it is horridly messy
#include <map>

namespace vulcan
{
namespace ui
{

/**
* TopologicalGraphRenderer renders a TopologicalGraph as circles with lines connecting them. The TopologicalGraph
* represents both GlobalPlaces and GlobalPathSegments as vertices. The different types of vertices are rendered
* as different colors, so they can be distinguished. Individual vertices or edges can be specified to be rendered
* using a specific color that overrides the default setting. Overriding the default color is useful for displaying
* a path through the graph.
*/
class TopologicalGraphRenderer
{
public:

    /**
    * setRenderColors sets the default colors to render the elements of the graph.
    *
    * \param    placeColor          Color to render place vertices
    * \param    pathSegmentColor    Color to render path segment vertices
    * \param    edgeColor           Color to render the edges between vertices
    */
    void setRenderColor(const GLColor& placeColor, const GLColor& pathSegmentColor, const GLColor& edgeColor);

    /**
    * setVertexColor sets the color to render a particular vertex.
    *
    * \param    id          Id of the vertex for which to set the color
    * \param    color       Color to render the vertex
    */
    void setVertexColor(uint32_t id, const GLColor& color);

    /**
    * setEdgeColor sets the color to render a particular edge.
    *
    * \param    id          Id of the edge for which to set the color
    * \param    color       Color to render the edge
    */
    void setEdgeColor(uint32_t id, const GLColor& color);

    /**
    * resetColors resets all colors to their defaults.
    *
    * NOTE: Perhaps individual edges/vertices should be able to be set as well?
    */
    void resetColors(void);

    /**
    * render renders the provided TopologicalGraph.
    */
    void render(const hssh::TopologicalGraph& graph) const;

private:

    void renderEdges   (const std::vector<hssh::TopologicalEdge>&   edges)    const;
    void renderVertices(const std::vector<hssh::TopologicalVertex>& vertices) const;

    GLColor placeColor;
    GLColor pathSegmentColor;
    GLColor edgeColor;

    std::map<uint32_t, GLColor> vertexColors;
    std::map<uint32_t, GLColor> edgeColors;
};

}
}

#endif // UI_COMPONENTS_TOPOLOGICAL_GRAPH_RENDERER_H

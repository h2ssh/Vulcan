/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     visibility_graph_renderer.h
* \author   Collin Johnson
*
* Declaration of VisibilityGraphRenderer.
*/

#ifndef UI_COMPONENTS_VISIBILITY_GRAPH_RENDERER_H
#define UI_COMPONENTS_VISIBILITY_GRAPH_RENDERER_H

#include <ui/common/color_interpolator.h>
#include <ui/common/ui_color.h>
#include <core/point.h>

namespace vulcan
{
namespace utils { class VisibilityGraph; }
namespace utils { class VisibilityGraphFeature; }
namespace ui
{

/**
* VisibilityGraphRenderer draws a visibility graph as circles (nodes) with edges connecting them.
*
* The default colors are:
*   - vertex : red
*   - edge : black
*/
class VisibilityGraphRenderer
{
public:

    /**
    * Constructor for VisibilityGraphRenderer.
    */
    VisibilityGraphRenderer(void);

    /**
    * setVertexColor sets the color to use for drawing a vertex.
    */
    void setVertexColor(GLColor color) { vertexColor_ = color; }

    /**
    * setEdgeColor sets the color to use for drawing an edge.
    */
    void setEdgeColor(GLColor color) { edgeColor_  = color; }

    /**
    * renderVisibilityGraph draws the visibility using the specified colors.
    *
    * The coordinates of the visibility graph are cells, so map coordinate system needs to be provided for
    * transforming to the metric coordinate system used by the renderer.
    */
    void renderVisibilityGraph(const utils::VisibilityGraph& graph,
                               Point<float> globalOrigin,
                               double metersPerCell);


    /**
    * renderVisibilityGraph renders the visibility graph with colors adjusted based on the provided feature. Blue
    * is used for 0.0, green for 0.5, and red for 1.0.
    */
    void renderVisibilityGraph(const utils::VisibilityGraph& graph,
                               const utils::VisibilityGraphFeature& feature,
                               Point<float> globalOrigin,
                               double metersPerCell);

private:

    GLColor vertexColor_;
    GLColor edgeColor_;
    LinearColorInterpolator featureColor_;

    void drawVertices(const utils::VisibilityGraph& graph, Point<float> globalOrigin, double metersPerCell);
    void drawVertices(const utils::VisibilityGraphFeature& graph, Point<float> globalOrigin, double metersPerCell);
    void drawEdges(const utils::VisibilityGraph& graph, Point<float> globalOrigin, double metersPerCell);
};

}
}

#endif // UI_COMPONENTS_VISIBILITY_GRAPH_RENDERER_H

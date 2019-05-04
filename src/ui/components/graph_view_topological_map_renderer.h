/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     graph_view_topological_map_renderer.h
* \author   Collin Johnson
*
* Declaration of GraphViewTopologicalMapRenderer.
*/

#ifndef GRAPH_VIEW_TOPOLOGICAL_MAP_RENDERER_H
#define GRAPH_VIEW_TOPOLOGICAL_MAP_RENDERER_H

#include <ui/components/topological_map_renderer.h>
#include <memory>

namespace vulcan
{
namespace ui
{

class SmallScaleStarRenderer;

/**
* GraphViewTopologicalMapRenderer is a renderer for TopologicalMaps that represents each
* place as a circle and each path as a simple line. The circle representing a place is scaled based on
* the area of the underlying LocalPlace.
*/
class GraphViewTopologicalMapRenderer : public TopologicalMapRenderer
{
public:

    /**
    * Constructor for GraphViewTopologicalMapRenderer.
    */
    GraphViewTopologicalMapRenderer(void);

    ~GraphViewTopologicalMapRenderer(void);

private:

    // Interface for TopologicalMapRenderer
    void renderPlace(const map_place_info_t& place, place_attribute_t attributes);

    std::unique_ptr<SmallScaleStarRenderer> starRenderer_;
};

}
}

#endif // GRAPH_VIEW_TOPOLOGICAL_MAP_RENDERER_H

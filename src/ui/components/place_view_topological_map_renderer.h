/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     place_view_topological_map_renderer.h
* \author   Collin Johnson
*
* Declaration of PlaceViewTopologicalMapRenderer.
*/

#ifndef UI_COMPONENTS_PLACE_VIEW_TOPOLOGICAL_MAP_RENDERER_H
#define UI_COMPONENTS_PLACE_VIEW_TOPOLOGICAL_MAP_RENDERER_H

#include "ui/components/topological_map_renderer.h"
#include "ui/components/occupancy_grid_renderer.h"

namespace vulcan
{
namespace ui
{
class LocalPlaceRenderer;

/**
* PlaceViewTopologicalMapRenderer is a renderer for TopologicalMaps that renders the places
* in the map as the actual LocalPlaces underlying the abstracted topological places. The paths are
* rendered emanating from the correct gateways associated with the LargeScaleStar for the place. Thus, the
* metric connectivity is much more apparent. As the places are each textures, this renderer has a larger
* footprint than the simpler GraphViewTopologicalMapRenderer.
*/
class PlaceViewTopologicalMapRenderer : public TopologicalMapRenderer
{
private:

    // Interface for TopologicalMapRenderer
    void renderPlace(const map_place_info_t& place, place_attribute_t attributes);
    void renderPathSegment(const hssh::GlobalPathSegment& segment,
                           const map_place_info_t&  plusPlace,
                           const map_place_info_t&  minusPlace,
                           path_segment_attribute_t attributes);
    
    OccupancyGridRenderer gridRenderer_;
};

} // namespace ui
} // namespace vulcan

#endif // UI_COMPONENTS_PLACE_VIEW_TOPOLOGICAL_MAP_RENDERER_H

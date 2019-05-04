/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topological_map_renderer.h.h
* \author   Collin Johnson
*
* Declaration of TopologicalMapRenderer.
*/

#ifndef UI_COMPONENTS_TOPOLOGICAL_MAP_RENDERER_H
#define UI_COMPONENTS_TOPOLOGICAL_MAP_RENDERER_H

#include <math/geometry/rectangle.h>
#include <core/pose_distribution.h>
#include <hssh/utils/id.h>
#include <ui/common/ui_color.h>
#include <map>

namespace vulcan
{
namespace hssh
{
    class  GlobalPlace;
    class  MetricMapCache;
    class  LocalPlace;
    class  GlobalPathSegment;
    struct TopologicalState;
    struct GlobalLocation;
}

namespace ui
{

/**
* TopologicalMapRenderer renders a TopologicalState as a series of circles representing
* places with connecting lines representing paths. Frontier path segments are rendered as unterminated lines.
* The current place or path is highlighted.
*/
class TopologicalMapRenderer
{
public:

    /**
    * setRenderColors sets the colors with which to render the topo map.
    *
    * \param    placeColor          Normal place color
    * \param    pathColor           Normal path color
    * \param    locationColor       Color for the current location of the robot in the map
    * \param    frontierColor       Color for frontier paths
    */
    void setRenderColors(const GLColor& placeColor,
                         const GLColor& pathColor,
                         const GLColor& locationColor,
                         const GLColor& frontierColor);

    /**
    * setPlaceColor sets the color to render a particular GlobalPlace.
    *
    * \param    id              Id of the place for which to render a special color
    * \param    color           Color to render the place
    */
    void setPlaceColor(int id, const GLColor& color);

    /**
    * setPathSegmentColor sets the color to render a particular path segment.
    *
    * \param    id              Id of the segment to be rendered a special color
    * \param    color           Color to render the segment
    */
    void setPathSegmentColor(uint32_t id, const GLColor& color);

    /**
    * resetPlaceColor resets the color for a particular place.
    *
    * \param    id              Id of the place whose color is being reset
    */
    void resetPlaceColor(int id);

    /**
    * resetPathSegmentColor resets the color for a particular path segment.
    *
    * \param    id              Unique id of the path segment whose color is being replaced
    */
    void resetPathSegmentColor(uint32_t id);

    /**
    * resetColors resets all special colors so all places in the map are rendered using the default colors.
    */
    void resetColors(void);

    /**
    * calculateRenderedBoundary calculates the boundary of the map to be rendered.
    *
    * \param    topoMap         Map to be rendered
    * \param    places          Place manager from which the LPMs can be loaded
    * \return   Bounding box around the topo map as it will be rendered.
    */
    math::Rectangle<float> calculateRenderedBoundary(const hssh::TopologicalState& topoMap,
                                                     const hssh::MetricMapCache& places);

    /**
    * renderTopoMap renders the provided map using the strategy mentioned in the class description.
    *
    * \param    topoMap         Map to be rendered
    * \param    places          Place manager from which the LPMs can be loaded
    */
    void renderTopoMap(const hssh::TopologicalState& topoMap, const hssh::MetricMapCache& places);

protected:

    enum place_attribute_t
    {
        NORMAL_PLACE,   // Nothing special about the place, just draw it
        TARGET_PLACE,   // Target place for the current path action
        ENTRY_PLACE,    // Entry place for the current path
        CURRENT_PLACE   // Current place in which robot is located
    };

    enum path_segment_attribute_t
    {
        NORMAL_PATH,    // Nothing special, just path connecting a couple places
        CURRENT_PATH,   // Current path on which the robot is located
        FRONTIER_PATH   // Path has no end connection
    };

    struct map_place_info_t
    {
        // NOTE: metric might be nullptr, so it must be checked for validity
        pose_distribution_t referenceFrame;       // reference frame of the place
        const hssh::GlobalPlace* topo = nullptr;
        const hssh::LocalPlace*  metric = nullptr;
    };

    GLColor placeColor;
    GLColor pathColor;
    GLColor locationColor;
    GLColor frontierColor;

    // Interface for subclasses
    virtual void renderPlace(const map_place_info_t& place, place_attribute_t attributes) = 0;

    // Default path segments are rendered as Bezier curves
    virtual void renderPathSegment(const hssh::GlobalPathSegment& segment,
                                   const map_place_info_t&        plusPlace,
                                   const map_place_info_t&        minusPlace,
                                   path_segment_attribute_t       attributes);
    virtual void renderFrontierPathSegment(const hssh::GlobalPathSegment& segment,
                                           const map_place_info_t&        plusPlace,
                                           const map_place_info_t&        minusPlace,
                                           path_segment_attribute_t       attributes);

    /**
    * getPlaceColor retrieves the color to use for rendering a particular place.
    */
    GLColor getPlaceColor(const map_place_info_t& place, place_attribute_t attributes);

    /**
    * getSegmentColor retrieves the color to use for rendering a particular path segment.
    */
    GLColor getSegmentColor(const hssh::GlobalPathSegment& segment, path_segment_attribute_t attributes);

private:

    void drawPlaces     (const hssh::TopologicalState& topoMap, const hssh::MetricMapCache& places);
    void drawPaths      (const hssh::TopologicalState& topoMap, const hssh::MetricMapCache& places);
    void drawLocation   (const hssh::TopologicalState& topoMap, const hssh::MetricMapCache& places);
    void drawPathSegment(const hssh::GlobalPathSegment& segment, const hssh::TopologicalState& topoMap, const hssh::MetricMapCache& places);

    void drawPlaceLocation(const hssh::GlobalLocation& location, const map_place_info_t& place);
    void drawPathLocation (const hssh::GlobalLocation& location, const map_place_info_t& entry, const map_place_info_t& target);

    void renderRobotTriangle(const pose_t& pose);
    void updateMapBoundary  (const pose_t& placeCenter, const math::Rectangle<float>& placeBoundary);

    map_place_info_t create_place_info(hssh::Id placeId,
                                       const hssh::TopologicalState& topoMap,
                                       const hssh::MetricMapCache& places);

    math::Rectangle<float> mapBoundary;
    bool                   boundaryIsInitialized;

    std::map<int,      GLColor> placeColors;
    std::map<uint32_t, GLColor> segmentColors;
};

} // namespace hssh
} // namespace vulcan

#endif // UI_COMPONENTS_TOPOLOGICAL_MAP_RENDERER_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_area_renderer.h
* \author   Collin Johnson
*
* Declaration of LocalAreaRenderer.
*/

#ifndef UI_COMPONENTS_LOCAL_AREA_RENDERER_H
#define UI_COMPONENTS_LOCAL_AREA_RENDERER_H

#include "ui/common/color_interpolator.h"
#include "ui/common/ui_color.h"
#include "hssh/local_topological/area_visitor.h"
#include "core/point.h"
#include <memory>

namespace vulcan
{
namespace hssh { class Gateway; }
namespace hssh { class HeatMapStatistics; }
namespace hssh { class LocalArea; }
namespace hssh { class LocalTopoMap; }
namespace hssh { class AreaExtent; }
namespace hssh { class AreaProposal; }
namespace hssh { enum class AreaType; }
namespace hssh { class SmallScaleStar; }
namespace hssh { class VoronoiSkeletonGrid; }
namespace ui
{

class AreaExtentRenderer;
class SmallScaleStarRenderer;

/**
* LocalAreaRenderer
*/
class LocalAreaRenderer : public hssh::LocalAreaVisitor
{
public:

    /**
    * Constructor for LocalAreaRenderer.
    */
    LocalAreaRenderer(void);

    ~LocalAreaRenderer(void);

    /**
    * showGateways sets the flag for whether or not gateways should be drawn.
    */
    void showGateways(bool show) { showGateways_ = show; }

    /**
    * showStar sets the flag for whether or not the small-scale star should be drawn.
    */
    void showStar(bool show) { showStar_ = show; }

    /**
    * setMetersPerCell sets the meters per cell to render the extent of the areas.
    */
    void setMetersPerCell(double metersPerCell) { metersPerCell_ = metersPerCell; }

    /**
    * renderLocalTopoMap
    */
    void renderLocalTopoMap(const hssh::LocalTopoMap& map);

    /**
    * renderLocalTopoMapAsHeatMap renders the LocalTopoMap using the provided heat map.
    */
    void renderLocalTopoMapAsHeatMap(const hssh::LocalTopoMap& map, const hssh::HeatMapStatistics& heatMap);

    /**
    * renderLocalTopoMapAsGraph renders the local topo map with the following representation:
    *
    *   - decision points are large squares
    *   - destinations are smaller squares
    *   - path segments are line segments
    *   - destinations along a path are drawn as a line from the path segment line to the center of the destination
    */
    void renderLocalTopoMapAsGraph(const hssh::LocalTopoMap& map);

    /**
    * renderLocalTopoMapAsHSSHGraph renders the local topo map with the following representation:
    *
    *   - decision points are large squares
    *   - destinations are smaller squares
    *   - path segments are line segments
    *   - destinations along a path are drawn as:
    *       - a line from the path segment line to the center of the destination
    *       - a square representing the decision point that would result from the decision point along the path segment
    */
    void renderLocalTopoMapAsHSSHGraph(const hssh::LocalTopoMap& map);

    /**
    * renderLocalArea
    */
    void renderLocalArea(const hssh::LocalArea& area);

    /**
    * renderAreaProposal
    */
    void renderAreaProposal(const hssh::AreaProposal& area, const hssh::VoronoiSkeletonGrid& grid);

    // LocalAreaVisitor interface
    virtual void visitDecisionPoint(const hssh::LocalDecisionPoint& decision);
    virtual void visitDestination  (const hssh::LocalDestination&   destination);
    virtual void visitPathSegment  (const hssh::LocalPathSegment&   path);

private:

    bool showGateways_;
    bool showStar_;

    std::unique_ptr<AreaExtentRenderer> extentRenderer_;
    std::unique_ptr<SmallScaleStarRenderer> starRenderer_;
    LinearColorInterpolator heatMapInterpolator_;

    double metersPerCell_;


    void drawExtent   (const hssh::AreaExtent& proposal, const GLColor& color);
    void drawGateways (const std::vector<hssh::Gateway>& gateways, const GLColor& color) const;
    void drawFrontiers(const std::vector<Point<double>>& frontiers) const;
    void drawStar     (const hssh::SmallScaleStar& star, const Point<double>& origin);
};

}
}

#endif // UI_COMPONENTS_LOCAL_AREA_RENDERER_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gateway_editor.h
 * \author   Collin Johnson
 *
 * Declaration of GatewayEditor.
 */

#ifndef UI_MAPEDITOR_GATEWAY_EDITOR_H
#define UI_MAPEDITOR_GATEWAY_EDITOR_H

#include "hssh/local_topological/gateway.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "ui/common/gl_event.h"
#include "ui/common/grid_object_selector.h"
#include <vector>

namespace vulcan
{
namespace ui
{

/**
 * GatewayEditor is a mouse handler responsible for the creation of gateways by a user. It captures GLMouse events and
 * allows for manual creation of gateways using the following mouse-based commands:
 *
 * All clicks are the right mouse button.
 *
 *   - Hover + no buttons/modifiers : create gateway w/normal as most straight endpoints
 *   - Release + no buttons/modifiers: create the currently shown hover gateway
 *   - Ctrl + click : mouse movement rotates gateway normal to find new endpoints
 *   - Ctrl + release : create the currently shown gateway
 *   - Shift + release : delete the most recent gateway (can repeat to delete many gateways)
 *
 */
class GatewayEditor : public GLMouseHandler
{
public:
    using const_iterator = std::vector<hssh::Gateway>::const_iterator;

    /**
     * Constructor for GatewayEditor.
     *
     * \param    gridWidget          Widget in which the gateways will be selected
     */
    GatewayEditor(const GridBasedDisplayWidget* gridWidget);

    /**
     * hoverGateway retrieves the current gateway that was constructed for the hover cell.
     *
     * If there is not currently a cell over which the hovering is happening, then there won't be an associated
     * gateway.
     */
    boost::optional<hssh::Gateway> hoverGateway(void) const { return hover_; }

    /**
     * selectedGateway retrieves the gateway that the user is currently creating by right-clicking and dragging the
     * mouse around.
     *
     * If the user isn't clicking-and-dragging, then there will be no selected gateway.
     */
    boost::optional<hssh::Gateway> selectedGateway(void) const { return selected_; }

    /**
     * setSkeleton sets the Voronoi skeleton in which gateways will be created. Setting a new skeleton erases all
     * existing gateways stored by the editor.
     */
    void setSkeleton(hssh::VoronoiSkeletonGrid skeleton);

    /**
     * addGateways adds gateways to the editor from some external source.
     */
    void addGateways(const std::vector<hssh::Gateway>& gateways);

    /**
     * constructedGateways retrieves all constructed gateways.
     */
    std::vector<hssh::Gateway> constructedGateways(void) const { return constructed_; }

    /**
     * clearGateways clears all stored gateway information.
     */
    void clearGateways(void);

    // GLMouseHandler interface
    GLEventStatus handleMouseMoved(const GLMouseEvent& event) override;
    GLEventStatus handleRightMouseDown(const GLMouseEvent& event) override;
    GLEventStatus handleRightMouseUp(const GLMouseEvent& event) override;

private:
    // The editor responds to the events differently depending on keyboard modifiers
    // The state specifies how the events should be responded too
    enum class State
    {
        hovering,    // no ctrl
        selecting,   // w/ctrl
        deleting,    // w/shift
    };

    // Three types of mouse events are handled. The GLMouseEvent doesn't contain information on what type of event
    // caused the construction of the event structure, so pass it along with the event in the state machine
    enum class EventType
    {
        moved,   // mouse just moved
        down,    // mouse was down
        up,      // mouse is up
    };

    State state_ = State::hovering;
    GridObjectSelector<hssh::cell_t, NearestCellSelector> skeletonSelector_;
    GridObjectSelector<hssh::Gateway, NearestCellSelector> gatewaySelector_;
    boost::optional<hssh::Gateway> hover_;
    boost::optional<hssh::Gateway> selected_;
    std::vector<hssh::Gateway> constructed_;
    hssh::VoronoiSkeletonGrid skeleton_;
    Point<double> selectedClickPoint_;   // point clicked when selection started -- used for relative angle
    double hoverNormal_ = 0.0;
    int32_t nextId_ = 0;

    void setObjectSelectorCells(void);
    void setGatewaySelectorCells(void);
    void setState(const GLMouseEvent& event);
    void processEvent(const GLMouseEvent& event, EventType type);
    void processHoverEvent(const GLMouseEvent& event, EventType type);
    void processSelectingEvent(const GLMouseEvent& event, EventType type);
    void processDeletingEvent(const GLMouseEvent& event, EventType type);

    boost::optional<hssh::Gateway> createGatewayAtCell(hssh::cell_t cell, double normal);
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_MAPEDITOR_GATEWAY_EDITOR_H

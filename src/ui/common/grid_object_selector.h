/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     grid_object_selector.h
 * \author   Collin Johnson
 *
 * Declaration of GridObjectSelector.
 */

#ifndef UI_COMMON_GRID_OBJECT_SELECTOR_H
#define UI_COMMON_GRID_OBJECT_SELECTOR_H

#include "ui/common/gl_event.h"
#include "ui/common/grid_cell_selector.h"
#include "ui/common/object_selector_event_handler.h"
#include "ui/common/selection_policies.h"
#include <boost/optional.hpp>
#include <map>

namespace vulcan
{
namespace ui
{

/**
 * GridObjectSelector is a mouse handler that allows hovering and selecting objects that are associated with grid cells.
 * These objects all have to have the same base class, but otherwise any object can be associated with cells.
 *
 * GridObjectSelector requires two pieces of data. First, a GridBasedDisplayWidget to convert the mouse coordinates to
 * grid cells. Second, a mapping of cell->GridObject.
 *
 * An object is selected using the right mouse button.
 *
 * Care should be taken with the objects to ensure they don't become invalidated. The GridObjectSelector doesn't
 * acquire ownership.
 *
 * SelectionPolicy is a policy that defines how objects are selected. The selection policy determines which object will
 * be selected when the underlying GridCellSelector has selected a new cell. The policy requires a single operator:
 *
 *   Point<int> operator()(Point<int> cell, const vector<Point<int>>& objectCells);
 *
 * The default policy will only match the exact cell. An alterate selects the nearest object cell. They are defined
 * in selection_policies.h.
 */
template <class GridObject, class SelectionPolicy = ExactCellSelector>
class GridObjectSelector : public GLMouseHandler
{
public:
    /**
     * Constructor for GridObjectSelector.
     *
     * \param    widget              GridBasedDisplayWidget in which objects are being selected
     */
    GridObjectSelector(const GridBasedDisplayWidget* widget)
    : hover_(nullptr)
    , selected_(nullptr)
    , cellSelector_(widget, 1)
    , handler_(nullptr)
    {
    }

    /**
     * setObjects sets the objects to be selected.
     *
     * \param    cellToObjectMap     Cell->object mapping to be used for determining which object is currently selected
     */
    void setObjects(std::map<Point<int>, GridObject> cellToObjectMap)
    {
        hover_ = nullptr;
        selected_ = nullptr;
        cellToObject_ = std::move(cellToObjectMap);
        objectCells_.clear();
        for (auto& obj : cellToObject_) {
            objectCells_.push_back(obj.first);
        }
    }

    /**
     * clearObjects clears the objects currently being selected.
     */
    void clearObjects(void)
    {
        hover_ = nullptr;
        selected_ = nullptr;
        cellToObject_.clear();
        objectCells_.clear();
    }

    /**
     * setHandler sets the event handler for when the selected object changes. Set to nullptr if no event callbacks
     * are going to be used.
     */
    void setHandler(ObjectSelectorEventHandler<GridObject>* handler) { handler_ = handler; }

    /**
     * hoverObject retrieves the object over which the mouse is currently hovering. If the mouse isn't hovering over
     * a cell with an associated object, then nullptr is returned to indicate this condition.
     */
    boost::optional<GridObject> hoverObject(void) const
    {
        if (hover_) {
            return *hover_;
        } else {
            return boost::none;
        }
    }

    /**
     * selectedObject retrieves the currently selected object. If no object has been selected, then
     */
    boost::optional<GridObject> selectedObject(void) const
    {
        if (selected_) {
            return *selected_;
        } else {
            return boost::none;
        }
    }

    // GLMouseHandler interface
    virtual GLEventStatus handleRightMouseDown(const GLMouseEvent& event)
    {
        cellSelector_.handleRightMouseDown(event);
        // If a cell has been selected, then find the object associated with it
        if (cellSelector_.numSelectedCells() > 0) {
            auto newSelected = findObjectForCell(cellSelector_.selectedCells().front());

            if (handler_ && (newSelected != selected_) && newSelected) {
                handler_->objectSelected(*newSelected);
            }

            selected_ = newSelected;
        }

        return GLEventStatus::passthrough;
    }

    virtual GLEventStatus handleRightMouseUp(const GLMouseEvent& event)
    {
        cellSelector_.handleRightMouseUp(event);
        // If no cells are selected any longer, than no object can be selected
        if ((cellSelector_.numSelectedCells() == 0) && selected_) {
            selected_ = nullptr;
        }

        return GLEventStatus::passthrough;
    }

    virtual GLEventStatus handleMouseMoved(const GLMouseEvent& event)
    {
        cellSelector_.handleMouseMoved(event);
        // When the mouse moves, a new hover object always exists
        auto newHover_ = findObjectForCell(cellSelector_.hoverCell());

        if (handler_ && (newHover_ != hover_)) {
            if (hover_) {
                handler_->objectExited(*hover_);
            }

            if (newHover_) {
                handler_->objectEntered(*newHover_);
            }
        }

        hover_ = newHover_;

        return event.rightIsDown ? GLEventStatus::capture : GLEventStatus::passthrough;
    }

private:
    std::map<Point<int>, GridObject> cellToObject_;
    std::vector<Point<int>> objectCells_;
    GridObject* hover_;
    GridObject* selected_;
    GridCellSelector cellSelector_;
    ObjectSelectorEventHandler<GridObject>* handler_;
    SelectionPolicy selector_;


    GridObject* findObjectForCell(Point<int> cell)
    {
        auto objCell = selector_(cell, objectCells_);
        auto objectIt = cellToObject_.find(objCell);
        return (objectIt != cellToObject_.end()) ? &(objectIt->second) : nullptr;
    }
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMMON_GRID_OBJECT_SELECTOR_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     grid_cell_selector.cpp
 * \author   Collin Johnson
 *
 * Definition of GridCellSelector.
 */

#include "ui/common/grid_cell_selector.h"
#include "ui/components/grid_based_display_widget.h"
#include <algorithm>
#include <cassert>

namespace vulcan
{
namespace ui
{

GridCellSelector::GridCellSelector(const GridBasedDisplayWidget* gridWidget, std::size_t maxSelectionSize)
: maxSelectionSize_(maxSelectionSize)
, currentlySelecting_(false)
, widget_(gridWidget)
{
    assert(widget_);
}


GLEventStatus GridCellSelector::handleRightMouseDown(const GLMouseEvent& event)
{
    if (event.rightIsDown) {
        selected_.clear();
        selected_.push_back(widget_->convertWorldToGrid(event.glCoords));
        currentlySelecting_ = true;
    }

    return GLEventStatus::passthrough;
}


GLEventStatus GridCellSelector::handleRightMouseUp(const GLMouseEvent& event)
{
    currentlySelecting_ = event.rightIsDown;

    hover_ = widget_->convertWorldToGrid(event.glCoords);
    addMousePointIfUnique(hover_);

    return GLEventStatus::passthrough;
}


GLEventStatus GridCellSelector::handleMouseMoved(const GLMouseEvent& event)

{
    // Only capture the event if the left mouse button is down, indicating that cells are being selected
    hover_ = widget_->convertWorldToGrid(event.glCoords);

    if (event.rightIsDown) {
        addMousePointIfUnique(hover_);
        currentlySelecting_ = true;
    } else {
        currentlySelecting_ = false;
    }

    return event.rightIsDown ? GLEventStatus::capture : GLEventStatus::passthrough;
}


void GridCellSelector::addMousePointIfUnique(Point<int> point)
{
    // Only add if the cell has changed since the mouse moved. Repeated cells might happen eventually if the
    // mouse path twists, but that's okay
    if ((selected_.size() < maxSelectionSize_)
        && (std::find(selected_.begin(), selected_.end(), point) == selected_.end())) {
        selected_.push_back(hover_);
    }
}

}   // namespace ui
}   // namespace vulcan

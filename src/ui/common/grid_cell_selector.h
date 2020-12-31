/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     grid_cell_selector.h
 * \author   Collin Johnson
 *
 * Declaration of GridCellSelector.
 */

#ifndef UI_COMMON_GRID_CELL_SELECTOR_H
#define UI_COMMON_GRID_CELL_SELECTOR_H

#include "ui/common/gl_event.h"
#include <atomic>
#include <vector>

namespace vulcan
{
namespace ui
{

class GridBasedDisplayWidget;

/**
 * GridCellSelector is a mouse event handler that translates the mouse position into grid coordinates.
 * The selector maintains two distinct cells. The hover cell is the cell over which the mouse was last located,
 * it always has some value. The selected cells are all cells the mouse passes over while the left mouse button
 * is held down. The selected cells will continue to be concatenated until clearSelectedCells() is called.
 * The selected cells continue to be accumulated because the rate at which they are read likely differs from
 * the rate at which the UI updates.
 */
class GridCellSelector : public GLMouseHandler
{
public:
    /**
     * Constructor for GridCellSelector.
     *
     * \param    gridWidget      GridBasedDisplayWidget to be used for converting the mouse into the currently displayed
     * grid
     */
    GridCellSelector(const GridBasedDisplayWidget* gridWidget, std::size_t maxSelectionSize = 100000);

    Point<int> hoverCell(void) const { return hover_; }
    std::vector<Point<int>> selectedCells(void) const { return selected_; }

    /**
     * isActivelySelecting checks to see if the user is currently selecting cells. Cells are being selected if
     * the user has the left mouse button held down.
     */
    bool isActivelySelecting(void) const { return currentlySelecting_; }

    /**
     * numSelectedCells retrieves the number of cells selected since the cells were last cleared out.
     */
    std::size_t numSelectedCells(void) const { return selected_.size(); }

    /**
     * clearSelectedCells clears any selected cells.
     */
    void clearSelectedCells(void) { selected_.clear(); }

    // GLMouseHandler interface
    virtual GLEventStatus handleRightMouseDown(const GLMouseEvent& event);
    virtual GLEventStatus handleRightMouseUp(const GLMouseEvent& event);
    virtual GLEventStatus handleMouseMoved(const GLMouseEvent& event);

private:
    Point<int> hover_;
    std::vector<Point<int>> selected_;
    std::size_t maxSelectionSize_;

    std::atomic<bool> currentlySelecting_;

    const GridBasedDisplayWidget* widget_;

    void addMousePointIfUnique(Point<int> point);
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMMON_GRID_CELL_SELECTOR_H

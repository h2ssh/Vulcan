/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lpm_editing.h
* \author   Collin Johnson
* 
* Functions and data types for editing an LPM:
* 
*   edit_cell_t : an edited cell
* 
*   apply_cell_edits : commit a collection of cell edits to an LPM
*   flood_fill_lpm   : fill a section of the LPM, replacing cells of the type at the start cell with the supplied type
*   cells_along_line : finds all cells along a line defined by the two endpoints
*/

#ifndef UI_MAPEDITOR_LPM_EDITING_H
#define UI_MAPEDITOR_LPM_EDITING_H

#include "hssh/local_metric/lpm.h"
#include "core/line.h"
#include "core/point.h"
#include <stack>
#include <vector>

namespace vulcan
{
namespace hssh { class LocalPerceptualMap; }

namespace ui
{

/**
* LPMEditor handles editing an LPM via the UI. The editor supports three actions:
* 
*   - changing a single cell
*   - changing cells along a line
*   - flood-filling a region
* 
* Each edit can be undone until the commit() method is called, at which point all edits
* are made permanent -- well you can manually change the cells back.
* 
* The LPM being edited needs to be set. A point to this LPM is maintained, but ownership is not acquired,
* so it's lifetime needs to be managed elsewhere and last as long as the LPMEditor is being used to edit it.
*/
class LPMEditor
{
public:
    
    /**
    * Constructor for LPMEditor.
    */
    LPMEditor(void);
    
    /**
    * setLPM sets the LPM to be edited. When the LPM is set, the edit history is erased. If the LPM is null,
    * then edits will be made or stored -- duh.
    */
    void setLPM(hssh::LocalPerceptualMap* lpm);
    
    /**
    * changeCell changes the type of a cell to the specified type.
    * 
    * \param    cell        Cell to be changed
    * \param    type        Type of the cell being changed
    */
    void changeCell(const Point<int>& cell, hssh::cell_type_t type);
    
    /**
    * changeGroup changes the type of a group of cells to the specified type.
    * 
    * \param    cells       Cells to be changed
    * \param    type        Type cells will change to
    */
    void changeGroup(const std::vector<Point<int>>& cell, hssh::cell_type_t type);
    
    /**
    * changeLine changes all cells along the line to the specified type.
    * 
    * \param    line        Line to be changed
    * \param    type        New type to assign to the cells
    */
    void changeLine(const Line<int>& line, hssh::cell_type_t type);
    
    /**
    * changeRegion flood fills a region starting at the specified cell and growing to all connected
    * cells of the same type.
    * 
    * \param    start       Start of the region
    * \param    type        Type to change region cells to
    */
    void changeRegion(const Point<int>& cell, hssh::cell_type_t type);
    
    /**
    * canUndo checks to see if they is any edit history that can be undone.
    */
    bool canUndo(void) const { return !edits.empty(); }
    
    /**
    * undo reverses the last change that was made if there are any changes.
    */
    void undo(void);
    
    /**
    * commit commits all the current changes, erasing any history that they occurred, so they can't be undone.
    */
    void commit(void);
    
private:
    
    enum class EditType
    {
        CELLS,
        FILL
    };
    
    // Holds information for undoing previous edits
    struct map_edit_t
    {
        EditType          type;
        Point<int>  cell;
        std::size_t       groupCellsRemaining;
        hssh::cell_type_t newType;
        uint8_t           newCost;
        hssh::cell_type_t originalType;
        uint8_t           originalCost;
    };
    
    std::stack<map_edit_t>    edits;
    hssh::LocalPerceptualMap* lpm;
    
    void undoEdit  (void);
    void undoGroup (void);
    void undoFill  (void);
    void changeCell(const map_edit_t& edit, bool undo);
    void floodFill (const map_edit_t& edit, bool undo);
};

} // namespace ui
} // namespace vulcan

#endif // UI_MAPEDITOR_LPM_EDITING_H

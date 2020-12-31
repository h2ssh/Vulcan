/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     lpm_editing.cpp
 * \author   Collin Johnson
 *
 * Definition of apply_cell_edits, flood_fill_lpm, and cells_along_line.
 */

#include "ui/mapeditor/lpm_editing.h"
#include <cassert>
#include <cmath>
#include <queue>

namespace vulcan
{
namespace ui
{

typedef std::queue<Point<int>> CellQueue;

uint8_t cost_for_edit_type(hssh::cell_type_t type);
std::vector<Point<int>> cells_along_line(const hssh::LocalPerceptualMap& lpm, const Line<int>& line);
std::size_t flood_fill_lpm(hssh::LocalPerceptualMap& lpm,
                           const Point<int>& startCell,
                           hssh::cell_type_t newType,
                           uint8_t newCost);
void enqueue_and_change_type_if_needed(hssh::LocalPerceptualMap& lpm,
                                       const Point<int>& cell,
                                       hssh::cell_type_t initialType,
                                       hssh::cell_type_t newType,
                                       CellQueue& queue);


LPMEditor::LPMEditor(void) : lpm(0)
{
}


void LPMEditor::setLPM(hssh::LocalPerceptualMap* lpm)
{
    this->lpm = lpm;
    edits = std::stack<map_edit_t>();   // have to assign an empty stack to clear it, no .clear() method
}


void LPMEditor::changeCell(const Point<int>& cell, hssh::cell_type_t type)
{
    if (lpm && lpm->isCellInGrid(cell)) {
        map_edit_t edit;
        edit.type = EditType::CELLS;
        edit.cell = cell;
        edit.groupCellsRemaining = 0;
        edit.newType = type;
        edit.originalType = lpm->getCellTypeNoCheck(cell);
        edit.originalCost = lpm->getCostNoCheck(cell);

        edits.push(edit);
        changeCell(edit, false);
    }
}


void LPMEditor::changeGroup(const std::vector<Point<int>>& cells, hssh::cell_type_t type)
{
    if (lpm && !cells.empty()) {
        map_edit_t edit;
        edit.type = EditType::CELLS;
        edit.groupCellsRemaining = 0;
        edit.newType = type;
        edit.newCost = cost_for_edit_type(type);

        for (auto cell : cells) {
            if (lpm->isCellInGrid(cell)) {
                edit.cell = cell;
                edit.originalType = lpm->getCellTypeNoCheck(cell);
                edit.originalCost = lpm->getCostNoCheck(cell);

                edits.push(edit);
                changeCell(edit, false);
                ++edit.groupCellsRemaining;
            }
        }
    }
}


void LPMEditor::changeLine(const Line<int>& line, hssh::cell_type_t type)
{
    if (lpm) {
        auto lineCells = cells_along_line(*lpm, line);

        // Create an edit for each cell along the line. Mark the number of cells to go in the line so the full
        // line can be undone in the future by popping until there are no more edits for the line
        for (std::size_t n = 0; n < lineCells.size(); ++n) {
            map_edit_t edit;
            edit.type = EditType::CELLS;
            edit.cell = lineCells[n];
            edit.groupCellsRemaining = n;
            edit.newType = type;
            edit.newCost = cost_for_edit_type(type);
            edit.originalType = lpm->getCellTypeNoCheck(lineCells[n]);
            edit.originalCost = lpm->getCostNoCheck(lineCells[n]);

            edits.push(edit);
            changeCell(edit, false);
        }
    }
}


void LPMEditor::changeRegion(const Point<int>& cell, hssh::cell_type_t type)
{
    if (lpm && lpm->isCellInGrid(cell)) {
        map_edit_t edit;
        edit.type = EditType::FILL;
        edit.cell = cell;
        edit.newType = type;
        edit.newCost = cost_for_edit_type(type);
        edit.originalType = lpm->getCellTypeNoCheck(cell);
        edit.originalCost = lpm->getCostNoCheck(cell);

        edits.push(edit);
        floodFill(edit, false);
    }
}


void LPMEditor::undo(void)
{
    if (lpm && !edits.empty()) {
        undoEdit();
    }
}


void LPMEditor::commit(void)
{
    edits = std::stack<map_edit_t>();   // have to assign an empty stack to clear it, no .clear() method
}


void LPMEditor::undoEdit()
{
    map_edit_t edit = edits.top();

    switch (edit.type) {
    case EditType::CELLS:
        undoGroup();
        break;

    case EditType::FILL:
        undoFill();
        break;

    default:
        assert("ERROR::LPMEditor: Invalid edit type!" && false);
    }
}


void LPMEditor::undoGroup(void)
{
    map_edit_t edit;

    do {
        edit = edits.top();
        assert(edit.type == EditType::CELLS);

        changeCell(edit, true);
        edits.pop();
    } while (edit.groupCellsRemaining > 0);
}


void LPMEditor::undoFill(void)
{
    map_edit_t fill = edits.top();

    assert(fill.type == EditType::FILL);

    floodFill(fill, true);
    edits.pop();
}


void LPMEditor::changeCell(const map_edit_t& edit, bool undo)
{
    lpm->setCost(edit.cell, (undo ? edit.originalCost : edit.newCost));
    lpm->setType(edit.cell, (undo ? edit.originalType : edit.newType));
}


void LPMEditor::floodFill(const map_edit_t& edit, bool undo)
{
    flood_fill_lpm(*lpm,
                   edit.cell,
                   (undo ? edit.originalType : edit.newType),
                   (undo ? edit.originalCost : edit.newCost));
}


uint8_t cost_for_edit_type(hssh::cell_type_t type)
{
    if (type == hssh::kFreeOccGridCell) {
        return 0;
    } else if ((type == hssh::kUnknownOccGridCell) || (type == hssh::kUnobservedOccGridCell)) {
        return 127;
    } else   // some sort of obstacle
    {
        return 255;
    }
}


std::vector<Point<int>> cells_along_line(const hssh::LocalPerceptualMap& lpm, const Line<int>& line)
{
    // Don't do fancy line rasterizing for now. Just fill in using the truncation via conversion style
    std::vector<Point<int>> cells;

    double xIncr = (line.b.x - line.a.x) / length(line);
    double yIncr = (line.b.y - line.a.y) / length(line);

    int numIncrements = std::ceil(length(line) / std::sqrt(xIncr * xIncr + yIncr * yIncr));

    for (int n = 0; n < numIncrements; ++n) {
        Point<int> cell(line.a.x + n * xIncr, line.a.y + n * yIncr);

        if (lpm.isCellInGrid(cell)) {
            cells.push_back(cell);
        }
    }

    return cells;
}


std::size_t
  flood_fill_lpm(hssh::LocalPerceptualMap& lpm, const Point<int>& startCell, hssh::cell_type_t newType, uint8_t newCost)
{
    if (!lpm.isCellInGrid(startCell)) {
        return 0;
    }

    hssh::cell_type_t replaceType = lpm.getCellTypeNoCheck(startCell);
    std::size_t numReplaced = 0;
    CellQueue cellQueue;

    if (replaceType == newType) {
        return 0;
    }

    lpm.setCostNoCheck(startCell, newCost);
    lpm.setTypeNoCheck(startCell, newType);

    cellQueue.push(startCell);

    while (!cellQueue.empty()) {
        auto cell = cellQueue.front();

        enqueue_and_change_type_if_needed(lpm, Point<int>(cell.x + 1, cell.y), replaceType, newType, cellQueue);
        enqueue_and_change_type_if_needed(lpm, Point<int>(cell.x - 1, cell.y), replaceType, newType, cellQueue);
        enqueue_and_change_type_if_needed(lpm, Point<int>(cell.x, cell.y + 1), replaceType, newType, cellQueue);
        enqueue_and_change_type_if_needed(lpm, Point<int>(cell.x, cell.y - 1), replaceType, newType, cellQueue);

        lpm.setCostNoCheck(cell, newCost);
        lpm.setTypeNoCheck(cell, newType);

        cellQueue.pop();
        ++numReplaced;
    }

    return numReplaced;
}


void enqueue_and_change_type_if_needed(hssh::LocalPerceptualMap& lpm,
                                       const Point<int>& cell,
                                       hssh::cell_type_t initialType,
                                       hssh::cell_type_t newType,
                                       CellQueue& queue)
{
    // The check to enqueue is simplified because all visited cells will have their types changed immediately, so the
    // visited set is automatically handled
    if (lpm.isCellInGrid(cell) && (lpm.getCellTypeNoCheck(cell) == initialType)) {
        queue.push(cell);
        lpm.setTypeNoCheck(cell, newType);
    }
}

}   // namespace ui
}   // namespace vulcan

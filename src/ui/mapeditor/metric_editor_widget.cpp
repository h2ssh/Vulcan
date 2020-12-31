/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     metric_editor_widget.cpp
 * \author   Collin Johnson
 *
 * Implementation of MetricEditorWidget.
 */

#include "ui/mapeditor/metric_editor_widget.h"
#include "ui/common/ui_params.h"
#include "ui/components/occupancy_grid_renderer.h"
#include "utils/auto_mutex.h"
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

void draw_cell(Point<double> cell, double scale);


MetricEditorWidget::MetricEditorWidget(wxWindow* parent,
                                       wxWindowID id,
                                       const wxPoint& pos,
                                       const wxSize& size,
                                       long style,
                                       const wxString& name,
                                       const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, lpmRenderer(new OccupancyGridRenderer)
, lpmIsDirty_(false)
{
}


MetricEditorWidget::~MetricEditorWidget(void)
{
}


void MetricEditorWidget::setParams(const ui_params_t& params)
{
    this->params = params.lpmParams;
}


void MetricEditorWidget::setLPM(std::shared_ptr<hssh::LocalPerceptualMap> lpm)
{
    utils::AutoMutex autoLock(lpmLock);

    this->lpm = lpm;
    lpmIsDirty_ = true;
    setViewRegion(lpm->getWidthInMeters(), lpm->getHeightInMeters());
    setCameraFocalPoint(lpm->getGlobalCenter());
}


void MetricEditorWidget::setHoverCell(Point<int> cell, hssh::cell_type_t type)
{
    hoverCell = cell;
    hoverType = type;

    usingHoverCell = true;
}


void MetricEditorWidget::setHoverLine(Line<int> line, hssh::cell_type_t type)
{
    hoverLine = line;
    hoverType = type;

    usingHoverCell = false;
}


void MetricEditorWidget::setSelectedCells(const std::vector<Point<int>>& selected)
{
    selectedCells = selected;
}


Point<int> MetricEditorWidget::convertWorldToGrid(const Point<float>& world) const
{
    if (lpm) {
        return utils::global_point_to_grid_cell(world, *lpm);
    }

    return Point<int>(0, 0);
}


void MetricEditorWidget::renderWidget(void)
{
    utils::AutoMutex autoLock(lpmLock);

    // Only render if there's actually an LPM. Otherwise, just chill out.
    if (lpm) {
        if (lpmIsDirty_) {
            lpmRenderer->setGrid(*lpm);
            lpmIsDirty_ = false;
        }

        lpmRenderer->renderGrid();

        if (showHover && usingHoverCell) {
            renderHoverCell();
            renderSelectedCells();
        } else if (showHover)   // in line mode, don't need to render the selected cells because a line is drawn instead
        {
            renderHoverLine();
        }
    }
}


void MetricEditorWidget::renderHoverCell(void)
{
    GLColor color = selectCellColor();

    color.set(0.66f);
    glBegin(GL_QUADS);
    draw_cell(utils::grid_point_to_global_point(hoverCell, *lpm), lpm->metersPerCell());
    glEnd();
}


void MetricEditorWidget::renderHoverLine(void)
{
    GLColor color = selectCellColor();

    Line<double> global(utils::grid_point_to_global_point(hoverLine.a, *lpm),
                        utils::grid_point_to_global_point(hoverLine.b, *lpm));

    color.set(0.66f);
    glBegin(GL_QUADS);
    draw_cell(global.a, lpm->metersPerCell());
    draw_cell(global.b, lpm->metersPerCell());
    glEnd();

    glLineWidth(2.0);
    glBegin(GL_LINES);
    glVertex2f(global.a.x, global.a.y);
    glVertex2f(global.b.x, global.b.y);
    glEnd();
}


void MetricEditorWidget::renderSelectedCells(void)
{
    GLColor color = selectCellColor();

    color.set(0.66f);
    glBegin(GL_QUADS);

    for (const auto& cell : selectedCells) {
        draw_cell(utils::grid_point_to_global_point(cell, *lpm), lpm->metersPerCell());
    }

    glEnd();
}


GLColor MetricEditorWidget::selectCellColor(void)
{
    GLColor cellColor = params.occupiedColor;

    if (hoverType & hssh::kDynamicOccGridCell) {
        cellColor = params.dynamicColor;
    } else if (hoverType & hssh::kLimitedVisibilityOccGridCell) {
        cellColor = params.limitedVisibilityColor;
    } else if (hoverType & hssh::kHazardOccGridCell) {
        cellColor = params.hazardColor;
    } else if (hoverType & hssh::kQuasiStaticOccGridCell) {
        cellColor = params.quasiStaticColor;
    }

    return cellColor;
}


void draw_cell(Point<double> cell, double scale)
{
    glVertex2f(cell.x, cell.y);
    glVertex2f(cell.x, cell.y + scale);
    glVertex2f(cell.x + scale, cell.y + scale);
    glVertex2f(cell.x + scale, cell.y);
}

}   // namespace ui
}   // namespace vulcan

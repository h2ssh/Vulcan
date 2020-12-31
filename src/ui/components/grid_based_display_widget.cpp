/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "ui/components/grid_based_display_widget.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include "core/point.h"
#include "ui/common/gl_utilities.h"


namespace vulcan
{
namespace ui
{

const float MAX_ZOOM = 1.0f;
const float MIN_ZOOM = 0.01f;


GridBasedDisplayWidget::GridBasedDisplayWidget(wxWindow* parent,
                                               wxWindowID id,
                                               const wxPoint& pos,
                                               const wxSize& size,
                                               long style,
                                               const wxString& name,
                                               const wxPalette& palette)
    : OpenGLWidget(parent, id, pos, size, style|wxFULL_REPAINT_ON_RESIZE, name, palette)
    , statusBar(0)
    , leftWasDownOnLastEvent(false)
{
    pushMouseHandler(this);
}


GridBasedDisplayWidget::~GridBasedDisplayWidget(void)
{
    // Nothing needed for now
}


void GridBasedDisplayWidget::setStatusBar(wxStatusBar* status)
{
    statusBar = status;
}


void GridBasedDisplayWidget::setGridDimensions(float width, float height)
{
    displayedGridWidth  = width;
    displayedGridHeight = height;

    setMaxViewDimensions(width, height);
}


void GridBasedDisplayWidget::enableScrolling(void)
{
    enablePanning();
}


void GridBasedDisplayWidget::disableScrolling(void)
{
    disablePanning();
}


GLEventStatus GridBasedDisplayWidget::handleMouseMoved(const GLMouseEvent& event)
{
    if(statusBar)
    {
        displayMousePositionOnStatusBar(event);
    }

    previousMousePosition = event.glCoords;

    return GLEventStatus::passthrough;
}

// Various event handlers
void GridBasedDisplayWidget::displayMousePositionOnStatusBar(const GLMouseEvent& event)
{
    auto cell       = convertWorldToGrid(event.glCoords);
    wxString status;
    status.Printf(wxT("Global:(%.2f,%.2f)   Cell:(%i,%i) "), event.glCoords.x, event.glCoords.y, cell.x, cell.y);
    status.Append(printCellInformation(cell));
    statusBar->SetStatusText(status);
}

} // namespace ui
} // namespace vulcan

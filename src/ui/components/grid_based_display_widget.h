/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UI_COMPONENTS_GRID_BASED_DISPLAY_WIDGET_H
#define UI_COMPONENTS_GRID_BASED_DISPLAY_WIDGET_H

#include <wx/wx.h>
#include <wx/glcanvas.h>
#include "ui/components/open_gl_widget.h"
#include "core/point.h"

namespace vulcan
{
namespace ui
{

/**
* GridBasedDisplayWidget is an abstract base class for any widgets that use a grid as their
* primary source of displayed information.
*
* The base class handles events for scrolling the map around and zooming in and out. The
* viewport for rendering the grid is established and then the renderWidget() method is
* called so that the subclass can do the actual drawing.
*
* The setGridHeight() method also needs to be called to give the height of the grid to be displayed.
* The grid height is used to properly scale the perspective so zooming works correctly and the full
* grid is correctly displayed.
*
* The GridBasedDisplayWidget also supports output of the cell over which the mouse is hovering to
* a status bar along the bottom of the UI. To use this feature, provide the display widget with
* an instance of wxStatusBar via the setStatusBar() method.
*/
class GridBasedDisplayWidget : public OpenGLWidget
                             , public GLMouseHandler
{
public:

    GridBasedDisplayWidget(wxWindow* parent,
                           wxWindowID id = wxID_ANY,
                           const wxPoint& pos = wxDefaultPosition,
                           const wxSize& size = wxDefaultSize,
                           long style = 0,
                           const wxString& name = wxString((const wxChar*)("GLCanvas")),
                           const wxPalette& palette = wxNullPalette);

    virtual ~GridBasedDisplayWidget(void);

    void setStatusBar(wxStatusBar* status);

    // Enable/disable the ability for the user to click-and-drag to scroll the map around the screen
    void enableScrolling(void);
    void disableScrolling(void);

    /**
    * mousePosition retrieves the global coordinate of the current mouse position.
    */
    Point<float> mousePosition(void) const { return previousMousePosition; }
    
    /**
    * convertWorldToGrid converts a world coordinate to a grid coordinate using the current grid being
    * used as the basis for rendering by a GridBasedDisplayWidget subclass.
    *
    * \param    world           World point to be converted
    */
    virtual Point<int> convertWorldToGrid(const Point<float>& world) const = 0;

    // GLMouseHandler interface
    virtual GLEventStatus handleMouseMoved(const GLMouseEvent& event);

protected:
    
    /**
    * printCellInformation is called when creating the string that goes in the status bar. The method will
    * be called whenever the cell the mouse is in changes. The GridBasedDisplayWidget will draw the cell
    * location and global position of the cell. If any additional information is desired, return it in
    * the string and it will be appended to the above description.
    * 
    * \param    cell            Cell the mouse is hovering over
    * \return   Information about the cell in string form.
    */
    virtual std::string printCellInformation(Point<int> cell) { return std::string(""); }

    /**
    * setGridDimensions sets the dimensions of the grid for determining the camera position needed to
    * display the desired portion of the grid.
    *
    * Both measurements are in grid coordinates
    *
    * \param    width       Width of the grid
    * \param    height      Height of the grid
    */
    void setGridDimensions(float width, float height);

private:

    void displayMousePositionOnStatusBar(const GLMouseEvent& event);

    wxStatusBar* statusBar;

    bool leftWasDownOnLastEvent;
    Point<float> previousMousePosition;

    float displayedGridWidth;
    float displayedGridHeight;
};

}
}

#endif // UI_COMPONENTS_GRID_BASED_DISPLAY_WIDGET_H

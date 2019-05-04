/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     metric_editor_widget.h
* \author   Collin Johnson
* 
* Definition of MetricEditorWidget.
*/

#ifndef UI_MAPEDITOR_METRIC_EDITOR_WIDGET_H
#define UI_MAPEDITOR_METRIC_EDITOR_WIDGET_H

#include <ui/components/grid_based_display_widget.h>
#include <ui/common/ui_params.h>
#include <hssh/local_metric/lpm.h>
#include <core/line.h>
#include <core/point.h>
#include <utils/mutex.h>
#include <memory>

namespace vulcan
{
namespace ui
{

class OccupancyGridRenderer;
    
/**
* MetricEditorWidget draws the LPM and provides mouse handling via the OpenGLWidget for editing the LPM.
*/
class MetricEditorWidget : public GridBasedDisplayWidget
{
public:
    
    MetricEditorWidget(wxWindow* parent,
                       wxWindowID id = wxID_ANY,
                       const wxPoint& pos = wxDefaultPosition,
                       const wxSize& size = wxDefaultSize,
                       long style = 0,
                       const wxString& name = wxString((const wxChar*)("GLCanvas")),
                       const wxPalette& palette = wxNullPalette);
    
    virtual ~MetricEditorWidget(void);
    
    /**
    * setParams sets the parameters that control the display of the LPM.
    */
    void setParams(const ui_params_t& params);
    
    /**
    * setLPM sets the LPM to be drawn.
    */
    void setLPM(std::shared_ptr<hssh::LocalPerceptualMap> lpm);
    
    /**
    * changedLPM sets a flag indicating the LPM was recently changed.
    */
    void changedLPM(void) { lpmIsDirty_ = true; }
    
    /**
    * setHoverCell sets the cell over which the mouse is currently hovering in edit mode.
    * 
    * Only the hover cell or the hover line can be used, whichever was most recently set is used.
    * 
    * \param    cell        Cell mouse is over
    * \param    type        Type of the cell
    */
    void setHoverCell(Point<int> cell, hssh::cell_type_t type);
    
    /**
    * setHoverLine sets the line the user is currently creating to fill-in a whole section of the LPM.
    * 
    * Only the hover cell or the hover line can be used, whichever was most recently set is used.
    * 
    * \param    line        Line the user has created
    * \param    type        Type of the line
    */
    void setHoverLine(Line<int> line, hssh::cell_type_t type);
    
    /**
    * setSelectedCells sets the cells currently selected by the user during this mouse click.
    */
    void setSelectedCells(const std::vector<Point<int>>& selected);
    
    /**
    * shouldRenderHover sets a flag indicating if the hover and selected cell information should be shown.
    */
    void shouldRenderHover(bool show) { showHover = show; }
    
    // GridBasedDisplayWidget interface
    virtual Point<int> convertWorldToGrid(const Point<float>& world) const;
    
private:
    
    std::unique_ptr<OccupancyGridRenderer>              lpmRenderer;
    std::shared_ptr<hssh::LocalPerceptualMap> lpm;
    bool                                      lpmIsDirty_;
    
    Point<int>              hoverCell;
    Line<int>               hoverLine;
    std::vector<Point<int>> selectedCells;    // cells selected during the current click
    bool                          usingHoverCell;   // if the hover cell isn't used, then the hoverLine will be used
    bool                          showHover;        // flag indicating if the hover stuff should be shown
    hssh::cell_type_t             hoverType;        // selected cell type must be the same as the hover type
    
    lpm_display_params_t params;
    
    utils::Mutex lpmLock;
    
    // OpenGLWidget interface
    virtual void renderWidget(void);
    
    void       renderHoverCell(void);
    void       renderHoverLine(void);
    void       renderSelectedCells(void);
    GLColor selectCellColor(void);
};

}
}

#endif // UI_MAPEDITOR_METRIC_EDITOR_WIDGET_H

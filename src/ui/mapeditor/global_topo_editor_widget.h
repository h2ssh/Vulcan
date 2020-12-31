/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_topo_editor_widget.h
* \author   Collin Johnson
* 
* Definition of GlobalTopoEditorWidget.
*/

#ifndef UI_MAPEDITOR_GLOBAL_TOPO_EDITOR_WIDGET_H
#define UI_MAPEDITOR_GLOBAL_TOPO_EDITOR_WIDGET_H

#include "ui/components/open_gl_widget.h"

namespace vulcan
{
namespace ui
{
    
class GlobalTopoEditorWidget : public OpenGLWidget
{
public:
    
    GlobalTopoEditorWidget(wxWindow* parent,
                           wxWindowID id = wxID_ANY,
                           const wxPoint& pos = wxDefaultPosition,
                           const wxSize& size = wxDefaultSize,
                           long style = 0,
                           const wxString& name = wxString((const wxChar*)("GLCanvas")),
                           const wxPalette& palette = wxNullPalette);
    
    virtual ~GlobalTopoEditorWidget(void);
    
private:
    
    // OpenGLWidget interface
    virtual void renderWidget(void) { }
};
    
}
}

#endif // UI_MAPEDITOR_GLOBAL_TOPO_EDITOR_WIDGET_H

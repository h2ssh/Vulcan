/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     global_topo_editor_widget.cpp
 * \author   Collin Johnson
 *
 * Implementation of GlobalTopoEditorWidget.
 */

#include "ui/mapeditor/global_topo_editor_widget.h"

namespace vulcan
{
namespace ui
{

GlobalTopoEditorWidget::GlobalTopoEditorWidget(wxWindow* parent,
                                               wxWindowID id,
                                               const wxPoint& pos,
                                               const wxSize& size,
                                               long style,
                                               const wxString& name,
                                               const wxPalette& palette)
: OpenGLWidget(parent, id, pos, size, style, name, palette)
{
}


GlobalTopoEditorWidget::~GlobalTopoEditorWidget(void)
{
}

}   // namespace ui
}   // namespace vulcan

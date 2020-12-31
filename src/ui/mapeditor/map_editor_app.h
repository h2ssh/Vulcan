/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UI_MAPEDITOR_MAP_EDITOR_APP_H
#define UI_MAPEDITOR_MAP_EDITOR_APP_H

#include <wx/wx.h>

namespace vulcan
{
namespace ui
{

class MapEditorFrame;

/**
 * MapEditorApp is where the main function is called.
 */
class MapEditorApp : public wxApp
{
public:
    virtual bool OnInit(void);
    virtual int OnExit(void);

private:
    MapEditorFrame* mainFrame;
};

DECLARE_APP(MapEditorApp)

}   // namespace ui
}   // namespace vulcan

#endif   // UI_MAPEDITOR_MAP_EDITOR_APP_H

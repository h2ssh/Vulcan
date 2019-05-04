/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     map_editor_frame.h
* \author   Collin Johnson
* 
* Definition of MapEditorFrame.
*/

#ifndef UI_MAPEDITOR_MAP_EDITOR_FRAME_H
#define UI_MAPEDITOR_MAP_EDITOR_FRAME_H

#include <ui/mapeditor/map_editor.h>
#include <memory>

namespace vulcan
{
namespace hssh { struct local_topology_params_t; }
namespace ui
{
    
class  MetricEditorPanel;
class  LocalTopoEditorPanel;
class  GlobalTopoEditorPanel;
struct ui_params_t;
    
/**
* MapEditorFrame
*/
class MapEditorFrame : public EditorFrame
{
public:
    
    /**
    * Constructor for MapEditorFrame.
    * 
    * \param    params              Parameters governing the behavior of the UI
    * \param    localTopoParams     Parameters for doing the local topo editing
    */
    MapEditorFrame(const ui_params_t& params, const hssh::local_topology_params_t& localTopoParams);
    
    /**
    * Destructor for MapEditorFrame.
    */
    virtual ~MapEditorFrame(void);
    
private:
    
    void setupMetricPanel    (const ui_params_t& params);
    void setupLocalTopoPanel (const ui_params_t& params, const hssh::local_topology_params_t& localTopoParams);
    void setupGlobalTopoPanel(const ui_params_t& params);
    
    MetricEditorPanel*     metricPanel_;
    LocalTopoEditorPanel*  localTopoPanel_;
    GlobalTopoEditorPanel* globalTopoPanel_;
};
    
}
}

#endif // UI_MAPEDITOR_MAP_EDITOR_FRAME_H 

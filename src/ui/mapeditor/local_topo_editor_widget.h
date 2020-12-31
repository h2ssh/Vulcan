/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_editor_widget.h
* \author   Collin Johnson
* 
* Definition of LocalTopoEditorWidget.
*/

#ifndef UI_MAPEDITOR_LOCAL_TOPO_EDITOR_WIDGET_H
#define UI_MAPEDITOR_LOCAL_TOPO_EDITOR_WIDGET_H

#include "ui/components/grid_based_display_widget.h"
#include "hssh/local_topological/gateway.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "utils/mutex.h"
#include <boost/optional.hpp>
#include <vector>

namespace vulcan
{
namespace hssh { class AreaHypothesis; }
namespace ui
{

class  AreaHypothesisRenderer;
class  VoronoiSkeletonGridRenderer;
class  GatewaysRenderer;
struct ui_params_t;

/**
* LocalTopoEditorWidget draws the information needed for editing the local topo map:
* 
*   - The LPM
*   - The current area boundaries
*   - The gateways
* 
* The area boundaries are drawn using a provided color.
* 
* For gateway editing, the hover and selected gateways can be specified. The hover gateway is shown slightly
* translucent, while the selected gateway is less translucent. This allows for seeing exactly where the click occurred
* vs. the currently modified gateway.
*/
class LocalTopoEditorWidget : public GridBasedDisplayWidget
{
public:
    
    LocalTopoEditorWidget(wxWindow* parent,
                          wxWindowID id = wxID_ANY,
                          const wxPoint& pos = wxDefaultPosition,
                          const wxSize& size = wxDefaultSize,
                          long style = 0,
                          const wxString& name = wxString((const wxChar*)("GLCanvas")),
                          const wxPalette& palette = wxNullPalette);
    
    virtual ~LocalTopoEditorWidget(void);
    
    void setParams(const ui_params_t& params);
    
    void setSkeleton(hssh::VoronoiSkeletonGrid skeleton);
    void setAreas   (const std::vector<hssh::AreaHypothesis*>& areas);
    void setGateways(const std::vector<hssh::Gateway>& gateways);
    
    void setHoverGateway(boost::optional<hssh::Gateway> gateway);
    void setSelectedGateway(boost::optional<hssh::Gateway> gateway);
    
    // GridBasedDisplayWidget interface
    Point<int> convertWorldToGrid(const Point<float>& world) const override;
    
private:
    
    hssh::VoronoiSkeletonGrid skeleton_;
    bool isDirtySkeleton_;
    
    std::vector<hssh::AreaHypothesis*> areas_;
    std::vector<hssh::Gateway> gateways_;
    
    boost::optional<hssh::Gateway> hoverGateway_;
    boost::optional<hssh::Gateway> selectedGateway_;
    
    std::unique_ptr<VoronoiSkeletonGridRenderer> skeletonRenderer_;
    std::unique_ptr<GatewaysRenderer> gatewayRenderer_;
    std::unique_ptr<AreaHypothesisRenderer> areaRenderer_;
    
    utils::Mutex dataLock_;
    
    // OpenGLWidget interface
    virtual void renderWidget(void);
    
    void renderAreas(void) const;
    void renderGateways(void) const;
};
    
} // namespace ui
} // namespace vulcan

#endif // UI_MAPEDITOR_LOCAL_TOPO_EDITOR_WIDGET_H

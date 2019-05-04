/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_topo_display_widget.h
* \author   Collin Johnson
*
* Declaration of GlobalTopoDisplayWidget.
*/

#ifndef UI_DEBUG_GLOBAL_TOPO_DISPLAY_WIDGET_H
#define UI_DEBUG_GLOBAL_TOPO_DISPLAY_WIDGET_H

#include <hssh/global_topological/state.h>
#include <hssh/global_topological/debug/hypothesis_tree.h>
#include <hssh/global_topological/utils/metric_map_cache.h>
#include <hssh/local_topological/event.h>
#include <ui/common/ui_params.h>
#include <ui/common/ui_forward_declarations.h>
#include <ui/components/open_gl_widget.h>
#include <utils/mutex.h>
#include <wx/wx.h>
#include <map>
#include <memory>

namespace vulcan
{
namespace ui
{

class GraphViewTopologicalMapRenderer;
class PlaceViewTopologicalMapRenderer;
class TopologicalMapRenderer;
class HypothesisTreeRenderer;
class GlobalTopoMapPreviewer;

/**
* GlobalTopoDisplayWidget is an OpenGl-based widget for rendering the global topology layer
* in the HSSH. The global topo is displayed with places as circles and edges as lines. The
* places and paths are selectable, which will cause information about the particular place/path
* to be displayed on the screen.
*
* The location of the robot is tracked and visible by a highlighted places or path edge.
*/
class GlobalTopoDisplayWidget : public OpenGLWidget
{
public:

    enum active_view_t
    {
        PLACE_VIEW,
        GRAPH_VIEW,
        HYPOTHESIS_TREE
    };

    /**
    * Constructor for GlobalTopoDisplayWidget.
    */
    GlobalTopoDisplayWidget(wxWindow* parent,
                            wxWindowID id = wxID_ANY,
                            const wxPoint& pos = wxDefaultPosition,
                            const wxSize& size = wxDefaultSize,
                            long style = 0,
                            const wxString& name = wxString((const wxChar*)("GLCanvas")),
                            const wxPalette& palette = wxNullPalette);
    
    /**
    * Destructor for GlobalTopoDisplayWidget.
    */
    ~GlobalTopoDisplayWidget(void);

    void setStatusBar(wxStatusBar* status) { statusBar     = status; }

    /**
    * setWidgetParams sets the widget parameters for the display of the various bits of the global topology.
    */
    void setWidgetParams(const global_topo_display_params_t& globalTopoParams, const lpm_display_params_t& localMetricParams);

    /**
    * getDisplayedMap retrieves the map currently being drawn.
    */
    hssh::hypothesis_tree_node_t getDisplayedMap(void) const;

    /**
    * getHoverMap retrieves the map over which the mouse is currently hovering displaying the hypothesis tree.
    */
    hssh::hypothesis_tree_node_t getHoverMap(void) const;

    // Methods for controlling the rendered
    void setActiveView(active_view_t viewToShow);
    void setIdToShow  (hssh::Id id);
    void showBestMap  (bool show);
    void setTreeOfMaps(const std::shared_ptr<hssh::TreeOfMaps>& maps);
    void setMapCache  (const hssh::MetricMapCache& cache);
    void clearMaps    (void);

    void handleData(const hssh::TopologicalState& map, const std::string& channel);
    void handleData(const hssh::HypothesisTree& tree, const std::string& channel);
    void handleData(const hssh::LocalAreaEventVec& events, const std::string& channel);

private:

    // Renderers
    std::unique_ptr<PlaceViewTopologicalMapRenderer> placeViewRenderer;
    std::unique_ptr<GraphViewTopologicalMapRenderer> graphViewRenderer;
    TopologicalMapRenderer* activeGraphRenderer;
    std::unique_ptr<HypothesisTreeRenderer> treeRenderer;
    std::unique_ptr<GlobalTopoMapPreviewer> mapPreviewer;

    // Data to be rendered
    hssh::TopologicalState bestTopoMap;
    std::shared_ptr<hssh::TreeOfMaps> treeOfMaps;
    hssh::HypothesisTree hypTree;
    hssh::MetricMapCache placeManager;

    bool shouldShowBestMap;
    active_view_t viewToShow;
    
    hssh::Id mapToShowId;
    hssh::Id hoverId;

    bool haveMap;
    bool showingNewMap;
    bool showingNewTree;

    global_topo_display_params_t params;
    wxStatusBar* statusBar;
    utils::Mutex dataLock;

    // OpenGLWidget interface
    void getViewportBoundary(Point<int>& bottomLeft, int& width, int& height);
    void renderWidget(void);
    void drawHypothesisTree(void);
    
    // Event handlers
    void mouseMotion  (wxMouseEvent& event);
    void mouseLeftDown(wxMouseEvent& event);
    void mouseLeftUp  (wxMouseEvent& event);
    void mouseRightUp (wxMouseEvent& event);
    
    hssh::Id nearestTreeNode(const Point<uint16_t>& mousePoint);
    void previewNode        (hssh::Id nodeId);
    void highlightHoverNode (hssh::Id nodeId);

    DECLARE_EVENT_TABLE()
};

} // namespace ui
} // namespace vulcan

#endif // UI_DEBUG_GLOBAL_TOPO_DISPLAY_WIDGET_H

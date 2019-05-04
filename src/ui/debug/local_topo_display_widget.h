/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_display_widget.h
* \author   Collin Johnson
*
* Declaration of LocalTopoDisplayWidget.
*/

#ifndef UI_DEBUG_LOCAL_TOPO_DISPLAY_WIDGET_H
#define UI_DEBUG_LOCAL_TOPO_DISPLAY_WIDGET_H

#include <ui/common/ui_params.h>
#include <ui/components/grid_based_display_widget.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <hssh/local_topological/gateway.h>
#include <hssh/local_topological/frontier.h>
#include <hssh/local_topological/location.h>
#include <hssh/local_topological/evaluation/heat_map.h>
#include <hssh/local_topological/area_detection/local_topo_isovist_field.h>
#include <hssh/local_topological/area_detection/gateways/isovist_gradients.h>
#include <hssh/local_topological/area_detection/gateways/isovist_maxima.h>
#include <hssh/local_topological/debug_info.h>
#include <utils/mutex.h>
#include <utils/visibility_graph.h>
#include <utils/visibility_graph_feature.h>
#include <memory>

namespace vulcan
{
namespace hssh { class LocalAreaEvent; }
namespace hssh { class LocalTopoMap; }
namespace hssh { class LocalPose; }
namespace ui
{

class GridCellSelector;
class VoronoiSkeletonGridRenderer;
class GatewaysRenderer;
class AreaGraphRenderer;
class VisibilityGraphRenderer;
class AreaProposalRenderer;
class AreaHypothesisRenderer;
class LocalAreaRenderer;
class LocalAreaEventRenderer;
class FrontiersRenderer;
class IsovistRenderer;
class VoronoiIsovistGradientsRenderer;
class LabelingCSPPlayer;


/**
* LocalTopoDisplayWidget is responsible for drawing the output of the local topology layer onto
* the screen. The base of the local topology information is the VoronoiSkeletonGrid, from which all topological
* information is extracted. The VoronoiSkeletonGrid serves as the background for the other pieces of data
* produced by the local topology modules, like frontiers, gateways, and the small-scale star.
*/
class LocalTopoDisplayWidget : public GridBasedDisplayWidget
{
public:

    LocalTopoDisplayWidget(wxWindow* parent,
                           wxWindowID id = wxID_ANY,
                           const wxPoint& pos = wxDefaultPosition,
                           const wxSize& size = wxDefaultSize,
                           long style = 0,
                           const wxString& name = wxString((const wxChar*)("GLCanvas")),
                           const wxPalette& palette = wxNullPalette);

    virtual ~LocalTopoDisplayWidget(void);

    void setWidgetParams(const local_topo_display_params_t& params);

    // Various event handlers
    void showVoronoiGrid     (bool show);
    void showDistanceGradient(bool show);
    void showFullSkeleton    (bool show);
    void showReducedSkeleton (bool show);
    void shouldFollowRobot   (bool follow);

    void showFrontiers     (bool show);
    void showSmallScaleStar(bool show);
    void showAreaGateways  (bool show);

    void showGateways     (bool show) { shouldShowGateways = show; }
    void showNormals      (bool show) { shouldShowNormals = show; }
    void showAreaGraph    (bool show) { shouldShowAreaGraph = show; }
    void showAreas        (int show) { areasToShow = show; }
    void showEvents       (bool show) { shouldShowEvents_ = show; }
    
    void showIsovist         (bool show);
    void selectIsovists      (bool select);
    void showIsovistField    (bool show) { shouldShowIsovistField     = show; }
    void showIsovistDerivField(bool show) { shouldShowIsovistDerivField_ = show; }
    void showIsovistGradients(bool show) { shouldShowGradients_       = show; }
    void showGradientMaxima  (bool show) { shouldShowGradientMaxima_  = show; }
    void showGatewayProbabilities(bool show) { shouldShowProbabilities_ = show; }
    void showVisibilityGraph(bool show) { shouldShowVisibilityGraph_ = show; }
    
    void setHypothesisValueToDisplay(int toDisplay);
    
    void setSkeleton(std::shared_ptr<hssh::VoronoiSkeletonGrid> skeleton);
    void setGateways(const std::vector<hssh::Gateway>& gateways); // The LocalTopoPanel manages which gateways should be shown
    void setLocalTopoMap(std::shared_ptr<hssh::LocalTopoMap> map);
    void setLocation(const hssh::LocalLocation& location);
    void setVisibleHypotheses(const std::vector<hssh::DebugHypothesis>& hypotheses);
    void setSelectedHypothesis(const hssh::DebugHypothesis* hypothesis);
    void setHypothesisFeatureToShow(int featureIndex);
    void setEvent(std::shared_ptr<hssh::LocalAreaEvent> event);
    void setIsovistField(std::shared_ptr<hssh::VoronoiIsovistField> field, utils::Isovist::Scalar scalar);
    void setIsovistGradients(std::shared_ptr<hssh::VoronoiIsovistGradients> gradients, std::shared_ptr<hssh::VoronoiIsovistMaxima> maxima);
    void setGatewayProbabilities(const hssh::CellToTypeMap<double>& cellProbabilities);

    void setCSPPlayer(LabelingCSPPlayer* player);

    void useHeatMap(bool show) { showTopoMapAsHeatMap_ = show; }
    void setHeatMap(const hssh::HeatMapStatistics& heatMap);

    void setVisibilityGraphFeature(utils::VisibilityGraphFeatureType featureType);

    // Data handlers
    void handleData(const hssh::LocalPose&                pose, const std::string& channel);
    void handleData(const hssh::local_area_debug_info_t&  info, const std::string& channel);

    // GLMouseHandler interface -- override these two methods for additional debugging. They'll both be forwarded up
    // to the base class as well
    GLEventStatus handleRightMouseUp(const GLMouseEvent& event) override;

private:

    // Renderers
    std::unique_ptr<VoronoiSkeletonGridRenderer>     gridRenderer;
    std::unique_ptr<GatewaysRenderer>                gatewaysRenderer;
    std::unique_ptr<AreaGraphRenderer>               graphRenderer;
    std::unique_ptr<VisibilityGraphRenderer>         visibilityGraphRenderer_;
    std::unique_ptr<AreaHypothesisRenderer>          hypothesisRenderer;
    std::unique_ptr<LocalAreaRenderer>               areaRenderer_;
    std::unique_ptr<LocalAreaEventRenderer>          eventRenderer_;
    std::unique_ptr<IsovistRenderer>                 isovistRenderer;
    std::unique_ptr<VoronoiIsovistGradientsRenderer> gradientsRenderer_;
    LabelingCSPPlayer* cspPlayer_;

    // Data
    pose_t robotPose;

    std::shared_ptr<hssh::VoronoiSkeletonGrid> grid;
    std::shared_ptr<hssh::LocalTopoMap>        topoMap_;
    std::shared_ptr<hssh::LocalAreaEvent>      event_;
    hssh::LocalLocation                        location_;
    std::size_t gridWidth;
    std::size_t gridHeight;

    std::vector<hssh::Gateway>     gateways;
    hssh::local_area_debug_info_t  areaInfo;
    std::set<hssh::cell_t>         selectedSources;
    
    std::vector<hssh::DebugHypothesis>   visibleHypotheses_;
    const hssh::DebugHypothesis*         selectedHypothesis_;

    bool showTopoMapAsHeatMap_;
    hssh::HeatMapStatistics heatMap_;
    
    std::vector<hssh::cell_t>                      selectedIsovists_;
    std::vector<GLColor>                           isovistColors_;
    std::shared_ptr<hssh::VoronoiIsovistField>     field_;
    std::shared_ptr<hssh::VoronoiIsovistGradients> gradients_;
    std::shared_ptr<hssh::VoronoiIsovistMaxima>    maxima_;
    utils::Isovist::Scalar                         scalar_;
    
    hssh::CellToTypeMap<double> probabilities_;
    
    std::shared_ptr<GridCellSelector> isovistSelector_;

    utils::VisibilityGraph visGraph_;
    utils::VisibilityGraphFeature visGraphFeature_;
    utils::VisibilityGraphFeatureType visGraphFeatureType_;

    bool havePath;

    // Flags
    bool gridIsDirty;           // the grid becomes dirty any time a new grid arrives, or the grid rendering options change
    bool gradientsAreDirty_;    // if the gradients change, the rendering info needs to be regenerated
    bool probabilitiesAreDirty_;    // probability information about cells is dirty and needs to be regenerated
    bool visibilityGraphIsDirty_;   // whenever the AreaGraph changes, then the visibility graph becomes dirty

    bool doFollowRobot;
    bool showVoronoiGrid_;
    bool shouldShowGateways;
    bool shouldShowNormals;
    bool shouldShowAreaGraph;
    bool shouldShowVisibilityGraph_;
    int  areaHypothesisValue_;
    int  hypFeatureIndex_;
    double hypFeatureNormalizer_;
    bool shouldShowFrontiers;
    int  areasToShow;
    bool shouldShowEvents_;
    bool shouldShowIsovist;
    bool shouldSelectIsovists;
    bool shouldShowIsovistField;
    bool shouldShowIsovistDerivField_;
    bool shouldShowGradients_;
    bool shouldShowGradientMaxima_;
    bool shouldShowProbabilities_;
    
    // Utilities
    local_topo_display_params_t params;

    mutable utils::Mutex dataLock;
    
    // GridBasedDisplayWidget interface
    void             renderWidget        (void) override;
    Point<int> convertWorldToGrid  (const Point<float>& world) const override;
    std::string      printCellInformation(Point<int> cell) override;
    
    void renderGrid(void);
    void renderIsovists(void);
    void renderGateways(void);
    void renderAreas(void);
    
    void renderPose      (void);
    void renderProposals (const std::vector<hssh::AreaProposal>&  proposals);
    void renderHypothesis(const hssh::DebugHypothesis& hypothesis);

    std::string printVoronoiDistance   (hssh::cell_t cell) const;
    std::string printHypothesisFeatures(void)              const;
    std::string printIsovistInformation(hssh::cell_t cell) const;
};

} // namespace ui
} // namespace vulcan

#endif // UI_DEBUG_LOCAL_TOPO_DISPLAY_WIDGET_H

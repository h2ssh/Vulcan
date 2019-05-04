/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     localtopo_panel.h
* \author   Collin Johnson
*
* Declaration of LocalTopoPanel.
*/

#ifndef UI_DEBUG_LOCALTOPO_PANEL_H
#define UI_DEBUG_LOCALTOPO_PANEL_H

#include <ui/common/ui_panel.h>
#include <ui/common/grid_object_selector.h>
#include <ui/common/object_selector_event_handler.h>
#include <hssh/local_topological/debug_info.h>
#include <hssh/local_topological/event.h>
#include <hssh/local_topological/area_detection/labeling/csp_debug.h>
#include <utils/mutex.h>
#include <wx/wx.h>
#include <wx/tglbtn.h>
#include <atomic>

namespace vulcan
{
namespace hssh { class GatewayClassifier; }
namespace hssh { class LocalTopoMap; }
namespace hssh { class VoronoiIsovistField; }
namespace hssh { class VoronoiIsovistGradients; }
namespace hssh { class VoronoiIsovistMaxima; }
namespace ui
{

class LocalTopoDisplayWidget;
class LabelingCSPPlayer;
struct ui_params_t;

struct local_topo_panel_widgets_t
{
    LocalTopoDisplayWidget* displayWidget = nullptr;
    wxCheckBox*             showVoronoiGridBox = nullptr;
    wxCheckBox*             showDistanceGradientBox = nullptr;
    wxCheckBox*             showFullSkeletonBox = nullptr;
    wxCheckBox*             showReducedSkeletonBox = nullptr;
    wxCheckBox*             showFrontiersBox = nullptr;
    wxCheckBox*             centerOnRobotBox = nullptr;
    wxChoice*               gatewayTypeChoice = nullptr;
    wxCheckBox*             showAreaGraphBox = nullptr;
    wxRadioBox*             areaHypothesisValueRadio = nullptr;
    wxStaticText*           hypValueText = nullptr;
    wxChoice*               hypFeatureChoice = nullptr;
    wxRadioBox*             hypothesesToShowRadio = nullptr;
    wxStaticText*           labelDistributionText = nullptr;
    wxSlider*               cspIterationSlider = nullptr;
    wxSlider*               cspSpeedSlider = nullptr;
    wxCheckBox*             showHeatMapBox = nullptr;
    wxTextCtrl*             numPathsText = nullptr;
    wxListBox*              eventsList = nullptr;
    wxCheckBox*             showEventsVisualizationBox = nullptr;
    wxRadioBox*             isovistLocationRadio = nullptr;
    wxChoice*               isovistScalarChoice = nullptr;
    wxCheckBox*             showIsovistBox = nullptr;
    wxCheckBox*             showIsovistFieldBox = nullptr;
    wxCheckBox*             showIsovistDerivFieldBox = nullptr;
    wxToggleButton*         selectIsovistsButton = nullptr;
    wxCheckBox*             showGradientsBox = nullptr;
    wxCheckBox*             showLocalMaximaBox = nullptr;
    wxCheckBox*             showGatewayProbabilitiesBox = nullptr;
    wxSlider*               gatewayProbabilitySlider = nullptr;
    wxRadioBox*             classifierToUseRadio = nullptr;
    wxCheckBox*             showVisibilityGraphBox = nullptr;
    wxChoice*               visibilityFeatureChoice = nullptr;
};


/**
* LocalTopoPanel handles events for the local topo display panel.
*/
class LocalTopoPanel : public UIPanel,
                       public ObjectSelectorEventHandler<const hssh::DebugHypothesis*>
{
public:

    /**
    * Constructor for LocalTopoPanel.
    *
    * \param    params          Parameters for the rendering
    * \param    widgets         Interactive widgets on the panel
    */
    LocalTopoPanel(const ui_params_t& params, const local_topo_panel_widgets_t& widgets);

    /**
    * Destructor for LocalTopoPanel.
    */
    virtual ~LocalTopoPanel(void);

    // UIPanel interface
    void setup       (wxGLContext* context, wxStatusBar* statusBar) override;
    void subscribe   (system::ModuleCommunicator& producer) override;
    void setConsumer (system::ModuleCommunicator* consumer) override;
    void update      (void) override;
    void saveSettings(utils::ConfigFileWriter& config) override;
    void loadSettings(const utils::ConfigFile& config) override;

    // Data handlers
    void handleData(const hssh::VoronoiSkeletonGrid&     grid,   const std::string& channel);
    void handleData(const hssh::gateway_debug_info_t&    debug,  const std::string& channel);
    void handleData(const hssh::local_area_debug_info_t& debug,  const std::string& channel);
    void handleData(const hssh::CSPDebugInfo&            debug,  const std::string& channel);
    void handleData(const hssh::LocalTopoMap&            map,    const std::string& channel);
    void handleData(const hssh::LocalAreaEventVec&       events, const std::string& channel);

    // ObjectSelectorEventHandler interface
    void objectEntered(const hssh::DebugHypothesis*& object) override;
    void objectExited(const hssh::DebugHypothesis*& object) override;
    void objectSelected(const hssh::DebugHypothesis*& object) override;

private:

    // Internal handling
    local_topo_panel_widgets_t widgets;

    std::shared_ptr<hssh::VoronoiSkeletonGrid>         grid;
    hssh::gateway_debug_info_t                         gatewayInfo;
    hssh::local_area_debug_info_t                      areaInfo_;
    hssh::CSPDebugInfo                                 cspInfo_;
    std::vector<std::string>                           featureNames_;
    std::vector<std::shared_ptr<hssh::LocalAreaEvent>> events_;
    std::shared_ptr<hssh::VoronoiIsovistField>         field_;
    std::shared_ptr<hssh::VoronoiIsovistGradients>     gradients_;
    std::shared_ptr<hssh::VoronoiIsovistMaxima>        maxima_;
    std::shared_ptr<hssh::LocalTopoMap>                topoMap_;
    std::shared_ptr<hssh::GatewayClassifier>           classifier_;
    GridObjectSelector<const hssh::DebugHypothesis*> hypothesisSelector_;

    std::unique_ptr<LabelingCSPPlayer> cspPlayer_;

    bool gatewaysAreDirty_;
    bool hypothesesAreDirty_;
    bool cspInfoIsDirty_;
    bool eventsAreDirty_;

    std::size_t gatewaysIndex;

    int hypothesesToShow_;
    int hypFeatureIndex_;
    const hssh::DebugHypothesis* hoverHypothesis_;

    double gatewayProbCutoff_ = 0.5;

    system::ModuleCommunicator* consumer_ = nullptr;
    utils::Mutex dataLock_;

    // Helpers
    void updateGateways(void);
    void updateHypotheses(void);
    void changeVisibleHypotheses(const std::vector<hssh::DebugHypothesis>& visibleHypotheses);
    void updateCSPInfo(void);
    void updateEvents(void);
    void calculateIsovistField(void);
    void calculateProbableGateways(void);

    // Event handlers
    void modeRadioChanged       (wxCommandEvent& event);
    void showVoronoiGrid        (wxCommandEvent& event);
    void showDistanceGradient   (wxCommandEvent& event);
    void showFullSkeleton       (wxCommandEvent& event);
    void showReducedSkeleton    (wxCommandEvent& event);
    void centerOnRobot          (wxCommandEvent& event);
    void showGateways           (wxCommandEvent& event);
    void gatewayChoiceChanged   (wxCommandEvent& event);
    void showNormals            (wxCommandEvent& event);
    void saveLocalTopoMapPressed(wxCommandEvent& event);
    void loadLocalTopoMapPressed(wxCommandEvent& event);
    void sendLocalTopoMapPressed(wxCommandEvent& event);
    void showAreaGraph          (wxCommandEvent& event);
    void showVisibilityGraph    (wxCommandEvent& event);
    void areaHypothesisValueChanged(wxCommandEvent& event);
    void hypFeatureChoiceChanged(wxCommandEvent& event);
    void hypothesesToShowChanged(wxCommandEvent& event);
    void showFrontiers          (wxCommandEvent& event);
    void showAreasChanged       (wxCommandEvent& event);
    void showSmallScaleStar     (wxCommandEvent& event);
    void showAreaGateways       (wxCommandEvent& event);

    void cspLoadPressed(wxCommandEvent& event);
    void cspPlayPressed(wxCommandEvent& event);
    void cspPausePressed(wxCommandEvent& event);
    void cspStopPressed(wxCommandEvent& event);
    void cspJumpToStartPressed(wxCommandEvent& event);
    void cspPrevIterationPressed(wxCommandEvent& event);
    void cspNextIterationPressed(wxCommandEvent& event);
    void cspJumpToEndPressed(wxCommandEvent& event);
    void iterationSliderChanged(wxCommandEvent& event);
    void speedSliderChanged(wxCommandEvent& event);

    void showHeatMapChanged(wxCommandEvent& event);
    void generateHeatMapPressed(wxCommandEvent& event);

    void showEventVisualizations(wxCommandEvent& event);
    void showIsovistChanged        (wxCommandEvent& event);
    void showIsovistFieldChanged   (wxCommandEvent& event);
    void showIsovistDerivFieldChanged(wxCommandEvent& event);
    void calculateIsovistPressed   (wxCommandEvent& event);
    void selectIsovistsChanged     (wxCommandEvent& event);
    void scalarChoiceChanged       (wxCommandEvent& event);
    void calculateGradientsPressed (wxCommandEvent& event);
    void showGradientsChanged      (wxCommandEvent& event);
    void showLocalMaximaChanged    (wxCommandEvent& event);
    void loadGatewayClassifierPressed(wxCommandEvent& event);
    void calculateGatewayProbabilitiesPressed(wxCommandEvent& event);
    void showGatewayProbabilitiesChanged(wxCommandEvent& event);
    void gatewayProbabilitySliderChanged(wxCommandEvent& event);
    void visibilityFeatureChoiceChanged(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};

} // namespace ui
} // namespace vulcan

#endif // UI_DEBUG_LOCALTOPO_PANEL_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_panel.cpp
* \author   Collin Johnson
*
* Definition of LocalTopoPanel.
*/

#include "ui/debug/local_topo_panel.h"
#include "ui/debug/local_topo_display_widget.h"
#include "ui/debug/debug_ui.h"
#include "ui/components/labeling_csp_player.h"
#include "ui/common/file_dialog_settings.h"
#include "hssh/local_metric/pose.h"
#include "hssh/local_topological/command.h"
#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/area_detection/local_topo_isovist_field.h"
#include "hssh/local_topological/area_detection/gateways/feature_extraction.h"
#include "hssh/local_topological/area_detection/gateways/gateway_classifier.h"
#include "hssh/local_topological/area_detection/gateways/isovist_gradients.h"
#include "hssh/local_topological/area_detection/gateways/isovist_maxima.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_features.h"
#include "system/module_communicator.h"
#include "utils/auto_mutex.h"
#include "utils/serialized_file_io.h"
#include "utils/visibility_graph_feature.h"
#include "utils/stub.h"
#include <cassert>

// For KDevelop code completion to work with the wxWidgets event table. It doesn't like the togglebutton.
namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(LocalTopoPanel, wxEvtHandler)
    EVT_RADIOBOX(ID_LOCAL_TOPO_MODE_RADIO, LocalTopoPanel::modeRadioChanged)
    EVT_CHECKBOX(ID_SHOW_VORONOI_GRID_BOX, LocalTopoPanel::showVoronoiGrid)
    EVT_CHECKBOX(ID_SHOW_DISTANCE_GRADIENT_BOX, LocalTopoPanel::showDistanceGradient)
    EVT_CHECKBOX(ID_SHOW_FULL_SKELETON_BOX,     LocalTopoPanel::showFullSkeleton)
    EVT_CHECKBOX(ID_SHOW_REDUCED_SKELETON_BOX,  LocalTopoPanel::showReducedSkeleton)
    EVT_CHECKBOX(ID_FOLLOW_ROBOT_TOPO_BOX,      LocalTopoPanel::centerOnRobot)
    EVT_CHECKBOX(ID_SHOW_GATEWAYS_BOX,          LocalTopoPanel::showGateways)
    EVT_CHOICE(ID_GATEWAY_TYPE_CHOICE,          LocalTopoPanel::gatewayChoiceChanged)
    EVT_CHECKBOX(ID_SHOW_NORMALS_BOX,           LocalTopoPanel::showNormals)
    EVT_BUTTON(ID_SAVE_LOCAL_TOPO_MAP_BUTTON,   LocalTopoPanel::saveLocalTopoMapPressed)
    EVT_BUTTON(ID_LOAD_LOCAL_TOPO_MAP_BUTTON,   LocalTopoPanel::loadLocalTopoMapPressed)
    EVT_BUTTON(ID_SEND_LOCAL_TOPO_MAP_BUTTON,   LocalTopoPanel::sendLocalTopoMapPressed)
    EVT_CHECKBOX(ID_SHOW_FRONTIERS_BOX,           LocalTopoPanel::showFrontiers)
    EVT_RADIOBOX(ID_SHOW_AREAS_BOX,               LocalTopoPanel::showAreasChanged)
    EVT_CHECKBOX(ID_SHOW_SMALL_STAR_BOX,          LocalTopoPanel::showSmallScaleStar)
    EVT_CHECKBOX(ID_SHOW_AREA_GATEWAYS_BOX,       LocalTopoPanel::showAreaGateways)
    EVT_CHECKBOX(ID_SHOW_AREA_GRAPH_BOX,        LocalTopoPanel::showAreaGraph)
    EVT_CHECKBOX(ID_SHOW_VISIBILITY_GRAPH_BOX,    LocalTopoPanel::showVisibilityGraph)
    EVT_RADIOBOX(ID_AREA_HYPOTHESIS_VALUE_RADIO,   LocalTopoPanel::areaHypothesisValueChanged)
    EVT_CHOICE(ID_HYP_FEATURE_CHOICE, LocalTopoPanel::hypFeatureChoiceChanged)
    EVT_RADIOBOX(ID_HYPOTHESES_TO_SHOW_RADIO,     LocalTopoPanel::hypothesesToShowChanged)
    EVT_BUTTON(ID_CSP_LOAD_BUTTON,              LocalTopoPanel::cspLoadPressed)
    EVT_BUTTON(ID_CSP_PLAY_BUTTON,              LocalTopoPanel::cspPlayPressed)
    EVT_BUTTON(ID_CSP_PAUSE_BUTTON,             LocalTopoPanel::cspPausePressed)
    EVT_BUTTON(ID_CSP_STOP_BUTTON,              LocalTopoPanel::cspStopPressed)
    EVT_BUTTON(ID_CSP_JUMP_TO_START_BUTTON,     LocalTopoPanel::cspJumpToStartPressed)
    EVT_BUTTON(ID_CSP_PREV_ITERATION_BUTTON,    LocalTopoPanel::cspPrevIterationPressed)
    EVT_BUTTON(ID_CSP_NEXT_ITERATION_BUTTON,    LocalTopoPanel::cspNextIterationPressed)
    EVT_BUTTON(ID_CSP_JUMP_TO_END_BUTTON,       LocalTopoPanel::cspJumpToEndPressed)
    EVT_SLIDER(ID_CSP_ITERATION_SLIDER,         LocalTopoPanel::iterationSliderChanged)
    EVT_SLIDER(ID_CSP_SPEED_SLIDER,             LocalTopoPanel::speedSliderChanged)
    EVT_CHECKBOX(ID_SHOW_LOCAL_HEAT_MAP_BOX,    LocalTopoPanel::showHeatMapChanged)
    EVT_BUTTON(ID_GENERATE_LOCAL_HEAT_MAP_BUTTON, LocalTopoPanel::generateHeatMapPressed)
    EVT_CHECKBOX(ID_SHOW_LOCAL_TOPO_EVENT_BOX,    LocalTopoPanel::showEventVisualizations)
    EVT_CHECKBOX(ID_SHOW_ISOVIST_BOX,             LocalTopoPanel::showIsovistChanged)
    EVT_CHECKBOX(ID_SHOW_ISOVIST_FIELD_BOX,       LocalTopoPanel::showIsovistFieldChanged)
    EVT_CHECKBOX(ID_SHOW_ISOVIST_DERIV_FIELD_BOX, LocalTopoPanel::showIsovistDerivFieldChanged)
    EVT_BUTTON(ID_CALCULATE_ISOVISTS_BUTTON,      LocalTopoPanel::calculateIsovistPressed)
    EVT_CHOICE(ID_FIELD_SCALAR_CHOICE,            LocalTopoPanel::scalarChoiceChanged)
    EVT_BUTTON(ID_CALCULATE_GRADIENTS_BUTTON,     LocalTopoPanel::calculateGradientsPressed)
    EVT_TOGGLEBUTTON(ID_SELECT_ISOVISTS_BUTTON,   LocalTopoPanel::selectIsovistsChanged)
    EVT_CHECKBOX(ID_SHOW_CELL_GRADIENTS_BOX,      LocalTopoPanel::showGradientsChanged)
    EVT_CHECKBOX(ID_SHOW_ISOVIST_MAXIMA_BOX,      LocalTopoPanel::showLocalMaximaChanged)
    EVT_BUTTON(ID_LOAD_GATEWAY_CLASSIFIER_BUTTON, LocalTopoPanel::loadGatewayClassifierPressed)
    EVT_BUTTON(ID_CALCULATE_GATEWAY_PROBABILITIES_BUTTON, LocalTopoPanel::calculateGatewayProbabilitiesPressed)
    EVT_CHECKBOX(ID_SHOW_GATEWAY_PROBABILITIES_BOX, LocalTopoPanel::showGatewayProbabilitiesChanged)
    EVT_SLIDER(ID_GATEWAY_CUTOFF_SLIDER,            LocalTopoPanel::gatewayProbabilitySliderChanged)
    EVT_CHOICE(ID_VISIBILITY_FEATURE_CHOICE,      LocalTopoPanel::visibilityFeatureChoiceChanged)
END_EVENT_TABLE()

}
}

namespace vulcan
{
namespace ui
{

enum HypothesesToShow
{
    kShowNoAssignments,
    kShowMaxLikelihood,
    kShowUnnormalized,
    kShowBoosting,
    kShowIsovist,
    kNumAssignments,
};


hssh::LocalTopoMode radio_selection_to_mode(int radio);
hssh::IsovistLocation radio_selection_to_location(int radio);
int int_from_text(wxTextCtrl* text);
double double_from_text(wxTextCtrl* text);


LocalTopoPanel::LocalTopoPanel(const ui_params_t& params, const local_topo_panel_widgets_t& widgets)
: widgets(widgets)
, hypothesisSelector_(widgets.displayWidget)
, cspPlayer_(new LabelingCSPPlayer)
, gatewaysAreDirty_(false)
, hypothesesAreDirty_(false)
, cspInfoIsDirty_(false)
, eventsAreDirty_(false)
, gatewaysIndex(0)
, hypFeatureIndex_(0)
, hoverHypothesis_(nullptr)
{
    assert(widgets.showVoronoiGridBox);
    assert(widgets.showDistanceGradientBox);
    assert(widgets.showFullSkeletonBox);
    assert(widgets.showReducedSkeletonBox);
    assert(widgets.showFrontiersBox);
    assert(widgets.centerOnRobotBox);
    assert(widgets.gatewayTypeChoice);
    assert(widgets.showAreaGraphBox);
    assert(widgets.showVisibilityGraphBox);
    assert(widgets.areaHypothesisValueRadio);
    assert(widgets.hypFeatureChoice);
    assert(widgets.hypothesesToShowRadio);
    assert(widgets.labelDistributionText);
    assert(widgets.cspIterationSlider);
    assert(widgets.cspSpeedSlider);
    assert(widgets.showHeatMapBox);
    assert(widgets.numPathsText);
    assert(widgets.eventsList);
    assert(widgets.showEventsVisualizationBox);
    assert(widgets.isovistLocationRadio);
    assert(widgets.isovistScalarChoice);
    assert(widgets.showIsovistBox);
    assert(widgets.showIsovistFieldBox);
    assert(widgets.showIsovistDerivFieldBox);
    assert(widgets.selectIsovistsButton);
    assert(widgets.showGradientsBox);
    assert(widgets.showLocalMaximaBox);
    assert(widgets.showGatewayProbabilitiesBox);
    assert(widgets.gatewayProbabilitySlider);
    assert(widgets.classifierToUseRadio);
    assert(widgets.visibilityFeatureChoice);

    widgets.displayWidget->setWidgetParams(params.localTopoParams);

    // Toggle all the show values to match the initialized UI
    widgets.displayWidget->showVoronoiGrid(widgets.showVoronoiGridBox->IsChecked());
    widgets.displayWidget->showDistanceGradient(widgets.showDistanceGradientBox->IsChecked());
    widgets.displayWidget->showFullSkeleton(widgets.showFullSkeletonBox->IsChecked());
    widgets.displayWidget->showReducedSkeleton(widgets.showReducedSkeletonBox->IsChecked());
    widgets.displayWidget->showFrontiers(widgets.showFrontiersBox->IsChecked());
    widgets.displayWidget->showAreaGraph(widgets.showAreaGraphBox->IsChecked());

    widgets.displayWidget->setHypothesisFeatureToShow(widgets.areaHypothesisValueRadio->GetSelection());

    widgets.displayWidget->showIsovist(widgets.showIsovistBox->IsChecked());
    widgets.displayWidget->showIsovistField(widgets.showIsovistFieldBox->IsChecked());
    widgets.displayWidget->showIsovistDerivField(widgets.showIsovistDerivFieldBox->IsChecked());
    widgets.displayWidget->showIsovistGradients(widgets.showGradientsBox->IsChecked());
    widgets.displayWidget->showGradientMaxima(widgets.showLocalMaximaBox->IsChecked());
    widgets.displayWidget->shouldFollowRobot(widgets.centerOnRobotBox->IsChecked());

    cspPlayer_->setFramesPerIteration(widgets.cspSpeedSlider->GetValue());
    widgets.displayWidget->setCSPPlayer(cspPlayer_.get());
    widgets.cspIterationSlider->SetValue(0);

    widgets.selectIsovistsButton->Enable(widgets.showIsovistBox->IsChecked());
    widgets.selectIsovistsButton->SetValue(false);

    widgets.displayWidget->useHeatMap(false);
    widgets.showHeatMapBox->SetValue(false);

    widgets.displayWidget->pushMouseHandler(&hypothesisSelector_);
    hypothesisSelector_.setHandler(this);

    // Create the names for the hypothesis features
    hssh::HypothesisFeatures hypFeatures;
    for(std::size_t n = 0; n < hypFeatures.numFeatures(); ++n)
    {
        widgets.hypFeatureChoice->Append(hssh::HypothesisFeatures::featureName(n));
    }

    widgets.hypFeatureChoice->SetSelection(0);

    // Create the names for the isovists
    for(int n = 0; n < utils::Isovist::kNumScalars; ++n)
    {
        widgets.isovistScalarChoice->Append(utils::Isovist::scalarName(static_cast<utils::Isovist::Scalar>(n)));
    }

    widgets.isovistScalarChoice->SetSelection(0);

    // Create the names for the visibility graph features
    for(int n = 0; n < static_cast<int>(utils::VisibilityGraphFeatureType::num_features); ++n)
    {
        widgets.visibilityFeatureChoice->Append(utils::feature_type_to_string(static_cast<utils::VisibilityGraphFeatureType>(n)));
    }

    widgets.visibilityFeatureChoice->SetSelection(0);

    gatewayProbCutoff_ = widgets.gatewayProbabilitySlider->GetValue() / 100.0;
}


LocalTopoPanel::~LocalTopoPanel(void)
{
    // For std::unique_ptr
}


void LocalTopoPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widgets.displayWidget->setRenderContext(context);
    widgets.displayWidget->setStatusBar(statusBar);
}


void LocalTopoPanel::subscribe(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<hssh::LocalPose>(widgets.displayWidget);
    producer.subscribeTo<hssh::local_area_debug_info_t>(widgets.displayWidget);

    producer.subscribeTo<hssh::VoronoiSkeletonGrid>(this);
    producer.subscribeTo<hssh::gateway_debug_info_t>(this);
    producer.subscribeTo<hssh::local_area_debug_info_t>(this);
    producer.subscribeTo<hssh::CSPDebugInfo>(this);
    producer.subscribeTo<hssh::LocalTopoMap>(this);
    producer.subscribeTo<hssh::LocalAreaEventVec>(this);
}


void LocalTopoPanel::setConsumer(system::ModuleCommunicator* consumer)
{
    consumer_ = consumer;
}


void LocalTopoPanel::update(void)
{
    utils::AutoMutex autoLock(dataLock_);

    widgets.displayWidget->Refresh();

    updateGateways();
    updateHypotheses();
    updateCSPInfo();
    updateEvents();
}


void LocalTopoPanel::saveSettings(utils::ConfigFileWriter& config)
{

}


void LocalTopoPanel::loadSettings(const utils::ConfigFile& config)
{

}


void LocalTopoPanel::handleData(const hssh::VoronoiSkeletonGrid& grid, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    this->grid.reset(new hssh::VoronoiSkeletonGrid(grid));
    widgets.displayWidget->setSkeleton(this->grid);
}


void LocalTopoPanel::handleData(const hssh::gateway_debug_info_t& debug, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    // If there aren't the same number of intermediate hypotheses, then clearly
    if(debug.intermediateHypotheses.size() != gatewayInfo.intermediateHypotheses.size())
    {
        gatewayInfo       = debug;
        gatewaysAreDirty_ = true;
        gatewaysIndex     = 0;
        return;
    }

    // Otherwise, see if the descriptions are all the same, if so, just copy over the hypotheses -- these should almost
    // always be the same
    for(std::size_t n = 0; n < debug.intermediateHypotheses.size(); ++n)
    {
        if(gatewayInfo.intermediateHypotheses[n].first == debug.intermediateHypotheses[n].first)
        {
            gatewayInfo.intermediateHypotheses[n].second = debug.intermediateHypotheses[n].second;
        }
        else
        {
            // A description has changed, so the choice box needs to be rebuilt
            gatewayInfo.intermediateHypotheses[n] = debug.intermediateHypotheses[n];
            gatewaysAreDirty_                     = true;
        }
    }

    if(gatewaysIndex < gatewayInfo.intermediateHypotheses.size())
    {
        widgets.displayWidget->setGateways(gatewayInfo.intermediateHypotheses[gatewaysIndex].second);
    }
}


void LocalTopoPanel::handleData(const hssh::local_area_debug_info_t& debug, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    hypothesisSelector_.clearObjects();
    hoverHypothesis_ = nullptr;

    widgets.displayWidget->setSelectedHypothesis(nullptr);

    hypothesesAreDirty_ = true;
    areaInfo_ = debug;
}


void LocalTopoPanel::handleData(const hssh::CSPDebugInfo& debug, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    cspInfo_ = debug;
    cspInfoIsDirty_ = true;
}


void LocalTopoPanel::handleData(const hssh::LocalTopoMap& map, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    topoMap_.reset(new hssh::LocalTopoMap(map));
    grid.reset(new hssh::VoronoiSkeletonGrid(map.voronoiSkeleton()));

    widgets.displayWidget->useHeatMap(false);
    widgets.displayWidget->setLocalTopoMap(topoMap_);
    widgets.displayWidget->setSkeleton(grid);
}


void LocalTopoPanel::handleData(const hssh::LocalAreaEventVec& events, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    if(events.empty())
    {
        return;
    }

    // If the arriving events have lower sequence numbers than the stored events, clear
    // out the current events because the local_topo_hssh module must have been reset
    if(!events_.empty() && events.front()->sequenceId() < events_.back()->sequenceId())
    {
        events_.clear();
    }

    std::transform(events.begin(),
                   events.end(),
                   std::back_inserter(events_),
                   [](const hssh::LocalAreaEventPtr& event)
                   {
                       return event->clone();
                   });

    eventsAreDirty_ = true;

    if(!events_.empty())
    {
        widgets.displayWidget->setEvent(events_.back());
    }
}


void LocalTopoPanel::objectEntered(const hssh::DebugHypothesis*& object)
{
    // Entering a new object always overrides the previous object
    hoverHypothesis_ = object;
    widgets.displayWidget->setSelectedHypothesis(hoverHypothesis_);
}


void LocalTopoPanel::objectExited(const hssh::DebugHypothesis*& object)
{
    // If the hover hypothesis is the same as the exited object, then the hover should be reset
    // If it is different, then some other event has changed it, so it should remain as-is
    if(hoverHypothesis_ == object)
    {
        hoverHypothesis_ = nullptr;
    }

    widgets.displayWidget->setSelectedHypothesis(hoverHypothesis_);
}


void LocalTopoPanel::objectSelected(const hssh::DebugHypothesis*& object)
{
    // No object selection happens in the LocalTopoPanel
}


void LocalTopoPanel::updateGateways(void)
{
    if(gatewaysAreDirty_)
    {
        widgets.gatewayTypeChoice->Clear();

        for(auto& hypotheses : gatewayInfo.intermediateHypotheses)
        {
            widgets.gatewayTypeChoice->Append(wxString(hypotheses.first.c_str(), wxConvUTF8));
        }

        gatewaysIndex     = (gatewaysIndex < gatewayInfo.intermediateHypotheses.size()) ? gatewaysIndex : 0;
        gatewaysAreDirty_ = false;
    }
}


void LocalTopoPanel::updateHypotheses(void)
{
    // Only go through the process of changing the visible hypotheses if the hypotheses have changed in some way
    if(hypothesesAreDirty_)
    {
        // Determine which hypotheses are currently being shown.
        switch(hypothesesToShow_)
        {
        case kShowMaxLikelihood:
            changeVisibleHypotheses(areaInfo_.maximumLikelihoodHypotheses);
            break;
        case kShowUnnormalized:
            changeVisibleHypotheses(areaInfo_.unnormalizedHypotheses);
            break;
        case kShowBoosting:
            changeVisibleHypotheses(areaInfo_.boostingHypotheses);
            break;
        case kShowIsovist:
            changeVisibleHypotheses(areaInfo_.isovistHypotheses);
            break;
        default:
            changeVisibleHypotheses(std::vector<hssh::DebugHypothesis>());
        }

        hypothesesAreDirty_ = false;
    }

    if(hoverHypothesis_)
    {
        auto distribution = hoverHypothesis_->getDistribution();
        widgets.labelDistributionText->SetLabel(wxString::Format(wxT("P:%.3f  Dec:%.3f  Dest:%.3f"),
                                                                 distribution.path,
                                                                 distribution.decision,
                                                                 distribution.destination));

        widgets.hypValueText->SetLabel(wxString::Format(wxT("%.3f"), hoverHypothesis_->featureValue(hypFeatureIndex_)));
    }
}


void LocalTopoPanel::changeVisibleHypotheses(const std::vector<hssh::DebugHypothesis>& visibleHypotheses)
{
    // Clear out any previously stored information about the hypotheses
    hoverHypothesis_ = nullptr;
    widgets.displayWidget->setSelectedHypothesis(nullptr);

    // Assign the hypotheses to be displayed
    widgets.displayWidget->setVisibleHypotheses(visibleHypotheses);

    if(grid)
    {
        // Assign the hypotheses to be selected amongst via the mouse
        std::map<Point<int>, const hssh::DebugHypothesis*> cellToVisible;
        for(auto& hyp : visibleHypotheses)
        {
            const auto& extent = hyp.getExtent();
            for(auto& cell : extent)
            {
                cellToVisible.insert(std::make_pair(utils::global_point_to_grid_cell_round(cell, *grid), &hyp));
            }
        }

        hypothesisSelector_.setObjects(cellToVisible);
    }
}


void LocalTopoPanel::updateCSPInfo(void)
{
    if(cspInfoIsDirty_ && grid)
    {
        cspPlayer_->setCSPInfo(cspInfo_, grid->metersPerCell());
        cspPlayer_->setFramesPerIteration(widgets.cspSpeedSlider->GetValue());
        widgets.cspIterationSlider->SetMin(0);
        widgets.cspIterationSlider->SetMax(cspPlayer_->numIterations()-1);
        cspInfoIsDirty_ = false;
    }

    widgets.cspIterationSlider->SetValue(cspPlayer_->currentIteration());
}


void LocalTopoPanel::updateEvents(void)
{
    if(eventsAreDirty_)
    {
        widgets.eventsList->Clear();

        for(auto& e : events_)
        {
            widgets.eventsList->Append(wxString(e->description().c_str(), wxConvUTF8));
        }

        eventsAreDirty_ = false;
    }
}


void LocalTopoPanel::calculateIsovistField(void)
{
    if(!grid)
    {
        return;
    }

    utils::isovist_options_t options;
    options.numRays       = 1440;
    options.maxDistance   = 20.0f;
    options.saveEndpoints = widgets.showIsovistBox->IsChecked();

    field_.reset(new hssh::VoronoiIsovistField(*grid, radio_selection_to_location(widgets.isovistLocationRadio->GetSelection()), options));
    widgets.displayWidget->setIsovistField(field_, static_cast<utils::Isovist::Scalar>(widgets.isovistScalarChoice->GetSelection()));
}


void LocalTopoPanel::calculateProbableGateways(void)
{
    if(!classifier_)
    {
        std::cerr << "ERROR: LocalTopoPanel: No gateway classifier loaded. Can't compute probabilities.\n";
        return;
    }
    else if(!grid)
    {
        std::cerr << "ERROR: LocalTopoPanel: No Voronoi skeleton available. Can't compute probabilities.\n";
        return;
    }
    else if(!field_)
    {
        std::cerr << "ERROR: LocalTopoPanel: No isovist field available. Can't compute probabilities.\n";
        return;
    }

    auto classifierType = static_cast<hssh::GatewayClassifier::ClassifierType>(
        widgets.classifierToUseRadio->GetSelection());

    hssh::VoronoiEdges edges{*grid, hssh::SKELETON_CELL_REDUCED_SKELETON};
    auto features = extract_gateway_features_default(edges, *field_);
    hssh::CellToTypeMap<double> probabilities;
    for(auto& f : features)
    {
        double probability = classifier_->classifyGateway(f.second, classifierType).probability;
        if((probability >= gatewayProbCutoff_) || (gatewayProbCutoff_ < 0.01))
        {
            probabilities[f.first] = probability;
        }
    }
    widgets.displayWidget->setGatewayProbabilities(probabilities);
}


void LocalTopoPanel::modeRadioChanged(wxCommandEvent& event)
{
    // Just fire off a message to local_topo_hssh
    auto cmd = std::make_shared<hssh::LocalTopoCommand>(radio_selection_to_mode(event.GetSelection()));
    consumer_->sendMessage(cmd);
}


void LocalTopoPanel::showVoronoiGrid(wxCommandEvent& event)
{
    widgets.displayWidget->showVoronoiGrid(event.IsChecked());
}


void LocalTopoPanel::showDistanceGradient(wxCommandEvent& event)
{
    widgets.displayWidget->showDistanceGradient(event.IsChecked());
}


void LocalTopoPanel::showFullSkeleton(wxCommandEvent& event)
{
    widgets.displayWidget->showFullSkeleton(event.IsChecked());
}


void LocalTopoPanel::showReducedSkeleton(wxCommandEvent& event)
{
    widgets.displayWidget->showReducedSkeleton(event.IsChecked());
}


void LocalTopoPanel::centerOnRobot(wxCommandEvent& event)
{
    widgets.displayWidget->shouldFollowRobot(event.IsChecked());
}


void LocalTopoPanel::showGateways(wxCommandEvent& event)
{
    widgets.displayWidget->showGateways(event.IsChecked());
}


void LocalTopoPanel::gatewayChoiceChanged(wxCommandEvent& event)
{
    std::size_t selectedIndex = event.GetSelection();
    if(selectedIndex < gatewayInfo.intermediateHypotheses.size())
    {
        gatewaysIndex = event.GetSelection();
        widgets.displayWidget->setGateways(gatewayInfo.intermediateHypotheses[gatewaysIndex].second);
    }
}


void LocalTopoPanel::showNormals(wxCommandEvent& event)
{
    widgets.displayWidget->showNormals(event.IsChecked());
}


void LocalTopoPanel::saveLocalTopoMapPressed(wxCommandEvent& event)
{
    if(!topoMap_)
    {
        std::cerr << "WARNING:LocalTopoPanel: No local topo map to save!\n";
        return;
    }

    wxFileDialog saveDialog(widgets.displayWidget,
                            wxT("Select map file..."),
                            wxT(""),
                            wxT(""),
                            wxT("*.ltm"),
                            kFileSaveFlags);

    if(saveDialog.ShowModal() == wxID_OK)
    {
        auto path = std::string{saveDialog.GetPath().mb_str()};
        if(!utils::save_serializable_to_file(path, *topoMap_))
        {
            std::cerr << "ERROR:LocalTopoPanel: Failed to save topo map to file " << path << '\n';
        }
    }
}


void LocalTopoPanel::loadLocalTopoMapPressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets.displayWidget,
                            wxT("Select map file..."),
                            wxT(""),
                            wxT(""),
                            wxT("*.ltm"),
                            kFileOpenFlags);

    if(loadDialog.ShowModal() == wxID_OK)
    {
        auto path = std::string{loadDialog.GetPath().mb_str()};
        hssh::LocalTopoMap topoMap;

        if(!utils::load_serializable_from_file(path, topoMap))
        {
            std::cerr << "ERROR:LocalTopoPanel: Failed to load topo map to file " << path << '\n';
        }
        else
        {
            topoMap_.reset(new hssh::LocalTopoMap(topoMap));
            widgets.displayWidget->setLocalTopoMap(topoMap_);
            grid = std::make_shared<hssh::VoronoiSkeletonGrid>(topoMap_->voronoiSkeleton());
            widgets.displayWidget->setSkeleton(grid);
        }
    }
}


void LocalTopoPanel::sendLocalTopoMapPressed(wxCommandEvent& event)
{
    if(topoMap_ && consumer_)
    {
        consumer_->sendMessage(*topoMap_);
    }
}


void LocalTopoPanel::showAreaGraph(wxCommandEvent& event)
{
    widgets.displayWidget->showAreaGraph(event.IsChecked());
}


void LocalTopoPanel::showVisibilityGraph(wxCommandEvent& event)
{
    widgets.displayWidget->showVisibilityGraph(event.IsChecked());
}


void LocalTopoPanel::areaHypothesisValueChanged(wxCommandEvent& event)
{
    widgets.displayWidget->setHypothesisValueToDisplay(event.GetSelection());
}


void LocalTopoPanel::hypFeatureChoiceChanged(wxCommandEvent& event)
{
    hypFeatureIndex_ = event.GetSelection();
    widgets.displayWidget->setHypothesisFeatureToShow(hypFeatureIndex_);
}


void LocalTopoPanel::hypothesesToShowChanged(wxCommandEvent& event)
{
    hypothesesToShow_ = event.GetSelection();
    hypothesesAreDirty_ = true;
}


void LocalTopoPanel::showFrontiers(wxCommandEvent& event)
{
    widgets.displayWidget->showFrontiers(event.IsChecked());
}


void LocalTopoPanel::showAreasChanged(wxCommandEvent& event)
{
    widgets.displayWidget->showAreas(event.GetSelection());
}


void LocalTopoPanel::showSmallScaleStar(wxCommandEvent& event)
{
    widgets.displayWidget->showSmallScaleStar(event.IsChecked());
}


void LocalTopoPanel::showAreaGateways(wxCommandEvent& event)
{
    widgets.displayWidget->showAreaGateways(event.IsChecked());
}


void LocalTopoPanel::cspLoadPressed(wxCommandEvent& event)
{
    hssh::CSPDebugInfo debug;
    std::string path = getenv("VULCAN_BIN");
    path += "/csp_debug_info.log";
    if(utils::load_serializable_from_file(path, debug))
    {
        handleData(debug, "HSSH_CSP_DEBUG_INFO");
    }
}


void LocalTopoPanel::cspPlayPressed(wxCommandEvent& event)
{
    cspPlayer_->play();
}


void LocalTopoPanel::cspPausePressed(wxCommandEvent& event)
{
    cspPlayer_->pause();
}

void LocalTopoPanel::cspStopPressed(wxCommandEvent& event)
{
    cspPlayer_->stop();
}


void LocalTopoPanel::cspJumpToStartPressed(wxCommandEvent& event)
{
    cspPlayer_->jumpToStart();
}


void LocalTopoPanel::cspPrevIterationPressed(wxCommandEvent& event)
{
    cspPlayer_->stepBackward();
}


void LocalTopoPanel::cspNextIterationPressed(wxCommandEvent& event)
{
    cspPlayer_->stepForward();
}


void LocalTopoPanel::cspJumpToEndPressed(wxCommandEvent& event)
{
    cspPlayer_->jumpToEnd();
}


void LocalTopoPanel::iterationSliderChanged(wxCommandEvent& event)
{
    cspPlayer_->jumpToIteration(event.GetSelection());
}


void LocalTopoPanel::speedSliderChanged(wxCommandEvent& event)
{
    cspPlayer_->setFramesPerIteration(widgets.cspSpeedSlider->GetValue());
}


void LocalTopoPanel::showHeatMapChanged(wxCommandEvent& event)
{
    widgets.displayWidget->useHeatMap(event.IsChecked());
}


void LocalTopoPanel::generateHeatMapPressed(wxCommandEvent& event)
{
    long numPaths = 0;
    if(widgets.numPathsText->GetValue().ToLong(&numPaths) && topoMap_)
    {
        numPaths = std::max(10l, numPaths);

        hssh::LocalTopoHeatMap heatMap(*topoMap_);
        heatMap.generatePaths(numPaths);
        widgets.displayWidget->setHeatMap(heatMap.labeledStats());

        if(widgets.showHeatMapBox->IsChecked())
        {
            widgets.displayWidget->useHeatMap(true);
        }
    }
}


void LocalTopoPanel::showEventVisualizations(wxCommandEvent& event)
{
    widgets.displayWidget->showEvents(event.IsChecked());
}


void LocalTopoPanel::showIsovistChanged(wxCommandEvent& event)
{
    widgets.displayWidget->showIsovist(event.IsChecked());
    widgets.selectIsovistsButton->Enable(event.IsChecked());
}


void LocalTopoPanel::showIsovistFieldChanged(wxCommandEvent& event)
{
    widgets.displayWidget->showIsovistField(event.IsChecked());
}


void LocalTopoPanel::showIsovistDerivFieldChanged(wxCommandEvent& event)
{
    widgets.displayWidget->showIsovistDerivField(event.IsChecked());
}


void LocalTopoPanel::calculateIsovistPressed(wxCommandEvent& event)
{
    calculateIsovistField();
}


void LocalTopoPanel::selectIsovistsChanged(wxCommandEvent& event)
{
    widgets.displayWidget->selectIsovists(event.IsChecked());
}


void LocalTopoPanel::scalarChoiceChanged(wxCommandEvent& event)
{
    if(field_)
    {
        widgets.displayWidget->setIsovistField(field_, static_cast<utils::Isovist::Scalar>(event.GetSelection()));
    }
}


void LocalTopoPanel::calculateGradientsPressed(wxCommandEvent& event)
{
    if(field_)
    {
        hssh::VoronoiEdges edges(*grid, hssh::SKELETON_CELL_REDUCED_SKELETON);
        gradients_.reset(new hssh::VoronoiIsovistGradients(edges));
        gradients_->calculateGradients(static_cast<utils::Isovist::Scalar>(widgets.isovistScalarChoice->GetSelection()),
                                       *field_);
        maxima_.reset(new hssh::VoronoiIsovistMaxima(*gradients_, edges, *grid));
        widgets.displayWidget->setIsovistGradients(gradients_, maxima_);
    }
}


void LocalTopoPanel::showGradientsChanged(wxCommandEvent& event)
{
    widgets.displayWidget->showIsovistGradients(event.IsChecked());
}


void LocalTopoPanel::showLocalMaximaChanged(wxCommandEvent& event)
{
    widgets.displayWidget->showGradientMaxima(event.IsChecked());
}


void LocalTopoPanel::loadGatewayClassifierPressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets.displayWidget,
                            wxT("Select base filename..."),
                            wxT(""),
                            wxT(""),
                            wxT(""),
                            kFileOpenFlags);
    if(loadDialog.ShowModal() == wxID_OK)
    {
        auto noExtension = loadDialog.GetPath().BeforeFirst(wxUniChar('.'));
        auto path = std::string{noExtension.mb_str()};

        classifier_ = std::make_unique<hssh::GatewayClassifier>(path);

        if(!classifier_)
        {
            std::cerr << "ERROR:LocalTopoPanel: Failed to load classifier " << path << '\n';
        }
    }
}


void LocalTopoPanel::calculateGatewayProbabilitiesPressed(wxCommandEvent& event)
{
    calculateProbableGateways();
}


void LocalTopoPanel::showGatewayProbabilitiesChanged(wxCommandEvent& event)
{
    widgets.displayWidget->showGatewayProbabilities(event.IsChecked());
}


void LocalTopoPanel::gatewayProbabilitySliderChanged(wxCommandEvent& event)
{
    gatewayProbCutoff_ = widgets.gatewayProbabilitySlider->GetValue() / 100.0;
    calculateProbableGateways();
}


void LocalTopoPanel::visibilityFeatureChoiceChanged(wxCommandEvent& event)
{
    widgets.displayWidget->setVisibilityGraphFeature(static_cast<utils::VisibilityGraphFeatureType>(event.GetSelection()));;
}


hssh::LocalTopoMode radio_selection_to_mode(int radio)
{
    switch(radio)
    {
    case 0:
        return hssh::LocalTopoMode::event_detection;
    case 1:
        return hssh::LocalTopoMode::label_only;
    }

    // If forget to update this, then bail out
    assert(false);
    return hssh::LocalTopoMode::event_detection;
}


hssh::IsovistLocation radio_selection_to_location(int radio)
{
    switch(radio)
    {
    case 0:
        return hssh::IsovistLocation::FREE_SPACE;
    case 1:
        return hssh::IsovistLocation::SKELETON;
    case 2:
        return hssh::IsovistLocation::REDUCED_SKELETON;
    default:
        assert(false);
    }

    return hssh::IsovistLocation::FREE_SPACE;
}


int int_from_text(wxTextCtrl* text)
{
    long value = 0;
    return text->GetValue().ToLong(&value) ? value : 0;
}


double double_from_text(wxTextCtrl* text)
{
    double value = 0.0;
    return text->GetValue().ToDouble(&value) ? value: 0.0;
}

} // namespace ui
} // namespace vulcan

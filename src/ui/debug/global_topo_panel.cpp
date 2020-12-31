/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     global_topo_panel.cpp
 * \author   Collin Johnson
 *
 * Definition of GlobalTopoPanel.
 */

#include "ui/debug/global_topo_panel.h"
#include "hssh/global_topological/commands/save_topo_slam_data.h"
#include "hssh/global_topological/commands/serialization.h"
#include "hssh/global_topological/debug/hypothesis_tree.h"
#include "hssh/global_topological/mapping/tree_of_maps.h"
#include "system/module_communicator.h"
#include "ui/debug/debug_ui.h"
#include "ui/debug/global_topo_display_widget.h"
#include "utils/serialized_file_io.h"
#include "utils/stub.h"
#include <algorithm>
#include <boost/range/iterator_range.hpp>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <wx/grid.h>

namespace vulcan
{
namespace ui
{

const std::string kTreeOfMapsFile("current_tree_of_maps.tom");
const std::string kMapCacheFile("current_map_cache.mmc");


BEGIN_EVENT_TABLE(GlobalTopoPanel, wxEvtHandler)
EVT_RADIOBOX(ID_GLOBAL_TOPO_MAP_VIEW_RADIO_BOX, GlobalTopoPanel::selectedMapView)
EVT_CHECKBOX(ID_SHOW_BEST_TOPO_MAP_CHECK_BOX, GlobalTopoPanel::showBestMap)
EVT_COMBOBOX(ID_TOPO_HYPOTHESIS_COMBO_BOX, GlobalTopoPanel::activeMapSelected)
EVT_BUTTON(ID_PREVIOUS_HYPOTHESIS_BUTTON, GlobalTopoPanel::previousMapPressed)
EVT_BUTTON(ID_NEXT_HYPOTHESIS_BUTTON, GlobalTopoPanel::nextMapPressed)
EVT_BUTTON(ID_USE_GLOBAL_TOPO_MAP_BUTTON, GlobalTopoPanel::useCurrentMapPressed)
EVT_BUTTON(ID_LOAD_GLOBAL_TOPO_MAP_FROM_FILE_BUTTON, GlobalTopoPanel::loadGlobalTopoMapPressed)
EVT_BUTTON(ID_SAVE_CURRENT_MAP_BUTTON, GlobalTopoPanel::saveGlobalTopoMapPressed)
EVT_BUTTON(ID_SAVE_TREE_BUTTON, GlobalTopoPanel::saveTreePressed)
EVT_BUTTON(ID_LOAD_TREE_BUTTON, GlobalTopoPanel::loadTreePressed)
EVT_BUTTON(ID_SAVE_MAP_CACHE_BUTTON, GlobalTopoPanel::saveMapCachePressed)
EVT_BUTTON(ID_LOAD_MAP_CACHE_BUTTON, GlobalTopoPanel::loadMapCachePressed)
EVT_BUTTON(ID_CLEAR_GLOBAL_TOPO_MAPS_BUTTON, GlobalTopoPanel::clearMapsPressed)
END_EVENT_TABLE()

const std::size_t MAX_HYPOTHESES_FOR_COMBO = 200;


std::vector<hssh::Id> sort_hypotheses_by_depth(const hssh::TreeOfMaps& treeOfMaps);


GlobalTopoPanel::GlobalTopoPanel(const ui_params_t& params, const global_topo_panel_widgets_t& widgets)
: widget(widgets.displayWidget)
, activeMapComboBox(widgets.activeMapComboBox)
, numHypothesesLabel(widgets.numHypothesesLabel)
, numCompleteLabel(widgets.numCompleteLabel)
, hypothesisInfoGrid(widgets.hypothesisInfoGrid)
, showingTree(false)
, numHypothesesInCombo(0)
, numActiveHypotheses(0)
, numCompleteHypotheses(0)
, hypothesesChanged(false)
, consumer(nullptr)
{
    assert(widget);
    assert(activeMapComboBox);
    assert(numHypothesesLabel);
    assert(numCompleteLabel);
    assert(hypothesisInfoGrid);

    widget->setWidgetParams(params.globalTopoParams, params.lpmParams);
}


void GlobalTopoPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widget->setStatusBar(statusBar);
    widget->setRenderContext(context);
}


void GlobalTopoPanel::subscribe(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<hssh::HypothesisTree>(this);

    //     producer.subscribeTo<hssh::TopologicalMap>(widget);
    //     producer.subscribeTo<hssh::TopoMapVec>(widget);
    producer.subscribeTo<hssh::TopologicalState>(widget);
    producer.subscribeTo<hssh::HypothesisTree>(widget);
    producer.subscribeTo<hssh::LocalAreaEventVec>(widget);
}


void GlobalTopoPanel::setConsumer(system::ModuleCommunicator* consumer)
{
    if (consumer) {
        this->consumer = consumer;
    }
}


void GlobalTopoPanel::update(void)
{
    if (hypothesesChanged) {
        activeMapComboBox->Clear();
        activeMapComboBox->Append(numberStrings);
        activeMapComboBox->SetSelection(0);

        // Change the label with the number of hypotheses
        numHypothesesLabel->SetLabel(wxString::Format(wxT("%zu"), numActiveHypotheses));
        numCompleteLabel->SetLabel(wxString::Format(wxT("%zu"), numCompleteHypotheses));

        hypothesesChanged = false;
    }

    widget->Refresh();

    //     PRINT_PRETTY_STUB()

    auto activeHypothesis = showingTree ? widget->getHoverMap() : widget->getDisplayedMap();
    hypothesisInfoGrid->SetCellValue(0, 0, wxString::Format(wxT("%li"), activeHypothesis.id));
    hypothesisInfoGrid->SetCellValue(1, 0, wxString::Format(wxT("%i"), activeHypothesis.depth));
    hypothesisInfoGrid->SetCellValue(2, 0, wxString::Format(wxT("%f"), activeHypothesis.probability.logPosterior));
    hypothesisInfoGrid->SetCellValue(3, 0, wxString::Format(wxT("%f"), activeHypothesis.probability.logLikelihood));
    hypothesisInfoGrid->SetCellValue(4, 0, wxString::Format(wxT("%f"), activeHypothesis.probability.logPrior));
    hypothesisInfoGrid->SetCellValue(5,
                                     0,
                                     wxString::Format(wxT("%f"), activeHypothesis.probability.estimatedLogLikelihood));
    hypothesisInfoGrid->SetCellValue(6, 0, wxString::Format(wxT("%f"), activeHypothesis.probability.estimatedLogPrior));
}


void GlobalTopoPanel::saveSettings(utils::ConfigFileWriter& config)
{
}


void GlobalTopoPanel::loadSettings(const utils::ConfigFile& config)
{
}


void GlobalTopoPanel::handleData(const hssh::HypothesisTree& tree, const std::string& channel)
{
    numActiveHypotheses = tree.numLeafNodes();
    numCompleteHypotheses = tree.numCompleteNodes();
    hypothesesChanged = true;
}


void GlobalTopoPanel::updateTreeOfMaps(void)
{
    // When new hypotheses arrive, set the number of hypotheses and then set the flag. Can't update here because
    // then UI updates are happening in a separate thread, which will cause the UI to occasionally crash
    assert(tree_);
    auto sortedIds = sort_hypotheses_by_depth(*tree_);
    numHypothesesInCombo = std::min(sortedIds.size(), MAX_HYPOTHESES_FOR_COMBO);

    numberStrings.Clear();
    numberStrings.Alloc(numHypothesesInCombo);
    for (size_t n = 0; n < numHypothesesInCombo; ++n) {
        numberStrings.Add(wxString::Format(wxT("%li"), sortedIds[n]));
    }

    hypothesesChanged = true;

    widget->setTreeOfMaps(tree_);
}


void GlobalTopoPanel::selectedMapView(wxCommandEvent& event)
{
    if (event.GetSelection() == 0) {
        widget->setActiveView(GlobalTopoDisplayWidget::GRAPH_VIEW);
    } else if (event.GetSelection() == 1) {
        widget->setActiveView(GlobalTopoDisplayWidget::PLACE_VIEW);
    } else if (event.GetSelection() == 2) {
        widget->setActiveView(GlobalTopoDisplayWidget::HYPOTHESIS_TREE);
    }

    showingTree = event.GetSelection() == 2;
}


void GlobalTopoPanel::showBestMap(wxCommandEvent& event)
{
    widget->showBestMap(event.IsChecked());
}


void GlobalTopoPanel::activeMapSelected(wxCommandEvent& event)
{
    // A new map was selected in the combo box
    wxString selected = activeMapComboBox->GetStringSelection();

    unsigned long selectedId = 0;
    if (selected.ToULong(&selectedId)) {
        widget->setIdToShow(selectedId);
    }
}


void GlobalTopoPanel::previousMapPressed(wxCommandEvent& event)
{
    int mapIndex = activeMapComboBox->GetSelection();

    if (mapIndex > 0) {
        activeMapComboBox->SetSelection(mapIndex - 1);
        activeMapSelected(event);
    }
}


void GlobalTopoPanel::nextMapPressed(wxCommandEvent& event)
{
    unsigned int mapIndex = activeMapComboBox->GetSelection();

    if (mapIndex < activeMapComboBox->GetCount() - 1) {
        activeMapComboBox->SetSelection(mapIndex + 1);
        activeMapSelected(event);
    }
}


void GlobalTopoPanel::useCurrentMapPressed(wxCommandEvent& event)
{
    assert(consumer && "ERROR: You forgot to set the output consumer for GlobalTopoPanel\n");

    wxString selected = activeMapComboBox->GetStringSelection();

    unsigned long selectedId = 0;
    if (selected.ToULong(&selectedId)) {
        PRINT_PRETTY_STUB()
    } else {
        std::cerr << "ERROR:GlobalTopoPanel:Non-numeric map id. Cannot send correct map message.\n";
    }
}


void GlobalTopoPanel::loadGlobalTopoMapPressed(wxCommandEvent& event)
{
    assert(consumer && "ERROR: You forgot to set the output consumer for GlobalTopoPanel\n");

    // TODO: Fix me
    wxFileDialog openDialog(widget);

    if (openDialog.ShowModal() == wxID_OK) {
        wxString path = openDialog.GetPath();
        PRINT_PRETTY_STUB()
    }
}


void GlobalTopoPanel::saveGlobalTopoMapPressed(wxCommandEvent& event)
{
    assert(consumer && "ERROR: You forgot to set the output consumer for GlobalTopoPanel\n");

    // TODO: Fix me
    wxFileDialog saveDialog(widget);

    if (saveDialog.ShowModal() == wxID_OK) {
        wxString path = saveDialog.GetPath();
        PRINT_PRETTY_STUB()
    }
}


void GlobalTopoPanel::saveTreePressed(wxCommandEvent& event)
{
    assert(consumer && "ERROR: You forgot to set the output consumer for GlobalTopoPanel\n");

    // Send a message out to the global_topo_hssh to save the TreeOfMaps
    hssh::GlobalTopoCommandPtr cmd =
      std::make_shared<hssh::SaveTopoSlamDataCommand>(hssh::TopoSlamDataType::tree_of_maps,
                                                      kTreeOfMapsFile,
                                                      "debug_ui");

    consumer->sendMessage(cmd);
}


void GlobalTopoPanel::loadTreePressed(wxCommandEvent& event)
{
    hssh::TreeOfMaps tree;
    bool success = utils::load_serializable_from_file(kTreeOfMapsFile, tree);

    if (success) {
        std::cout << "INFO: Successfully loaded TreeOfMaps from " << kTreeOfMapsFile << '\n';
        tree_ = std::make_shared<hssh::TreeOfMaps>(std::move(tree));
        updateTreeOfMaps();
    } else {
        std::cerr << "ERROR: Failed to load TreeOfMaps from " << kTreeOfMapsFile << '\n';
    }
}


void GlobalTopoPanel::saveMapCachePressed(wxCommandEvent& event)
{
    assert(consumer && "ERROR: You forgot to set the output consumer for GlobalTopoPanel\n");

    // Send a message out to the global_topo_hssh to save the MetricMapCache
    hssh::GlobalTopoCommandPtr cmd =
      std::make_shared<hssh::SaveTopoSlamDataCommand>(hssh::TopoSlamDataType::map_cache, kMapCacheFile, "debug_ui");

    consumer->sendMessage(cmd);
}


void GlobalTopoPanel::loadMapCachePressed(wxCommandEvent& event)
{
    hssh::MetricMapCache cache;
    bool success = utils::load_serializable_from_file(kMapCacheFile, cache);

    if (success) {
        std::cout << "INFO: Successfully loaded MetricMapCache from " << kMapCacheFile << '\n';
        widget->setMapCache(cache);
    } else {
        std::cerr << "ERROR: Failed to load MetricMapCache from " << kMapCacheFile << '\n';
    }
}


void GlobalTopoPanel::clearMapsPressed(wxCommandEvent& event)
{
    widget->clearMaps();
}


std::vector<hssh::Id> sort_hypotheses_by_depth(const hssh::TreeOfMaps& treeOfMaps)
{
    std::vector<std::tuple<int, double, hssh::Id>> leaves;

    for (auto& l : boost::make_iterator_range(treeOfMaps.beginLeaves(), treeOfMaps.endLeaves())) {
        leaves.emplace_back(l->visitDepth, l->probability.logPosterior, l->id);
    }

    std::sort(leaves.begin(), leaves.end(), [](const auto& lhs, const auto& rhs) {
        return (std::get<0>(lhs) > std::get<0>(rhs))
          || ((std::get<0>(lhs) == std::get<0>(rhs)) && (std::get<1>(lhs) > std::get<1>(rhs)));
    });

    std::vector<hssh::Id> sortedIds(leaves.size());
    std::transform(leaves.begin(), leaves.end(), sortedIds.begin(), [](const auto& v) {
        return std::get<2>(v);
    });

    return sortedIds;
}

}   // namespace ui
}   // namespace vulcan

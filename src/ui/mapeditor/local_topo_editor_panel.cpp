/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     local_topo_editor_panel.cpp
 * \author   Collin Johnson
 *
 * Implementation of LocalTopoEditorPanel.
 */

#include "ui/mapeditor/local_topo_editor_panel.h"
#include "hssh/local_metric/lpm.h"
#include "hssh/local_metric/lpm_io.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_classifier.h"
#include "hssh/local_topological/area_extent.h"
#include "hssh/local_topological/training/area_labels.h"
#include "hssh/local_topological/training/gateway_classifier_test.h"
#include "hssh/local_topological/training/labeled_area_data.h"
#include "hssh/local_topological/training/labeled_boundary_data.h"
#include "hssh/local_topological/training/local_topo_area_editor.h"
#include "system/module_communicator.h"
#include "ui/common/file_dialog_settings.h"
#include "ui/common/ui_params.h"
#include "ui/mapeditor/classification_test_results_dialog.h"
#include "ui/mapeditor/gateway_classifier_test_dialog.h"
#include "ui/mapeditor/local_topo_editor_widget.h"
#include "ui/mapeditor/map_editor.h"
#include "utils/serialized_file_io.h"
#include <cassert>
#include <iostream>


#include "hssh/local_topological/area_detection/gateways/isovist_gradients.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_edges.h"
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/skewness.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(LocalTopoEditorPanel, wxEvtHandler)
EVT_BUTTON(ID_LOAD_LOCAL_TOPO_FROM_FILE_BUTTON, LocalTopoEditorPanel::loadFromFilePressed)
EVT_BUTTON(ID_LOAD_LABELS_BUTTON, LocalTopoEditorPanel::loadLabelsPressed)
EVT_BUTTON(ID_SAVE_LABELS_BUTTON, LocalTopoEditorPanel::saveLabelsPressed)
EVT_BUTTON(ID_STORE_LABELS_BUTTON, LocalTopoEditorPanel::storeLabelsPressed)
EVT_RADIOBOX(ID_LOCAL_TOPO_EDIT_MODE_RADIO, LocalTopoEditorPanel::editModeChanged)
EVT_BUTTON(ID_AUTO_GENERATE_GATEWAYS_BUTTON, LocalTopoEditorPanel::generateGatewaysPressed)
EVT_TOGGLEBUTTON(ID_HAND_CREATE_GATEWAYS_BUTTON, LocalTopoEditorPanel::createGatewaysPressed)
EVT_BUTTON(ID_LOAD_GATEWAYS_BUTTON, LocalTopoEditorPanel::loadGatewaysPressed)
EVT_BUTTON(ID_CREATE_AREAS_FROM_GATEWAYS_BUTTON, LocalTopoEditorPanel::createAreasPressed)
EVT_TOGGLEBUTTON(ID_SELECT_LOCAL_AREAS_BUTTON, LocalTopoEditorPanel::selectAreasPressed)
EVT_BUTTON(ID_MERGE_SELECTED_AREAS_BUTTON, LocalTopoEditorPanel::mergeSelectedPressed)
EVT_BUTTON(ID_CLEAR_SELECTED_AREAS_BUTTON, LocalTopoEditorPanel::clearSelectedPressed)
EVT_BUTTON(ID_RESET_MERGED_AREAS_BUTTON, LocalTopoEditorPanel::resetMergedPressed)
EVT_RADIOBOX(ID_LABEL_TO_ASSIGN_RADIO, LocalTopoEditorPanel::labelToAssignChanged)
EVT_TOGGLEBUTTON(ID_ASSIGN_LABELS_BUTTON, LocalTopoEditorPanel::assignLabelsPressed)
EVT_BUTTON(ID_LABEL_ALL_AREAS_BUTTON, LocalTopoEditorPanel::labelAllAreasPressed)
EVT_BUTTON(ID_CLEAR_LOCAL_AREA_LABELS_BUTTON, LocalTopoEditorPanel::clearLabelsPressed)
EVT_BUTTON(ID_SIMPLIFY_VIA_LABELS_BUTTON, LocalTopoEditorPanel::simplifyViaLabelsPressed)
EVT_BUTTON(ID_TRAIN_GATEWAYS_BUTTON, LocalTopoEditorPanel::trainGatewaysPressed)
EVT_BUTTON(ID_TRAIN_AND_TEST_BUTTON, LocalTopoEditorPanel::trainAndTestPressed)
EVT_BUTTON(ID_ADD_TRAINING_DATA_BUTTON, LocalTopoEditorPanel::addTrainingDataPressed)
EVT_BUTTON(ID_REMOVE_TRAINING_DATA_BUTTON, LocalTopoEditorPanel::removeTrainingDataPressed)
EVT_BUTTON(ID_ADD_TEST_DATA_BUTTON, LocalTopoEditorPanel::addTestDataPressed)
EVT_BUTTON(ID_REMOVE_TEST_DATA_BUTTON, LocalTopoEditorPanel::removeTestDataPressed)
END_EVENT_TABLE()


// These need to be in the same order as the editModeRadio options.
enum
{
    kGatewayMode,
    kLabelMode,
    kMergeMode,
};

// These need to be in the same order as the labelToAssignRadio options.
enum
{
    kPathIndex = 0,
    kDecisionIndex,
    kDestIndex,
    kAreaIndex,
    kNumIndexes,
};

enum
{
    kMapNameColumn = 0,
    kPathColumn,
    kDecisionColumn,
    kDestColumn,
    kGatewayColumn,
    kNumColumns
};


void add_examples_to_labeled_data(const std::string& mapName,
                                  hssh::LocalTopoAreaEditor::HypothesisConstIter begin,
                                  hssh::LocalTopoAreaEditor::HypothesisConstIter end,
                                  hssh::LabeledAreaData& labeledData);

int hypothesis_type_to_int(hssh::HypothesisType type);
hssh::HypothesisType int_to_hypothesis_type(int type);

void brute_force_all_isovist_gradients(
  const std::map<std::string, std::shared_ptr<hssh::VoronoiSkeletonGrid>>& skeletons,
  const std::map<std::string, std::shared_ptr<hssh::VoronoiIsovistField>>& isovists_);


LocalTopoEditorPanel::LocalTopoEditorPanel(const local_topo_editor_panel_widgets_t& widgets,
                                           const ui_params_t& params,
                                           const hssh::local_topology_params_t& localTopoParams)
: widgets_(widgets)
, editor_(std::make_unique<hssh::LocalTopoAreaEditor>(localTopoParams))
, initialData_(std::make_unique<hssh::LabeledAreaData>())
, simplifiedData_(std::make_unique<hssh::LabeledAreaData>())
, boundaryData_(std::make_unique<hssh::LabeledBoundaryData>())
, gatewayData_(std::make_unique<hssh::LabeledGatewayData>())
, localTopoParams_(localTopoParams)
, areaSelector_(widgets.widget)
, gatewayEditor_(widgets.widget)
, mode_(widgets.editModeRadio->GetSelection())
, amSelecting_(false)
, label_(int_to_hypothesis_type(widgets.labelToAssignRadio->GetSelection()))
{
    assert(widgets_.widget);

    assert(widgets_.generateGatewaysButton);
    assert(widgets_.createGatewaysButton);
    assert(widgets_.loadGatewaysButton);
    assert(widgets_.createAreasButton);

    assert(widgets_.assignLabelsButton);
    assert(widgets_.editModeRadio);
    assert(widgets_.labelToAssignRadio);
    assert(widgets_.selectAreasButton);
    assert(widgets_.labelRemainingButton);

    assert(widgets_.mergedSelectedButton);
    assert(widgets_.clearSelectedButton);
    assert(widgets_.resetMergedButton);
    assert(widgets_.clearLabelsButton);
    assert(widgets_.simplifyViaLabelsButton);

    areaSelector_.setHandler(this);
    widgets_.widget->setParams(params);
    initializeLabeledDataList();

    changeMode(widgets_.editModeRadio->GetSelection());
}


LocalTopoEditorPanel::~LocalTopoEditorPanel(void)
{
    // For std::unique_ptr
}


void LocalTopoEditorPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widgets_.widget->setStatusBar(statusBar);
    widgets_.widget->setRenderContext(context);
}


void LocalTopoEditorPanel::subscribe(system::ModuleCommunicator& producer)
{
}


void LocalTopoEditorPanel::setConsumer(system::ModuleCommunicator* consumer)
{
    // All outputs are saved to disk, so nothing will be sent via LCM
}


void LocalTopoEditorPanel::update(void)
{
    widgets_.widget->setGateways(gatewayEditor_.constructedGateways());
    widgets_.widget->setHoverGateway(gatewayEditor_.hoverGateway());
    widgets_.widget->setSelectedGateway(gatewayEditor_.selectedGateway());
    widgets_.widget->Refresh();
}


void LocalTopoEditorPanel::saveSettings(utils::ConfigFileWriter& config)
{
    // No settings need to be saved. There aren't display options that could be usefully persisted across sessions.
}


void LocalTopoEditorPanel::loadSettings(const utils::ConfigFile& config)
{
    // No settings need to be loaded.
}


void LocalTopoEditorPanel::objectEntered(hssh::AreaHypothesis*& object)
{
    hoverType_ = object->getType();
    hoverArea_ = object;
    hoverArea_->setType(label_);
}


void LocalTopoEditorPanel::objectExited(hssh::AreaHypothesis*& object)
{
    // If there was a previous hover area and it wasn't selected, then it should have its type returned to the previous
    // value
    if (hoverArea_) {
        hoverArea_->setType(hoverType_);
    }

    hoverArea_ = nullptr;
}


void LocalTopoEditorPanel::objectSelected(hssh::AreaHypothesis*& object)
{
    selectedAreas_.insert(object);
    hoverType_ = label_;

    editor_->labelArea(object, label_);
}


void LocalTopoEditorPanel::changeLPM(const std::string& path, const std::string& mapName)
{
    // Clear out existing pointers into the areas before constructing a new set of areas
    areaSelector_.clearObjects();
    selectedAreas_.clear();
    widgets_.widget->setAreas(std::vector<hssh::AreaHypothesis*>());

    hssh::LocalPerceptualMap newLPM;
    hssh::load_lpm_1_0(path, newLPM);
    mapName_ = mapName;

    // The LPM needs to be switched and the areas need to be found again
    lpm_.reset(new hssh::LocalPerceptualMap(newLPM));
    auto skeleton = editor_->buildSkeleton(*lpm_);
    skeletons_[mapName_] = std::make_shared<hssh::VoronoiSkeletonGrid>(skeleton);
    isovists_[mapName_] = std::make_shared<hssh::VoronoiIsovistField>(editor_->isovistField());
    widgets_.widget->setSkeleton(skeleton);
    gatewayEditor_.setSkeleton(std::move(skeleton));   // skeleton isn't needed after this point, so can move it
    gatewayEditor_.clearGateways();
    widgets_.widget->setGateways(gatewayEditor_.constructedGateways());
    reloadAreas();
}


void LocalTopoEditorPanel::generateGatewaysForMap(void)
{
    auto currentGateways = gatewayEditor_.constructedGateways();
    auto gateways = editor_->findMoreGateways(currentGateways);
    gatewayEditor_.clearGateways();
    gatewayEditor_.addGateways(gateways);
}


void LocalTopoEditorPanel::createAreasForMap(void)
{
    editor_->constructHypotheses(gatewayEditor_.constructedGateways());
    reloadAreas();
}


void LocalTopoEditorPanel::reloadAreas(void)
{
    // The ShapeSelector and widget both need to reflect the new boundaries
    std::vector<hssh::AreaHypothesis*> areas(editor_->numHypotheses());
    std::copy(editor_->begin(), editor_->end(), areas.begin());

    widgets_.widget->setAreas(areas);

    // Assign the hypotheses to be selected amongst via the mouse
    std::map<Point<int>, hssh::AreaHypothesis*> cellToVisible;
    for (auto hyp : areas) {
        const auto& extent = hyp->extent();
        for (auto& cell : extent) {
            cellToVisible.insert(std::make_pair(utils::global_point_to_grid_cell_round(cell, *lpm_), hyp));
        }
    }

    areaSelector_.setObjects(cellToVisible);
}


void LocalTopoEditorPanel::clearSelection(void)
{
    // Set all areas back to being a generic label
    for (auto area : selectedAreas_) {
        area->setType(hssh::HypothesisType::kArea);
    }

    selectedAreas_.clear();
}


void LocalTopoEditorPanel::changeMode(int newMode)
{
    mode_ = newMode;
    toggleGatewayMode(mode_ == kGatewayMode);
    toggleMergeMode(mode_ == kMergeMode);
    toggleLabelMode(mode_ == kLabelMode);
}


void LocalTopoEditorPanel::toggleGatewayMode(bool enable)
{
    widgets_.generateGatewaysButton->Enable(enable);
    widgets_.createGatewaysButton->Enable(enable);
    widgets_.loadGatewaysButton->Enable(enable);
    widgets_.createAreasButton->Enable(enable);

    // Turn-off gateway editing when leaving gateway mode
    if (!enable) {
        widgets_.createGatewaysButton->SetValue(false);
        toggleGatewayEditing(false);
    }
}


void LocalTopoEditorPanel::toggleMergeMode(bool enable)
{
    widgets_.selectAreasButton->Enable(enable);
    widgets_.mergedSelectedButton->Enable(enable);
    widgets_.clearSelectedButton->Enable(enable);
    widgets_.resetMergedButton->Enable(enable);

    if (!enable) {
        clearSelection();
    }
}


void LocalTopoEditorPanel::toggleLabelMode(bool enable)
{
    widgets_.labelToAssignRadio->Enable(enable);
    widgets_.labelRemainingButton->Enable(enable);
    widgets_.assignLabelsButton->Enable(enable);
    widgets_.clearLabelsButton->Enable(enable);
    widgets_.simplifyViaLabelsButton->Enable(enable);
}


void LocalTopoEditorPanel::toggleGatewayEditing(bool enable)
{
    // Always remove the gateway editing, just in case
    widgets_.widget->removeMouseHandler(&gatewayEditor_);

    if (enable) {
        // Turn off shape selection for sure
        toggleShapeSelection(false);

        // Remove then add to make sure multiple copied don't get added by accident
        widgets_.widget->pushMouseHandler(&gatewayEditor_);
    }
}


void LocalTopoEditorPanel::toggleShapeSelection(bool enable)
{
    if (enable && !amSelecting_) {
        widgets_.widget->pushMouseHandler(&areaSelector_);
    } else if (!enable && amSelecting_) {
        widgets_.widget->removeMouseHandler(&areaSelector_);
    }

    if (enable) {
        toggleGatewayEditing(false);
    }

    amSelecting_ = enable;
}


void LocalTopoEditorPanel::addEditorAreasToLabeledData(void)
{
    add_examples_to_labeled_data(mapName_,
                                 editor_->beginInitialHypotheses(),
                                 editor_->endInitialHypotheses(),
                                 *initialData_);
    add_examples_to_labeled_data(mapName_, editor_->begin(), editor_->end(), *simplifiedData_);

    auto gatewayData = create_labeled_gateway_data(mapName_, gatewayEditor_.constructedGateways(), *editor_);
    gatewayData_->addExamples(gatewayData);

    auto boundaryData = create_labeled_boundary_data(mapName_, *editor_);
    boundaryData_->addExamples(boundaryData);
}


void LocalTopoEditorPanel::addSelectedDataToList(wxListBox* mapList)
{
    // Iterate through all elements in the labeled data and add all selected ones to the list of maps
    for (int n = 0; n < widgets_.labeledDataList->GetItemCount(); ++n) {
        if (widgets_.labeledDataList->GetItemState(n, wxLIST_STATE_SELECTED)) {
            // Add the map name to the list
            mapList->Append(widgets_.labeledDataList->GetItemText(n, kMapNameColumn));
            // Unselect the map from the list of selected maps
            widgets_.labeledDataList->SetItemState(n, 0, wxLIST_STATE_SELECTED);
        }
    }
}


void LocalTopoEditorPanel::removeSelectedDataFromList(wxListBox* mapList)
{
    // Iterate through all elements in the list
    for (unsigned int n = 0; n < mapList->GetCount();) {
        // If it is selected, then delete it
        if (mapList->IsSelected(n)) {
            mapList->Delete(n);
        }
        // Otherwise, move on to the next element
        else {
            ++n;
        }
    }
}


void LocalTopoEditorPanel::initializeLabeledDataList(void)
{
    // Same order as the enum above
    widgets_.labeledDataList
      ->InsertColumn(kMapNameColumn, "Map Name:", wxLIST_FORMAT_CENTER, wxLIST_AUTOSIZE_USEHEADER);
    widgets_.labeledDataList->InsertColumn(kPathColumn, "# Paths:", wxLIST_FORMAT_CENTER, wxLIST_AUTOSIZE_USEHEADER);
    widgets_.labeledDataList
      ->InsertColumn(kDecisionColumn, "# Decisions:", wxLIST_FORMAT_CENTER, wxLIST_AUTOSIZE_USEHEADER);
    widgets_.labeledDataList->InsertColumn(kDestColumn, "# Dests:", wxLIST_FORMAT_CENTER, wxLIST_AUTOSIZE_USEHEADER);
    widgets_.labeledDataList
      ->InsertColumn(kGatewayColumn, "# Gateways:", wxLIST_FORMAT_CENTER, wxLIST_AUTOSIZE_USEHEADER);
}


void LocalTopoEditorPanel::populateLabeledDataList(void)
{
    // Clear all data from the current list
    widgets_.labeledDataList->DeleteAllItems();
    widgets_.labeledDataList->Hide();

    // For each map in the labeled data
    int mapListRow = 0;
    for (auto& name : boost::make_iterator_range(simplifiedData_->beginMaps(), simplifiedData_->endMaps())) {
        // Count statistics on the number of each type of example
        int numDests = 0;
        int numDecisions = 0;
        int numPaths = 0;

        for (auto& example :
             boost::make_iterator_range(initialData_->beginMapExamples(name), initialData_->endMapExamples(name))) {
            switch (example.type) {
            case hssh::HypothesisType::kDecision:
                ++numDecisions;
                break;
            case hssh::HypothesisType::kDest:
                ++numDests;
                break;
            case hssh::HypothesisType::kPath:
                ++numPaths;
                break;
            default:
                // Don't count other types
                break;
            }
        }

        for (auto& example : boost::make_iterator_range(simplifiedData_->beginMapExamples(name),
                                                        simplifiedData_->endMapExamples(name))) {
            switch (example.type) {
            case hssh::HypothesisType::kDecision:
                ++numDecisions;
                break;
            case hssh::HypothesisType::kDest:
                ++numDests;
                break;
            case hssh::HypothesisType::kPath:
                ++numPaths;
                break;
            default:
                // Don't count other types
                break;
            }
        }

        int numGateways = 0;
        for (auto& example :
             boost::make_iterator_range(gatewayData_->beginMapExamples(name), gatewayData_->endMapExamples(name))) {
            if (example.isGateway) {
                ++numGateways;
            }
        }

        // Add a new item to the list with the map name and type counts
        long itemId = widgets_.labeledDataList->InsertItem(mapListRow, name);

        wxString columnFormat("%d");
        widgets_.labeledDataList->SetItem(itemId, kPathColumn, wxString::Format(columnFormat, numPaths));
        widgets_.labeledDataList->SetItem(itemId, kDecisionColumn, wxString::Format(columnFormat, numDecisions));
        widgets_.labeledDataList->SetItem(itemId, kDestColumn, wxString::Format(columnFormat, numDests));
        widgets_.labeledDataList->SetItem(itemId, kGatewayColumn, wxString::Format(columnFormat, numGateways));

        // Make sure the name column is wide enough to hold all names
        widgets_.labeledDataList->SetColumnWidth(kMapNameColumn, wxLIST_AUTOSIZE);

        ++mapListRow;
    }

    widgets_.labeledDataList->Show();
}


void LocalTopoEditorPanel::runClassificationTest(void)
{
    hssh::LabeledAreaData trainingDataInitial;
    hssh::LabeledAreaData trainingDataSimplified;
    hssh::LabeledAreaData trainingDataAll;
    hssh::LabeledAreaData testDataInitial;
    hssh::LabeledAreaData testDataSimplified;
    hssh::LabeledAreaData testDataAll;

    for (unsigned int n = 0; n < widgets_.trainingDataList->GetCount(); ++n) {
        std::string mapName = widgets_.trainingDataList->GetString(n).ToStdString();
        std::cout << "INFO:LocalTopoEditor: Adding to training data: " << mapName << '\n';
        trainingDataInitial.addExamples(initialData_->findMapExamples(mapName));
        trainingDataSimplified.addExamples(simplifiedData_->findMapExamples(mapName));
        trainingDataAll.addExamples(initialData_->findMapExamples(mapName));
        trainingDataAll.addExamples(simplifiedData_->findMapExamples(mapName));
    }

    for (unsigned int n = 0; n < widgets_.testDataList->GetCount(); ++n) {
        std::string mapName = widgets_.testDataList->GetString(n).ToStdString();
        std::cout << "INFO:LocalTopoEditor: Adding to test data: " << mapName << '\n';
        testDataInitial.addExamples(initialData_->findMapExamples(mapName));
        testDataSimplified.addExamples(simplifiedData_->findMapExamples(mapName));
        testDataAll.addExamples(initialData_->findMapExamples(mapName));
        testDataAll.addExamples(simplifiedData_->findMapExamples(mapName));
    }

    //     auto resultsInitialFull = new ClassificationTestResultsDialog(hssh::kFullPosteriorClassifierType,
    //                                                                   "initial",
    //                                                                   trainingDataInitial,
    //                                                                   testDataInitial,
    //                                                                   widgets_.widget);
    //     auto resultsInitialLikelihood = new ClassificationTestResultsDialog(hssh::kLikelihoodOnlyClassifierType,
    //                                                                         "initial",
    //                                                                         trainingDataInitial,
    //                                                                         testDataInitial,
    //                                                                         widgets_.widget);
    //     auto resultsSimplifiedFull = new ClassificationTestResultsDialog(hssh::kFullPosteriorClassifierType,
    //                                                                      "simplified",
    //                                                                      trainingDataSimplified,
    //                                                                      testDataSimplified,
    //                                                                      widgets_.widget);
    //     auto resultsSimplifiedLikelihood = new ClassificationTestResultsDialog(hssh::kLikelihoodOnlyClassifierType,
    //                                                                            "simplified",
    //                                                                            trainingDataSimplified,
    //                                                                            testDataSimplified,
    //                                                                            widgets_.widget);

    std::cout << "All data: Training:" << trainingDataAll.size() << " Test:" << testDataAll.size() << '\n';
    auto resultsAllLikelihood = new ClassificationTestResultsDialog(
      "",
      "all",
      trainingDataAll,
      //                                                                     trainingDataSimplified,
      testDataAll,
      widgets_.widget);

    //     resultsInitialFull->Show();
    //     resultsSimplifiedFull->Show();
    //     resultsInitialLikelihood->Show();
    //     resultsSimplifiedLikelihood->Show();
    resultsAllLikelihood->Show();
}


void LocalTopoEditorPanel::runGatewayTest(void)
{
    hssh::LabeledGatewayData trainingData;
    hssh::LabeledGatewayData testData;

    for (unsigned int n = 0; n < widgets_.trainingDataList->GetCount(); ++n) {
        std::string mapName = widgets_.trainingDataList->GetString(n).ToStdString();
        std::cout << "INFO:LocalTopoEditor: Adding to training data: " << mapName << '\n';
        trainingData.addExamples(gatewayData_->findMapExamples(mapName));
    }

    for (unsigned int n = 0; n < widgets_.testDataList->GetCount(); ++n) {
        std::string mapName = widgets_.testDataList->GetString(n).ToStdString();
        std::cout << "INFO:LocalTopoEditor: Adding to test data: " << mapName << '\n';
        testData.addExamples(gatewayData_->findMapExamples(mapName));
    }

    if (!trainingData.empty() && !testData.empty()) {
        hssh::GatewayClassifierTest test(localTopoParams_, trainingData, testData, skeletons_, isovists_);

        std::vector<GatewayClassifierTestResultsDialog*> dialogs;
        for (auto& result : test) {
            dialogs.push_back(new GatewayClassifierTestResultsDialog(result, widgets_.widget));
            dialogs.back()->Show();
        }
    }
}


void LocalTopoEditorPanel::loadFromFilePressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets_.widget, wxT("Select map file..."), wxT(""), wxT(""), wxT("*.lpm"), kFileOpenFlags);

    if (loadDialog.ShowModal() == wxID_OK) {
        wxString name = loadDialog.GetFilename().BeforeLast('.').ToStdString();

        mapName_ = name.ToStdString();
        mapDirectory_ = loadDialog.GetDirectory().ToStdString();

        changeLPM(loadDialog.GetPath().ToStdString(), mapName_);
        widgets_.mapNameText->SetLabel(name);
    }
}


void LocalTopoEditorPanel::loadLabelsPressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets_.widget,
                            wxT("Select labels file..."),
                            wxT(""),
                            wxT(""),
                            wxT("*.lbl"),
                            kFileOpenFlags);

    if (loadDialog.ShowModal() == wxID_OK) {
        wxString path = loadDialog.GetPath();
        auto allLabels = hssh::load_labels_file(path.ToStdString(), localTopoParams_);

        for (auto& building : allLabels.fullMaps) {
            for (auto& map : building.second) {
                if (map.gatewayData.empty()) {
                    std::cout << "Computing gateway features for " << map.name.toMapName() << '\n';
                    hssh::compute_gateway_features(map);
                }
                if (map.boundaryData.empty() || map.simplifiedData.empty()) {
                    std::cout << "Computing area features for " << map.name.toMapName() << '\n';
                    hssh::compute_area_and_boundary_features(map);
                }

                initialData_->addExamples(map.initialData);
                simplifiedData_->addExamples(map.simplifiedData);
                gatewayData_->addExamples(map.gatewayData);
                skeletons_[map.name.toMapName()] = map.skeleton;
                isovists_[map.name.toMapName()] = map.isovists;
                labeledMapNames_.emplace_back(map.name.toMapName(), map.directory);
            }
        }

        for (auto& building : allLabels.incrementalMaps) {
            for (auto& map : building.second) {
                initialData_->addExamples(map.initialData);
                simplifiedData_->addExamples(map.simplifiedData);
                gatewayData_->addExamples(map.gatewayData);
                skeletons_[map.name.toMapName()] = map.skeleton;
                isovists_[map.name.toMapName()] = map.isovists;
                labeledMapNames_.emplace_back(map.name.toMapName(), map.directory);
            }
        }

        populateLabeledDataList();

        //         brute_force_all_isovist_gradients(skeletons_, isovists_);
    }
}


void LocalTopoEditorPanel::saveLabelsPressed(wxCommandEvent& event)
{
    wxFileDialog saveDialog(widgets_.widget,
                            wxT("Select output file..."),
                            wxT(""),
                            wxT(""),
                            wxT("*.lbl"),
                            kFileSaveFlags);

    if (saveDialog.ShowModal() == wxID_OK) {
        if (labeledMapNames_.empty()) {
            std::cerr << "WARNING: Save Labels: No map data to save.\n";
        }

        wxString baseName = saveDialog.GetPath();

        // The labels file contains the location of the individually saved labels, which are saved whenever StoreLabels
        // is pressed
        std::ofstream out(baseName);
        for (auto& map : labeledMapNames_) {
            out << map.first << ' ' << map.second << '\n';
        }
    }
}


void LocalTopoEditorPanel::storeLabelsPressed(wxCommandEvent& event)
{
    if (lpm_ && editor_) {
        save_map_labels(*lpm_, *editor_, gatewayEditor_.constructedGateways(), mapName_, mapDirectory_);

        labeledMapNames_.push_back(std::make_pair(mapName_, mapDirectory_));

        addEditorAreasToLabeledData();
        populateLabeledDataList();
    }
}


void LocalTopoEditorPanel::editModeChanged(wxCommandEvent& event)
{
    changeMode(event.GetSelection());
}


void LocalTopoEditorPanel::generateGatewaysPressed(wxCommandEvent& event)
{
    generateGatewaysForMap();
}


void LocalTopoEditorPanel::createGatewaysPressed(wxCommandEvent& event)
{
    toggleGatewayEditing(event.IsChecked());
}


void LocalTopoEditorPanel::loadGatewaysPressed(wxCommandEvent& event)
{
    std::ostringstream gwyFile;
    gwyFile << mapDirectory_ << '/' << mapName_ << ".gwy";

    std::vector<hssh::Gateway> gateways;
    if (!utils::load_serializable_from_file(gwyFile.str(), gateways)) {
        std::cerr << "ERROR: Failed to load gateways from " << gwyFile.str() << '\n';
    } else {
        gatewayEditor_.addGateways(gateways);
    }
}


void LocalTopoEditorPanel::createAreasPressed(wxCommandEvent& event)
{
    createAreasForMap();
}


void LocalTopoEditorPanel::selectAreasPressed(wxCommandEvent& event)
{
    toggleShapeSelection(event.IsChecked());
}


void LocalTopoEditorPanel::mergeSelectedPressed(wxCommandEvent& event)
{
    if (selectedAreas_.empty()) {
        return;
    }

    std::vector<hssh::AreaHypothesis*> toMerge(selectedAreas_.begin(), selectedAreas_.end());
    editor_->mergeAreas(toMerge);
    clearSelection();
    reloadAreas();
}


void LocalTopoEditorPanel::clearSelectedPressed(wxCommandEvent& event)
{
    clearSelection();
}


void LocalTopoEditorPanel::resetMergedPressed(wxCommandEvent& event)
{
    editor_->resetAreas();
    clearSelection();
    reloadAreas();
}


void LocalTopoEditorPanel::labelToAssignChanged(wxCommandEvent& event)
{
    label_ = int_to_hypothesis_type(event.GetSelection());
}


void LocalTopoEditorPanel::assignLabelsPressed(wxCommandEvent& event)
{
    toggleShapeSelection(event.IsChecked());
}


void LocalTopoEditorPanel::labelAllAreasPressed(wxCommandEvent& event)
{
    auto numLabeled = editor_->labelRemainingAreas(label_);
    std::cout << "INFO:LocalTopoEditor: Applied label to " << numLabeled << " areas.\n";
}


void LocalTopoEditorPanel::clearLabelsPressed(wxCommandEvent& event)
{
    // Change all the labels back to area
    for (auto& area : *editor_) {
        editor_->labelArea(area, hssh::HypothesisType::kArea);
    }
}


void LocalTopoEditorPanel::simplifyViaLabelsPressed(wxCommandEvent& event)
{
    editor_->simplifyAreas();
    reloadAreas();
}


void LocalTopoEditorPanel::addTrainingDataPressed(wxCommandEvent& event)
{
    addSelectedDataToList(widgets_.trainingDataList);
}


void LocalTopoEditorPanel::removeTrainingDataPressed(wxCommandEvent& event)
{
    removeSelectedDataFromList(widgets_.trainingDataList);
}


void LocalTopoEditorPanel::addTestDataPressed(wxCommandEvent& event)
{
    addSelectedDataToList(widgets_.testDataList);
}


void LocalTopoEditorPanel::removeTestDataPressed(wxCommandEvent& event)
{
    removeSelectedDataFromList(widgets_.testDataList);
}


void LocalTopoEditorPanel::trainGatewaysPressed(wxCommandEvent& event)
{
    runGatewayTest();
}


void LocalTopoEditorPanel::trainAndTestPressed(wxCommandEvent& event)
{
    runClassificationTest();
}


void add_examples_to_labeled_data(const std::string& mapName,
                                  hssh::LocalTopoAreaEditor::HypothesisConstIter begin,
                                  hssh::LocalTopoAreaEditor::HypothesisConstIter end,
                                  hssh::LabeledAreaData& labeledData)
{
    std::vector<hssh::LabeledFeatures> examples;

    // Iterate through the hypotheses in the LocalTopoAreaEditor
    for (auto& hyp : boost::make_iterator_range(begin, end)) {
        // For each area, create the appropriate <type, features> data structure
        hssh::LabeledFeatures example;
        example.type = hyp->getType();
        example.features = hyp->features();
        examples.push_back(std::move(example));
    }

    // Add the examples to the labeled data using the current map name
    labeledData.addExamples(mapName, examples.begin(), examples.end());
}


int hypothesis_type_to_int(hssh::HypothesisType type)
{
    switch (type) {
    case hssh::HypothesisType::kDecision:
        return kDecisionIndex;

    case hssh::HypothesisType::kDest:
        return kDestIndex;

    case hssh::HypothesisType::kPath:
        return kPathIndex;

    default:
        return kAreaIndex;
    }
}


hssh::HypothesisType int_to_hypothesis_type(int type)
{
    switch (type) {
    case kDestIndex:
        return hssh::HypothesisType::kDest;

    case kDecisionIndex:
        return hssh::HypothesisType::kDecision;

    case kPathIndex:
        return hssh::HypothesisType::kPath;

    case kAreaIndex:
    default:
        return hssh::HypothesisType::kArea;
    }
}


void brute_force_all_isovist_gradients(
  const std::map<std::string, std::shared_ptr<hssh::VoronoiSkeletonGrid>>& skeletons,
  const std::map<std::string, std::shared_ptr<hssh::VoronoiIsovistField>>& isovists)
{
    using namespace boost::accumulators;

    using Acc = accumulator_set<double, stats<tag::mean, tag::lazy_variance, tag::skewness, tag::max, tag::min>>;

    std::cerr << "Performing brute-force search in the following maps:\n";
    std::vector<std::string> maps;
    for (auto& s : skeletons) {
        maps.push_back(s.first);
        std::cerr << maps.back() << '\n';
    }

    for (int n = 0; n < utils::Isovist::kNumScalars; ++n) {
        auto scalar = static_cast<utils::Isovist::Scalar>(n);

        std::cerr << "Brute-forcing " << utils::Isovist::scalarName(scalar) << "...\n";

        Acc acc;

        for (auto& m : maps) {
            hssh::VoronoiEdges edges(*skeletons.at(m), hssh::SKELETON_CELL_REDUCED_SKELETON);
            hssh::VoronoiIsovistGradients gradients(edges);
            gradients.calculateGradients(scalar, *isovists.at(m));

            for (auto& g : gradients) {
                acc(std::abs(g.value));
            }
        }

        std::cout << utils::Isovist::scalarName(scalar) << ":"
                  << "\n\tMin:  " << min(acc) << "\n\tMax:  " << max(acc) << "\n\tMean: " << mean(acc)
                  << "\n\tVar:  " << variance(acc) << "\n";
    }
}

}   // namespace ui
}   // namespace vulcan

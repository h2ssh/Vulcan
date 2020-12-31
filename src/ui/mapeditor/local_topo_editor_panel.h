/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_editor_panel.h
* \author   Collin Johnson
*
* Definition of LocalTopoEditorPanel.
*/

#ifndef UI_MAPEDITOR_LOCAL_TOPO_EDITOR_PANEL_H
#define UI_MAPEDITOR_LOCAL_TOPO_EDITOR_PANEL_H

#include "ui/mapeditor/gateway_editor.h"
#include "ui/common/ui_panel.h"
#include "ui/common/grid_object_selector.h"
#include "ui/common/object_selector_event_handler.h"
#include "ui/common/ui_color.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis.h"
#include "hssh/local_topological/params.h"
#include <wx/wx.h>
#include <wx/listctrl.h>
#include <wx/tglbtn.h>
#include <atomic>
#include <map>
#include <memory>
#include <set>
#include <vector>

namespace vulcan
{
namespace hssh { class LocalPerceptualMap; }
namespace hssh { class LocalTopoAreaEditor; }
namespace hssh { class LabeledAreaData; }
namespace hssh { class LabeledBoundaryData; }
namespace hssh { class LabeledGatewayData; }
namespace hssh { class VoronoiIsovistField; }
namespace hssh { class VoronoiSkeletonGrid; }
namespace ui
{

class  LocalTopoEditorWidget;
struct ui_params_t;    

struct local_topo_editor_panel_widgets_t
{
    LocalTopoEditorWidget* widget = nullptr;
    wxStaticText*          mapNameText = nullptr;
    wxRadioBox*            editModeRadio = nullptr;
    wxButton*              generateGatewaysButton = nullptr;
    wxToggleButton*        createGatewaysButton = nullptr;
    wxButton*              loadGatewaysButton = nullptr;
    wxButton*              createAreasButton = nullptr;
    wxToggleButton*        selectAreasButton = nullptr;
    wxButton*              mergedSelectedButton = nullptr;
    wxButton*              clearSelectedButton = nullptr;
    wxButton*              resetMergedButton = nullptr;
    wxRadioBox*            labelToAssignRadio = nullptr;
    wxToggleButton*        assignLabelsButton = nullptr;
    wxButton*              labelRemainingButton = nullptr;
    wxButton*              clearLabelsButton = nullptr;
    wxButton*              simplifyViaLabelsButton = nullptr;
    wxListCtrl*            labeledDataList = nullptr;
    wxListBox*             trainingDataList = nullptr;
    wxListBox*             testDataList = nullptr;
};

/**
* LocalTopoEditorPanel
*/
class LocalTopoEditorPanel : public UIPanel ,
                             public ObjectSelectorEventHandler<hssh::AreaHypothesis*>
{
public:
    
    /**
    * Constructor for LocalTopoEditorPanel.
    */
    LocalTopoEditorPanel(const local_topo_editor_panel_widgets_t& widgets,
                         const ui_params_t&                       params,
                         const hssh::local_topology_params_t&     localTopoParams);
    
    /**
    * Destructor for LocalTopoEditorPanel.
    */
    virtual ~LocalTopoEditorPanel(void);
    
    // UIPanel interface
    void setup(wxGLContext* context, wxStatusBar* statusBar) override;
    void subscribe(system::ModuleCommunicator& producer) override;
    void setConsumer(system::ModuleCommunicator* consumer) override;
    void update(void) override;
    void saveSettings(utils::ConfigFileWriter& config) override;
    void loadSettings(const utils::ConfigFile& config) override;

    // ObjectSelectorEventHandler interface
    void objectEntered(hssh::AreaHypothesis*& object) override;
    void objectExited(hssh::AreaHypothesis*& object) override;
    void objectSelected(hssh::AreaHypothesis*& object) override;

private:

    local_topo_editor_panel_widgets_t widgets_;
    
    std::shared_ptr<hssh::LocalPerceptualMap> lpm_;
    std::unique_ptr<hssh::LocalTopoAreaEditor> editor_;
    std::unique_ptr<hssh::LabeledAreaData> initialData_;
    std::unique_ptr<hssh::LabeledAreaData> simplifiedData_;
    std::unique_ptr<hssh::LabeledBoundaryData> boundaryData_;
    std::unique_ptr<hssh::LabeledGatewayData> gatewayData_;
    std::map<std::string, std::shared_ptr<hssh::VoronoiSkeletonGrid>> skeletons_;
    std::map<std::string, std::shared_ptr<hssh::VoronoiIsovistField>> isovists_;

    hssh::local_topology_params_t localTopoParams_;

    GridObjectSelector<hssh::AreaHypothesis*> areaSelector_;
    std::set<hssh::AreaHypothesis*> selectedAreas_;
    
    hssh::AreaHypothesis* hoverArea_;
    hssh::HypothesisType hoverType_;
    
    GatewayEditor gatewayEditor_;

    std::string mapName_;
    std::string mapDirectory_;

    std::vector<std::pair<std::string, std::string>> labeledMapNames_;
    
    int  mode_;
    bool amSelecting_;
    hssh::HypothesisType label_;
    
    void changeLPM(const std::string& path, const std::string& mapName);
    void generateGatewaysForMap(void);
    void createAreasForMap(void);
    void reloadAreas(void);
    void clearSelection(void);
    void changeMode(int newMode);
    void toggleGatewayMode(bool enable);
    void toggleMergeMode(bool enable);
    void toggleLabelMode(bool enable);
    void toggleGatewayEditing(bool enable);
    void toggleShapeSelection(bool enable);

    void addEditorAreasToLabeledData(void);
    void addSelectedDataToList(wxListBox* mapList);
    void removeSelectedDataFromList(wxListBox* mapList);
    void initializeLabeledDataList(void);
    void populateLabeledDataList(void);

    void runClassificationTest(void);
    void runGatewayTest(void);
    
    // Event handlers
    void loadFromFilePressed(wxCommandEvent& event);
    void loadLabelsPressed(wxCommandEvent& event);
    void saveLabelsPressed(wxCommandEvent& event);
    void storeLabelsPressed(wxCommandEvent& event);
    void editModeChanged(wxCommandEvent& event);
    void generateGatewaysPressed(wxCommandEvent& event);
    void createGatewaysPressed(wxCommandEvent& event);
    void loadGatewaysPressed(wxCommandEvent& event);
    void createAreasPressed(wxCommandEvent& event);
    void selectAreasPressed(wxCommandEvent& event);
    void mergeSelectedPressed(wxCommandEvent& event);
    void clearSelectedPressed(wxCommandEvent& event);
    void resetMergedPressed(wxCommandEvent& event);
    void labelToAssignChanged(wxCommandEvent& event);
    void assignLabelsPressed(wxCommandEvent& event);
    void labelAllAreasPressed(wxCommandEvent& event);
    void clearLabelsPressed(wxCommandEvent& event);
    void simplifyViaLabelsPressed(wxCommandEvent& event);
    void addTrainingDataPressed(wxCommandEvent& event);
    void removeTrainingDataPressed(wxCommandEvent& event);
    void addTestDataPressed(wxCommandEvent& event);
    void removeTestDataPressed(wxCommandEvent& event);
    void trainGatewaysPressed(wxCommandEvent& event);
    void trainAndTestPressed(wxCommandEvent& event);
    
    DECLARE_EVENT_TABLE()
};

} // namespace ui
} // namespace vulcan

#endif // UI_MAPEDITOR_LOCAL_TOPO_EDITOR_PANEL_H

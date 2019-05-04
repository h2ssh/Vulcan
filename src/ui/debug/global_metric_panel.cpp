/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_metric_panel.cpp
* \author   Collin Johnson
* 
* Definition of GlobalMetricPanel.
*/

#include <ui/debug/global_metric_panel.h>
#include <ui/debug/debug_ui.h>
#include <ui/debug/global_metric_display_widget.h>
#include <hssh/global_metric/messages.h>
#include <hssh/global_metric/debug_info.h>
#include <hssh/local_metric/lpm.h>
#include <hssh/local_metric/lpm_io.h>
#include <hssh/metrical/relocalization/filter_initializer_impl.h>
#include <hssh/metrical/relocalization/scan_matching_initializer.h>
#include <system/module_communicator.h>
#include <wx/filedlg.h>
#include <cassert>

namespace vulcan
{
namespace ui
{
    
BEGIN_EVENT_TABLE(GlobalMetricPanel, wxEvtHandler)
    EVT_BUTTON(ID_GLOBAL_METRIC_CAPTURE_LPM_BUTTON,  GlobalMetricPanel::captureLPMPressed)
    EVT_BUTTON(ID_LOAD_LPM_FOR_GLOBAL_METRIC_BUTTON, GlobalMetricPanel::loadLPMPressed)
    EVT_BUTTON(ID_LOAD_GLOBAL_METRIC_MAP_BUTTON,     GlobalMetricPanel::loadGMMPressed)
    EVT_BUTTON(ID_SAVE_GLOBAL_MAP_BUTTON,            GlobalMetricPanel::saveGMMPressed)
    EVT_BUTTON(ID_GLOBAL_METRIC_RELOCALIZE_BUTTON,   GlobalMetricPanel::relocalizePressed)
END_EVENT_TABLE()


GlobalMetricPanel::GlobalMetricPanel(const global_metric_panel_widgets_t& widgets, const ui_params_t& params)
: widgets_(widgets)
, shouldSaveLPM_(false)
, haveMap_(false)
{
    assert(widgets_.display);
    assert(widgets_.mapNameText);
    assert(widgets_.relocalizeButton);
    assert(widgets_.saveMapButton);
    
    widgets_.saveMapButton->Disable();
    widgets_.relocalizeButton->Disable();
}


void GlobalMetricPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widgets_.display->setRenderContext(context);
    widgets_.display->setStatusBar(statusBar);
}


void GlobalMetricPanel::subscribe(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<hssh::GlobalMetricMap>                    (this);
    producer.subscribeTo<hssh::GlobalPose>                         (this);
    producer.subscribeTo<hssh::LocalPerceptualMap>                 (this);
    producer.subscribeTo<hssh::global_metric_localization_info_t>  (this);
    producer.subscribeTo<hssh::global_metric_relocalization_info_t>(this);
}


void GlobalMetricPanel::setConsumer(system::ModuleCommunicator* consumer)
{
    consumer_ = consumer;
}


void GlobalMetricPanel::update(void)
{
    widgets_.display->Refresh();
}


void GlobalMetricPanel::saveSettings(utils::ConfigFileWriter& config)
{
    // TODO
}


void GlobalMetricPanel::loadSettings(const utils::ConfigFile& config)
{
    // TODO
}


void GlobalMetricPanel::handleData(const hssh::GlobalMetricMap& map, const std::string& channel)
{
    map_.reset(new hssh::GlobalMetricMap(map));
    widgets_.display->setMap(map_);
}


void GlobalMetricPanel::handleData(const hssh::GlobalPose& pose, const std::string& channel)
{
    pose_ = pose;
    widgets_.display->setPose(pose);
}


void GlobalMetricPanel::handleData(const hssh::LocalPerceptualMap& map, const std::string& channel)
{
    if(shouldSaveLPM_)
    {
        createGlobalMapFromLPM(map);
        shouldSaveLPM_ = false;
    }
}


void GlobalMetricPanel::handleData(const hssh::global_metric_relocalization_info_t& info, const std::string& channel)
{
    widgets_.display->setRelocalizationInfo(info.info);
}


void GlobalMetricPanel::handleData(const hssh::global_metric_localization_info_t& info, const std::string& channel)
{
    widgets_.display->setLocalizationInfo(info.info);
}


void GlobalMetricPanel::loadMap(const std::string& filename)
{
    map_.reset(new hssh::GlobalMetricMap());
    
    if(!map_->loadFromFile(filename))
    {
        std::cerr << "ERROR: GlobalMetricPanel: loadMap failed for file: " << filename << '\n';
        return;
    }
    
    haveMap_ = true;
    updateUIWithNewMap();
}


void GlobalMetricPanel::updateUIWithNewMap(void)
{
    assert(haveMap_);
    
    // Now that the map is loaded, fill in the text control and turn on the relocalize and save buttons
    widgets_.display->setMap(map_);
    widgets_.mapNameText->SetValue(map_->name());
    widgets_.relocalizeButton->Enable();
    widgets_.saveMapButton->Enable();
}


void GlobalMetricPanel::saveMap(const std::string& filename)
{
    assert(map_);
    
    // Ensure the saved map name matches the user-specified name of the map
    auto textName = widgets_.mapNameText->GetValue();
    
    hssh::GlobalMetricMap toSave(std::string(textName.mb_str()), *map_);
    
    if(!toSave.saveToFile(filename))
    {
        std::cerr << "ERROR: GlobalMetricPanel: saveMap failed for file: " << filename << '\n';
    }
}


void GlobalMetricPanel::startRelocalization(void)
{
    assert(map_);
    
    hssh::global_metric_relocalization_request_message_t request;
    request.map         = *map_;
//     request.initializer = std::make_shared<hssh::FreeSpaceFilterInitializer>(3, 10);
    request.initializer = std::make_shared<hssh::ScanMatchingInitializer>();
    consumer_->sendMessage(request);
}


void GlobalMetricPanel::createGlobalMapFromLPM(const hssh::LocalPerceptualMap& lpm)
{
    wxString mapName = widgets_.mapNameText->GetValue();
    map_.reset(new hssh::GlobalMetricMap(std::string(mapName.mb_str()), lpm));
    haveMap_ = true;
    updateUIWithNewMap();
}


void GlobalMetricPanel::captureLPMPressed(wxCommandEvent& event)
{
    shouldSaveLPM_ = true;
}


void GlobalMetricPanel::loadLPMPressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets_.display,
                            ("Load LPM..."),
                            "",     // default directory
                            "",     // default filename
                            "Local Perceptual Map (*.lpm)|*.lpm",
                            wxFD_OPEN);
    
    if(loadDialog.ShowModal() == wxID_OK)
    {
        wxString path = loadDialog.GetPath();
        
        hssh::LocalPerceptualMap lpm;
        hssh::load_lpm_1_0(std::string(path.mb_str()), lpm);
        
        createGlobalMapFromLPM(lpm);
    }
}


void GlobalMetricPanel::loadGMMPressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets_.display,
                            ("Load GMM..."),
                            "",     // default directory
                            "",     // default filename
                            "Global Metric Map (*.gmm)|*.gmm",
                            wxFD_OPEN);
    
    if(loadDialog.ShowModal() == wxID_OK)
    {
        wxString path = loadDialog.GetPath();
        loadMap(std::string(path.mb_str()));
    }
}


void GlobalMetricPanel::saveGMMPressed(wxCommandEvent& event)
{
    wxFileDialog saveDialog(widgets_.display,
                            ("Save Global Metric Map..."),
                            "",     // default directory
                            "",     // default filename
                            "Global Metric Map (*.gmm)|*.gmm",
                            wxFD_SAVE|wxFD_OVERWRITE_PROMPT);
    
    if((saveDialog.ShowModal() == wxID_OK) && map_)
    {
        wxString path = saveDialog.GetPath();
        saveMap(std::string(path.mb_str()));
    }
}


void GlobalMetricPanel::relocalizePressed(wxCommandEvent& event)
{
    assert(haveMap_);
    startRelocalization();
}
    
} // namespace ui
} // namespace vulcan

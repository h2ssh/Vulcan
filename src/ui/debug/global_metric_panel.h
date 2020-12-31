/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UI_DEBUG_GLOBAL_METRIC_PANEL_H
#define UI_DEBUG_GLOBAL_METRIC_PANEL_H

#include "ui/common/ui_panel.h"
#include "hssh/global_metric/pose.h"
#include <wx/wx.h>

namespace vulcan
{
namespace hssh { class GlobalMetricMap;    }
namespace hssh { class LocalPerceptualMap; }
namespace hssh { struct global_metric_relocalization_info_t; }
namespace hssh { struct global_metric_localization_info_t;   }
namespace ui
{

class  GlobalMetricDisplayWidget;
struct ui_params_t;


/**
* global_metric_panel_widgets_t contains the widgets controlled by GlobalMetricPanel.
*/
struct global_metric_panel_widgets_t
{
    GlobalMetricDisplayWidget* display;
    
    wxTextCtrl* mapNameText;
    wxButton*   saveMapButton;
    wxButton*   relocalizeButton;
};
  
/**
* GlobalMetricPanel controls the I/O for the Global Metric tab in the DebugUI. The GlobalMetricPanel performs the following tasks for the display of debugging information:
* 
*   1) A GlobalMetricMap is created by loading an existing GlobalMetricMap or by setting a non-empty name
*      in the Map Name text and then acquiring an LPM in one of two ways:
*       - Capturing an LPM from LCM and turning it into into a GlobalMetricMap
*       - Loading an LPM from a file and converting it into a GlobalMetricMap
* 
*   2) If a map is created, then the robot can relocalize in it via the Relocalize button.
*   3) If a map exists, then it can be saved to a file via the Save Global Map button.
*/
class GlobalMetricPanel : public UIPanel
{
public:
    
    /**
    * Constructor for GlobalMetricPanel.
    */
    GlobalMetricPanel(const global_metric_panel_widgets_t& widgets, const ui_params_t& params);
    
    
    // UIPanel interface
    virtual void setup       (wxGLContext* context, wxStatusBar* statusBar) override;
    virtual void subscribe   (system::ModuleCommunicator& producer)         override;
    virtual void setConsumer (system::ModuleCommunicator* consumer)         override;
    virtual void update      (void)                                         override;
    virtual void saveSettings(utils::ConfigFileWriter& config)              override;
    virtual void loadSettings(const utils::ConfigFile& config)              override;
    
    // Handlers for incoming data
    void handleData(const hssh::GlobalMetricMap&    map,  const std::string& channel);
    void handleData(const hssh::GlobalPose&         pose, const std::string& channel);
    void handleData(const hssh::LocalPerceptualMap& map,  const std::string& channel);
    
    void handleData(const hssh::global_metric_relocalization_info_t& info, const std::string& channel);
    void handleData(const hssh::global_metric_localization_info_t&   info, const std::string& channel);
    
private:
    
    global_metric_panel_widgets_t widgets_;
    system::ModuleCommunicator*   consumer_;
    
    std::shared_ptr<hssh::GlobalMetricMap> map_;
    hssh::GlobalPose                       pose_;
    
    bool shouldSaveLPM_;    // Flag indicating if the next LPM received from LCM should be saved
    bool haveMap_;          // Flag indicating if a global metric map exists
    
    void loadMap               (const std::string& filename);
    void saveMap               (const std::string& filename);
    void createGlobalMapFromLPM(const hssh::LocalPerceptualMap& lpm);
    void startRelocalization   (void);
    void updateUIWithNewMap    (void);
    
    // Event handlers
    void captureLPMPressed(wxCommandEvent& event);
    void loadLPMPressed   (wxCommandEvent& event);
    void loadGMMPressed   (wxCommandEvent& event);
    void saveGMMPressed   (wxCommandEvent& event);
    void relocalizePressed(wxCommandEvent& event);
    
    DECLARE_EVENT_TABLE()
};
    
}
}

#endif // UI_DEBUG_GLOBAL_METRIC_PANEL_H

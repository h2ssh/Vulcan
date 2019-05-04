/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     relocalization_panel.h
* \author   Collin Johnson
* 
* Declaration of RelocalizationPanel.
*/

#ifndef UI_DEBUG_RELOCALIZATION_PANEL_H
#define UI_DEBUG_RELOCALIZATION_PANEL_H

#include <wx/wx.h>
#include <ui/common/ui_forward_declarations.h>
#include <ui/common/ui_panel.h>
#include <math/geometry/rectangle.h>
#include <memory>

namespace vulcan
{
namespace hssh { struct local_metric_relocalization_debug_info_t; }
namespace hssh { struct global_metric_relocalization_info_t;      }
namespace hssh { class  FilterInitializer;                        }
namespace hssh { class  LocalPerceptualMap;                       }
namespace hssh { class  GlobalMetricMap;                          }
namespace ui
{

class  RectangleCreator;
class  RelocalizationDisplayWidget;
struct ui_params_t;

/**
* relocalization_widgets_t contains all the input widgets on the Relocalization panel.
*/
struct relocalization_widgets_t
{
    RelocalizationDisplayWidget* displayWidget;
    
    wxButton*   loadLPMButton;
    wxButton*   loadGMMButton;
    wxCheckBox* showLaserBox;
    wxCheckBox* showErrorBox;
    wxCheckBox* showParticlesBox;
    wxTextCtrl* regionNumSamplesText;
    wxTextCtrl* regionPosesPerPosText;
    wxTextCtrl* freeSpaceStrideText;
    wxTextCtrl* freeSpacePosesPerCellText;
    
    relocalization_widgets_t(void)
    : loadLPMButton(nullptr)
    , loadGMMButton(nullptr)
    , showLaserBox(nullptr)
    , showErrorBox(nullptr)
    , showParticlesBox(nullptr)
    , regionNumSamplesText(nullptr)
    , regionPosesPerPosText(nullptr)
    , freeSpaceStrideText(nullptr)
    , freeSpacePosesPerCellText(nullptr)
    {
    }
};

/**
* RelocalizationPanel handles events for the display of relocalization efforts and the creation of relocalization
* requests.
*/
class RelocalizationPanel : public UIPanel
{
public:
    
    /**
    * Constructor for RelocalizationPanel.
    * 
    * \param    params          Parameters for the panel
    * \param    widgets         Widgets associated with the panel
    */
    RelocalizationPanel(const ui_params_t& params, const relocalization_widgets_t widgets);
    
    // UIPanel interface
    virtual void setup       (wxGLContext* context, wxStatusBar* statusBar);
    virtual void subscribe   (system::ModuleCommunicator& producer);
    virtual void setConsumer (system::ModuleCommunicator* consumer);
    virtual void update      (void);
    virtual void saveSettings(utils::ConfigFileWriter& config);
    virtual void loadSettings(const utils::ConfigFile& config);
    
    virtual void handleData(const hssh::local_metric_relocalization_debug_info_t& info, const std::string& channel);
    virtual void handleData(const hssh::global_metric_relocalization_info_t&      info, const std::string& channel);
    
private:
    
    enum Mode
    {
        kLocal,
        kGlobal
    };
    
    Mode mode_;
    
    std::unique_ptr<RectangleCreator> regionCreator_;
    
    std::shared_ptr<hssh::LocalPerceptualMap> loadedLPM_;
    std::shared_ptr<hssh::GlobalMetricMap>    loadedGMM_;
    
    bool      shouldUpdateInitialRegion;            ///< Flag indicating that the initial region needs to keep being updated
    math::Rectangle<float> initialRegion_;
    
    relocalization_widgets_t    widgets_;
    system::ModuleCommunicator* consumer_;

    void loadLPM               (void);
    void loadGMM               (void);
    void changeDisplayedMap    (void);
    void enableRegionSelection (void);
    void disableRegionSelection(void);
    
    void sendRelocalizationMessage      (const std::shared_ptr<hssh::FilterInitializer>& initializer);
    void sendLocalRelocalizationMessage (const std::shared_ptr<hssh::FilterInitializer>& initializer);
    void sendGlobalRelocalizationMessage(const std::shared_ptr<hssh::FilterInitializer>& initializer);
    
    // Event handlers
    void modeChanged                   (wxCommandEvent& event);
    void showLaserChecked              (wxCommandEvent& event);
    void showErrorChecked              (wxCommandEvent& event);
    void showParticlesChecked          (wxCommandEvent& event);
    void loadLPMPressed                (wxCommandEvent& event);
    void loadGMMPressed                (wxCommandEvent& event);
    void setRegionPressed              (wxCommandEvent& event);
    void sendRegionMessagePressed      (wxCommandEvent& event);
    void sendFreeSpaceMessagePressed   (wxCommandEvent& event);
    void sendScanMatchingMessagePressed(wxCommandEvent& event);
    
    DECLARE_EVENT_TABLE()
};

} // namespace ui
} // namespace vulcan

#endif // UI_DEBUG_RELOCALIZATION_PANEL_H

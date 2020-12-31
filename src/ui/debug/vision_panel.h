/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     vision_panel.h
* \author   Collin Johnson
*
* Declaration of VisionPanel.
*/

#ifndef UI_DEBUG_VISION_PANEL_H
#define UI_DEBUG_VISION_PANEL_H

#include "ui/common/ui_panel.h"
#include "core/image.h"
#include "utils/mutex.h"
#include <atomic>

class wxSlider;

namespace vulcan
{
namespace vision { class ImageSegmenter; }
namespace ui
{

class ui_params_t;
class VisionDisplayWidget;

struct vision_panel_widgets_t
{
    VisionDisplayWidget* displayWidget;
    wxSlider*            minEdgeWeightSlider;
    wxSlider*            maxEdgeWeightSlider;
    wxSlider*            pixelSigmaSlider;
    wxSlider*            creditMultiplierSlider;
    wxSlider*            filterWidthSlider;
};

/**
* VisionPanel is responsible for handling events and data flow within the Vision tab
* in the DebugUI.
*/
class VisionPanel : public UIPanel
{
public:

    /**
    * Constructor for VisionPanel.
    */
    VisionPanel(const ui_params_t& params, const vision_panel_widgets_t& widgets);

    // UIPanel interface
    virtual void setup       (wxGLContext* context, wxStatusBar* statusBar);
    virtual void subscribe   (system::ModuleCommunicator& producer);
    virtual void setConsumer (system::ModuleCommunicator* consumer);
    virtual void update      (void);
    virtual void saveSettings(utils::ConfigFileWriter& config);
    virtual void loadSettings(const utils::ConfigFile& config);

    // Handlers for incoming data
    void handleData(const Image& image, const std::string& channel);

private:

    vision_panel_widgets_t widgets_;

    std::unique_ptr<vision::ImageSegmenter> imageSegmenter_;
    Image     image_;
    std::atomic<bool> haveNewImage_;
    bool              shouldFindSegments_;

    utils::Mutex imageLock_;

    void reloadSegmenter(void);

    // Event handlers for the vision panel
    void showSegmentsChecked(wxCommandEvent& event);
    void changedMinEdgeWeight(wxCommandEvent& event);
    void changedMaxEdgeWeight(wxCommandEvent& event);
    void changedPixelSigma(wxCommandEvent& event);
    void changedCreditMultiplier(wxCommandEvent& event);
    void changedFilterWidth(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};

}
}

#endif // UI_DEBUG_VISION_PANEL_H

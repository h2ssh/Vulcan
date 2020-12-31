/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     evaluation_panel.h
* \author   Collin Johnson
*
* Declaration of EvaluationPanel.
*/

#ifndef UI_DEBUG_EVALUATION_PANEL_H
#define UI_DEBUG_EVALUATION_PANEL_H

#include "ui/common/ui_panel.h"
#include <wx/wx.h>

namespace vulcan
{
namespace hssh { class AreaStabilityAnalyzer; }
namespace hssh { class LocalTopoMap; }
namespace ui
{

class EvaluationDisplayWidget;
struct ui_params_t;


struct evaluation_panel_widgets_t
{
    EvaluationDisplayWidget* widget = nullptr;

    wxCheckBox* drawBoundaryBox = nullptr;
    wxCheckBox* drawStarBox = nullptr;
};

/**
* EvaluationPanel
*/
class EvaluationPanel : public UIPanel
{
public:

    /**
    * Constructor for EvaluationPanel.
    *
    * \param    params          Parameters for the rendering
    * \param    widgets         Interactive widgets on the panel
    */
    EvaluationPanel(const ui_params_t& params, const evaluation_panel_widgets_t& widgets);

    // UIPanel interface
    void setup(wxGLContext* context, wxStatusBar* statusBar) override;
    void subscribe(system::ModuleCommunicator& communicator) override;
    void setConsumer(system::ModuleCommunicator* communicator) override;
    void update(void) override;
    void saveSettings(utils::ConfigFileWriter& config) override;
    void loadSettings(const utils::ConfigFile& config) override;

private:

    std::shared_ptr<hssh::AreaStabilityAnalyzer> stabilityAnalyzer_;
    std::shared_ptr<hssh::LocalTopoMap> map_;
    evaluation_panel_widgets_t widgets_;

    // Event handlers
    void loadMapPressed(wxCommandEvent& event);
    void importLogPressed(wxCommandEvent& event);
    void clearAllPressed(wxCommandEvent& event);
    void drawBoundaryChanged(wxCommandEvent& event);
    void drawStarChanged(wxCommandEvent& event);
    void loadResultsLogsPressed(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};

} // namespace ui
} // namespace vulcan

#endif // UI_DEBUG_EVALUATION_PANEL_H

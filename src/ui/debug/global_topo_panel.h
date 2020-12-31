/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     global_topo_panel.h
 * \author   Collin Johnson
 *
 * Declaration of GlobalTopoPanel for event handling with the GlobalTopology panel.
 */

#ifndef UI_DEBUG_GLOBAL_TOPO_PANEL_H
#define UI_DEBUG_GLOBAL_TOPO_PANEL_H

#include "ui/common/ui_forward_declarations.h"
#include "ui/common/ui_panel.h"
#include <vector>
#include <wx/wx.h>

class wxGrid;

namespace vulcan
{
namespace ui
{

class GlobalTopoDisplayWidget;
struct ui_params_t;

struct global_topo_panel_widgets_t
{
    GlobalTopoDisplayWidget* displayWidget = nullptr;
    wxComboBox* activeMapComboBox = nullptr;
    wxStaticText* numHypothesesLabel = nullptr;
    wxStaticText* numCompleteLabel = nullptr;
    wxGrid* hypothesisInfoGrid = nullptr;
};

/**
 * GlobalTopoPanel is the event and data handler for the Global Topology panel in the DebugUI.
 */
class GlobalTopoPanel : public UIPanel
{
public:
    /**
     * Constructor for GlobalTopoPanel.
     *
     * \param    params          Parameters for the panel widgets
     * \param    widgets         Widgets that control behavior of the panel
     */
    GlobalTopoPanel(const ui_params_t& params, const global_topo_panel_widgets_t& widgets);

    // UIPanel interface
    void setup(wxGLContext* context, wxStatusBar* statusBar) override;
    void subscribe(system::ModuleCommunicator& producer) override;
    void setConsumer(system::ModuleCommunicator* consumer) override;
    void update(void) override;
    void saveSettings(utils::ConfigFileWriter& config) override;
    void loadSettings(const utils::ConfigFile& config) override;

    void handleData(const hssh::HypothesisTree& tree, const std::string& channel);

private:
    GlobalTopoDisplayWidget* widget;
    wxComboBox* activeMapComboBox;
    wxStaticText* numHypothesesLabel;
    wxStaticText* numCompleteLabel;
    wxGrid* hypothesisInfoGrid;

    bool showingTree;

    wxArrayString numberStrings;
    std::size_t numHypothesesInCombo;
    std::size_t numActiveHypotheses;
    std::size_t numCompleteHypotheses;
    bool hypothesesChanged;

    std::shared_ptr<hssh::TreeOfMaps> tree_;

    system::ModuleCommunicator* consumer;


    void updateTreeOfMaps(void);

    void selectedMapView(wxCommandEvent& event);
    void showBestMap(wxCommandEvent& event);
    void activeMapSelected(wxCommandEvent& event);
    void previousMapPressed(wxCommandEvent& event);
    void nextMapPressed(wxCommandEvent& event);
    void useCurrentMapPressed(wxCommandEvent& event);
    void loadGlobalTopoMapPressed(wxCommandEvent& event);
    void saveGlobalTopoMapPressed(wxCommandEvent& event);
    void saveTreePressed(wxCommandEvent& event);
    void loadTreePressed(wxCommandEvent& event);
    void saveMapCachePressed(wxCommandEvent& event);
    void loadMapCachePressed(wxCommandEvent& event);
    void clearMapsPressed(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_DEBUG_GLOBAL_TOPO_PANEL_H

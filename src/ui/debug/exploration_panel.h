/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     exploration_panel.h
* \author   Collin Johnson
* 
* Declaration of ExplorationPanel.
*/

#ifndef UI_DEBUG_EXPLORATION_PANEL_H
#define UI_DEBUG_EXPLORATION_PANEL_H

#include "ui/common/ui_panel.h"
#include "planner/exploration/local_topo/exploration_status.h"
#include "utils/mutex.h"
#include <wx/wx.h>
#include <memory>

namespace vulcan
{
namespace hssh { class LocalPerceptualMap; }
namespace hssh { class LocalPose; }
namespace mpepc { struct dynamic_object_trajectory_debug_info_t; }
namespace mpepc { struct trajectory_planner_debug_info_t; }
namespace ui
{

class ExplorationDisplayWidget;
struct ui_params_t;

/**
* exploration_panel_widgets_t contains all widgets that are manipulated by the ExplorationPanel.
*/
struct exploration_panel_widgets_t
{
    ExplorationDisplayWidget* display = nullptr;

    wxStaticText* totalText = nullptr;
    wxStaticText* visitedText = nullptr;
    wxStaticText* remainingText = nullptr;

    wxCheckBox* centerOnRobotCheckbox = nullptr;
};

/**
* ExplorationPanel displays information related to the map_exploration module.
*/
class ExplorationPanel : public UIPanel
{
public:

    /**
    * Constructor for ExplorationPanel.
    *
    * \param    params          Parameters for the rendering
    * \param    widgets         Interactive widgets on the panel
    */
    ExplorationPanel(const ui_params_t& params, const exploration_panel_widgets_t& widgets);

    // UIPanel interface
    void setup(wxGLContext* context, wxStatusBar* statusBar) override;
    void subscribe(system::ModuleCommunicator& communicator) override;
    void setConsumer(system::ModuleCommunicator* communicator) override;
    void update(void) override;
    void saveSettings(utils::ConfigFileWriter& config) override;
    void loadSettings(const utils::ConfigFile& config) override;

    // Data handlers
    void handleData(const hssh::LocalPose& pose, const std::string& channel);
    void handleData(const planner::local_topo_exploration_status_t& status, const std::string& channel);
    void handleData(const hssh::LocalPerceptualMap& lpm, const std::string& channel);
    void handleData(const std::vector<mpepc::dynamic_object_trajectory_debug_info_t>& objectTrajectories,
                    const std::string& channel);
    void handleData(const mpepc::trajectory_planner_debug_info_t& mpepcInfo, const std::string& channel);

private:

    exploration_panel_widgets_t widgets_;
    int numLocalTopoVisited_;
    int numLocalTopoRemaining_;
    utils::Mutex dataLock_;

    void updateExplorationStatusText(void);

    // UI event handling methods
    void centerOnRobotChanged(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};

} // namespace ui
} // namespace vulcan

#endif // UI_DEBUG_EXPLORATION_PANEL_H

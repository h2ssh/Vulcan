/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     evaluation_panel.cpp
* \author   Collin Johnson
*
* Definition of EvaluationPanel.
*/

#include <ui/debug/evaluation_panel.h>
#include <ui/debug/evaluation_display_widget.h>
#include <ui/debug/debug_ui.h>
#include <ui/common/file_dialog_settings.h>
#include <hssh/local_topological/evaluation/stability_analyzer.h>
#include <hssh/local_topological/evaluation/stability_log.h>
#include <mpepc/evaluation/path_summary.h>
#include <utils/serialized_file_io.h>

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(EvaluationPanel, wxEvtHandler)
    EVT_BUTTON(ID_LOAD_EVAL_MAP_BUTTON, EvaluationPanel::loadMapPressed)
    EVT_BUTTON(ID_IMPORT_STABILITY_LOG_BUTTON, EvaluationPanel::importLogPressed)
    EVT_BUTTON(ID_CLEAR_STABILITY_EVAL_BUTTON, EvaluationPanel::clearAllPressed)
    EVT_CHECKBOX(ID_DRAW_EVAL_BOUNDARY_BOX, EvaluationPanel::drawBoundaryChanged)
    EVT_CHECKBOX(ID_DRAW_EVAL_STAR_BOX, EvaluationPanel::drawStarChanged)
    EVT_BUTTON(ID_LOAD_MPEPC_RESULTS_FILE_BUTTON, EvaluationPanel::loadResultsLogsPressed)
END_EVENT_TABLE()


EvaluationPanel::EvaluationPanel(const ui_params_t& params, const evaluation_panel_widgets_t& widgets)
: widgets_(widgets)
{
    assert(widgets_.widget);
    assert(widgets_.drawBoundaryBox);
    assert(widgets_.drawStarBox);
}


void EvaluationPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widgets_.widget->setRenderContext(context);
    widgets_.widget->setStatusBar(statusBar);
}


void EvaluationPanel::subscribe(system::ModuleCommunicator& communicator)
{
    // Not currently receiving output -- standalone operation
}


void EvaluationPanel::setConsumer(system::ModuleCommunicator* communicator)
{
    // Not currently sending output -- standalone operation
}


void EvaluationPanel::update(void)
{
    widgets_.widget->Refresh();
}


void EvaluationPanel::saveSettings(utils::ConfigFileWriter& config)
{

}


void EvaluationPanel::loadSettings(const utils::ConfigFile& config)
{

}


void EvaluationPanel::loadMapPressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets_.widget,
                            wxT("Select map file..."),
                            wxT(""),
                            wxT(""),
                            wxT("*.ltm"),
                            kFileOpenFlags);

    if(loadDialog.ShowModal() == wxID_OK)
    {
        auto path = std::string{loadDialog.GetPath().mb_str()};
        hssh::LocalTopoMap topoMap;

        if(!utils::load_serializable_from_file(path, topoMap))
        {
            std::cerr << "ERROR:LocalTopoPanel: Failed to load topo map to file " << path << '\n';
        }
        else
        {
            map_.reset(new hssh::LocalTopoMap(topoMap));
            widgets_.widget->setMap(map_);

            // Create a new analyzer for the loaded map
            stabilityAnalyzer_ = std::make_shared<hssh::AreaStabilityAnalyzer>(*map_);
            widgets_.widget->setAreas(stabilityAnalyzer_);
        }
    }
}


void EvaluationPanel::importLogPressed(wxCommandEvent& event)
{
    // If there's no analyzer, there's nothing to import a log into
    if(!stabilityAnalyzer_)
    {
        return;
    }

    wxFileDialog loadDialog(widgets_.widget,
                            wxT("Select log file..."),
                            wxT(""),
                            wxT(""),
                            wxT("*.log"),
                            kFileOpenFlags);

    if(loadDialog.ShowModal() == wxID_OK)
    {
        auto path = std::string{loadDialog.GetPath().mb_str()};
        hssh::AreaStabilityLog log(path);
        stabilityAnalyzer_->addLog(log);
    }
}


void EvaluationPanel::clearAllPressed(wxCommandEvent& event)
{
    // If we've actually loaded an analyzer, then just reinitialize it with a new map
    if(stabilityAnalyzer_)
    {
        stabilityAnalyzer_ = std::make_shared<hssh::AreaStabilityAnalyzer>(*map_);
        widgets_.widget->setAreas(stabilityAnalyzer_);
    }
}


void EvaluationPanel::drawBoundaryChanged(wxCommandEvent& event)
{
    widgets_.widget->shouldDrawBoundaries(event.IsChecked());
}


void EvaluationPanel::drawStarChanged(wxCommandEvent& event)
{
    widgets_.widget->shouldDrawStars(event.IsChecked());
}


void EvaluationPanel::loadResultsLogsPressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets_.widget,
                            wxT("Select results logs file..."),
                            wxT(""),
                            wxT(""),
                            wxT("*.*"),
                            kFileOpenFlags);

    if(loadDialog.ShowModal() == wxID_OK)
    {
        auto resultsFile = std::string{loadDialog.GetPath().mb_str()};
        mpepc::PathSummary paths(resultsFile);
        widgets_.widget->setPaths(paths);
    }
}

} // namespace ui
} // namespace vulcan

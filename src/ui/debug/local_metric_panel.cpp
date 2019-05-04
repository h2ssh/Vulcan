/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_metric_panel.cpp
* \author   Collin Johnson
*
* Definition of LocalMetricPanel.
*/

#include <ui/debug/local_metric_panel.h>
#include <ui/debug/local_metric_panel_text_updater.h>
#include <ui/debug/debug_ui.h>
#include <utils/timestamp.h>
#include <hssh/local_metric/commands/glass_evaluation.h>
#include <hssh/local_metric/commands/toggle_glass_mapping.h>
#include <hssh/local_metric/commands/set_slam_mode.h>
#include <hssh/local_metric/lpm_io.h>
#include <ui/debug/local_metric_display_widget.h>
#include <ui/common/grid_cell_selector.h>
#include <ui/common/file_dialog_settings.h>
#include <ui/components/glass_map_renderer.h>
#include <ui/debug/debug_ui.h>
#include <utils/serialized_file_io.h>
#include <utils/stub.h>
#include <system/module_communicator.h>
#include <cassert>

// Use a separate namespace declaration for the event table to help KDevelop parse the code. The macros throw it off.
namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(LocalMetricPanel, wxEvtHandler)
    EVT_RADIOBOX(ID_LOCAL_METRIC_MODE_BOX, LocalMetricPanel::modeChanged)
    EVT_RADIOBOX(ID_METRIC_GRID_TO_SHOW_RADIO, LocalMetricPanel::gridToShowChanged)
    EVT_RADIOBOX(ID_LASER_TO_SHOW_RADIO, LocalMetricPanel::laserToShowChanged)
    EVT_CHECKBOX(ID_SHOW_LASER_LINES_BOX, LocalMetricPanel::showLaserLinesChecked)
    EVT_CHECKBOX(ID_SHOW_EXTRACTED_LINES_BOX, LocalMetricPanel::showExtractedLinesChecked)
    EVT_CHECKBOX(ID_SHOW_INTENSITY_PLOTS_BOX, LocalMetricPanel::showIntensityPlotsChecked)
    EVT_CHECKBOX(ID_CENTER_ON_ROBOT, LocalMetricPanel::centerOnRobotChecked)
    EVT_CHECKBOX(ID_SHOW_POSE_TRACE, LocalMetricPanel::showPoseTraceChecked)
    EVT_CHECKBOX(ID_SHOW_MOTION_TRACE_BOX, LocalMetricPanel::showMotionTraceChecked)
    EVT_CHECKBOX(ID_SHOW_UNCERTAINTY_ELLIPSE, LocalMetricPanel::showUncertaintyEllipseChecked)
    EVT_CHECKBOX(ID_SHOW_PARTICLES, LocalMetricPanel::showParticlesChecked)
    EVT_BUTTON(ID_CLEAR_POSES_BUTTON, LocalMetricPanel::clearPosesPressed)
    EVT_BUTTON(ID_CLEAR_MOTION_BUTTON, LocalMetricPanel::clearMotionPressed)
    EVT_CHECKBOX(ID_SHOW_GLASS_INTENSITY_BOX, LocalMetricPanel::showGlassIntensityChecked)
    EVT_CHECKBOX(ID_SHOW_GLASS_WALLS_BOX, LocalMetricPanel::showWallsChecked)
    EVT_CHECKBOX(ID_SHOW_GLASS_ANGLES_BOX, LocalMetricPanel::showAnglesChecked)
    EVT_RADIOBOX(ID_GLASS_ANGLES_TO_SHOW_RADIO, LocalMetricPanel::anglesToShowChanged)
    EVT_BUTTON(ID_RUN_FLATTEN_MAP_BUTTON, LocalMetricPanel::runFlattenMapPressed)
    EVT_BUTTON(ID_RUN_DYNAMIC_FILTER_BUTTON, LocalMetricPanel::runDynamicFilterPressed)
    EVT_BUTTON(ID_ROTATE_LPM_BUTTON, LocalMetricPanel::rotateLPMPressed)
    EVT_BUTTON(ID_SAVE_CURRENT_LPM_BUTTON, LocalMetricPanel::saveLPMPressed)
    EVT_BUTTON(ID_SAVE_GLASS_MAP_BUTTON, LocalMetricPanel::saveGlassPressed)
    EVT_BUTTON(ID_LOAD_GLASS_MAP_BUTTON, LocalMetricPanel::loadGlassPressed)
    EVT_BUTTON(ID_SAVE_POSES_BUTTON, LocalMetricPanel::savePosesPressed)
    EVT_BUTTON(ID_SAVE_SCANS_BUTTON, LocalMetricPanel::saveScansPressed)
END_EVENT_TABLE()

}
}

namespace vulcan
{
namespace ui
{

enum LocalMetricModeRadio
{
    slam,
    slam_plus_glass,
    localization_only,
    high_res_slam,
};

const std::string kLPMFilename("current.lpm");
const std::string kGlassMapFilename("current_glass.map");


LocalMetricMapType get_map_from_radio(int selection);
LaserToShowType    get_laser_from_radio(int selection);
GlassAnglesToDraw  get_angles_from_radio(int selection);


LocalMetricPanel::LocalMetricPanel(const ui_params_t& params, const local_metric_panel_widgets_t& widgets)
: widgets(widgets)
, consumer(0)
{
    assert(widgets.lpmWidget);
    assert(widgets.updater);
    assert(widgets.localizationModeCheck);
    assert(widgets.gridToShowRadio);
    assert(widgets.centerOnRobotCheck);
    assert(widgets.laserToShowRadio);
    assert(widgets.showRaysCheck);
    assert(widgets.showExtractedCheck);
    assert(widgets.showIntensityCheck);
    assert(widgets.showPoseTraceCheck);
    assert(widgets.showMotionTraceCheck);
    assert(widgets.showErrorCheck);
    assert(widgets.showParticlesCheck);
    assert(widgets.rotationAngleText);
    assert(widgets.showGlassIntensityCheck);
    assert(widgets.showWallsCheck);
    assert(widgets.showAnglesCheck);
    assert(widgets.anglesToShowRadio);
    assert(widgets.flattenThresholdSlider);
    assert(widgets.highlyVisibleThresholdSlider);

    widgets.lpmWidget->setWidgetParams(params.lpmParams);
    widgets.lpmWidget->setMapToShow(get_map_from_radio(widgets.gridToShowRadio->GetSelection()));
    widgets.lpmWidget->centerOnRobot(widgets.centerOnRobotCheck->IsChecked());
    widgets.lpmWidget->setLaserToShow(get_laser_from_radio(widgets.laserToShowRadio->GetSelection()));
    widgets.lpmWidget->showLaserLines(widgets.showRaysCheck->IsChecked());
    widgets.lpmWidget->showExtractedLines(widgets.showExtractedCheck->IsChecked());
    widgets.lpmWidget->showExtractedLines(widgets.showIntensityCheck->IsChecked());
    widgets.lpmWidget->showPoseTrace(widgets.showPoseTraceCheck->IsChecked());
    widgets.lpmWidget->showMotionTrace(widgets.showMotionTraceCheck->IsChecked());
    widgets.lpmWidget->showGlassIntensity(widgets.showGlassIntensityCheck->IsChecked());
    widgets.lpmWidget->showGlassWalls(widgets.showWallsCheck->IsChecked());
    widgets.lpmWidget->showGlassAngles(widgets.showAnglesCheck->IsChecked());
    widgets.lpmWidget->setAnglesToShow(get_angles_from_radio(widgets.anglesToShowRadio->GetSelection()));
}


void LocalMetricPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widgets.lpmWidget->setRenderContext(context);
    widgets.lpmWidget->setStatusBar(statusBar);
}


void LocalMetricPanel::subscribe(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<polar_laser_scan_t>(widgets.lpmWidget);
    producer.subscribeTo<laser::MovingLaserScan>(widgets.lpmWidget);
    producer.subscribeTo<laser::ReflectedLaserScan>(widgets.lpmWidget);
    producer.subscribeTo<laser::laser_scan_lines_t>(widgets.lpmWidget);
    producer.subscribeTo<robot::proximity_warning_indices_t>(widgets.lpmWidget);
    producer.subscribeTo<hssh::LocalPerceptualMap>(widgets.lpmWidget);
    producer.subscribeTo<hssh::GlassMap>(widgets.lpmWidget);
    producer.subscribeTo<pose_t>(widgets.lpmWidget);
    producer.subscribeTo<pose_distribution_t>(widgets.lpmWidget);
    producer.subscribeTo<motion_state_t>(widgets.lpmWidget);
    producer.subscribeTo<hssh::local_metric_localization_debug_info_t>(widgets.lpmWidget);
    
    producer.subscribeTo<imu_data_t>(widgets.updater);
    producer.subscribeTo<motion_state_t>(widgets.updater);
    producer.subscribeTo<robot::commanded_velocity_t>(widgets.updater);
}


void LocalMetricPanel::setConsumer(system::ModuleCommunicator* consumer)
{
    if(consumer)
    {
        this->consumer = consumer;
    }
}


void LocalMetricPanel::update(void)
{
    widgets.updater->update();
    widgets.lpmWidget->Refresh();
}


void LocalMetricPanel::saveSettings(utils::ConfigFileWriter& config)
{
    // Saving settings not supported
}


void LocalMetricPanel::loadSettings(const utils::ConfigFile& config)
{
    // Loading settings not supported
}

// Event handlers
void LocalMetricPanel::modeChanged(wxCommandEvent& event)
{
    hssh::SlamMode      mode = hssh::SlamMode::unchanged;
    hssh::MapResolution res  = hssh::MapResolution::unchanged;
    bool useGlass = false;
    
    if(event.GetSelection() == slam)
    {
        mode = hssh::SlamMode::full_slam;
    }
    else if(event.GetSelection() == slam_plus_glass)
    {
        mode = hssh::SlamMode::full_slam;
        useGlass = true;
    }
    else if(event.GetSelection() == localization_only)
    {
        mode = hssh::SlamMode::localization_only;
    }
    else if(event.GetSelection() == high_res_slam)
    {
        res  = hssh::MapResolution::high;
        mode = hssh::SlamMode::full_slam;
    }
    else // Unknown mode type, so exit immediately as something has gone wrong
    {
        return;
    }
    
    std::shared_ptr<hssh::LocalMetricCommand> message = std::make_shared<hssh::SetSlamModeCommand>("DebugUI", 
                                                                                                   mode, 
                                                                                                   hssh::MappingMode::unchanged, 
                                                                                                   res);
    consumer->sendMessage(message);

    std::shared_ptr<hssh::LocalMetricCommand> glassMessage = std::make_shared<hssh::ToggleGlassMapping>(useGlass,
                                                                                                        "DebugUI");
    consumer->sendMessage(glassMessage);
}


void LocalMetricPanel::gridToShowChanged(wxCommandEvent& event)
{
    widgets.lpmWidget->setMapToShow(get_map_from_radio(event.GetSelection()));
}


void LocalMetricPanel::laserToShowChanged(wxCommandEvent& event)
{
    widgets.lpmWidget->setLaserToShow(get_laser_from_radio(event.GetSelection()));
    std::cout << "Laser to show:" << event.GetSelection() << '\n';
}


void LocalMetricPanel::showLaserLinesChecked(wxCommandEvent& event)
{
    widgets.lpmWidget->showLaserLines(event.IsChecked());
}


void LocalMetricPanel::showExtractedLinesChecked(wxCommandEvent& event)
{
    widgets.lpmWidget->showExtractedLines(event.IsChecked());
}


void LocalMetricPanel::showIntensityPlotsChecked(wxCommandEvent& event)
{
    widgets.lpmWidget->showIntensityPlots(event.IsChecked());
}


void LocalMetricPanel::centerOnRobotChecked(wxCommandEvent& event)
{
    widgets.lpmWidget->centerOnRobot(event.IsChecked());
}


void LocalMetricPanel::showPoseTraceChecked(wxCommandEvent& event)
{
    widgets.lpmWidget->showPoseTrace(event.IsChecked());
}


void LocalMetricPanel::showMotionTraceChecked(wxCommandEvent& event)
{
    widgets.lpmWidget->showMotionTrace(event.IsChecked());
}


void LocalMetricPanel::showUncertaintyEllipseChecked(wxCommandEvent& event)
{
    widgets.lpmWidget->showUncertaintyEllipse(event.IsChecked());
}


void LocalMetricPanel::showParticlesChecked(wxCommandEvent& event)
{
    widgets.lpmWidget->showParticles(event.IsChecked());
}


void LocalMetricPanel::clearPosesPressed(wxCommandEvent& event)
{
    widgets.lpmWidget->clearPoseTrace();
}


void LocalMetricPanel::clearMotionPressed(wxCommandEvent& event)
{
    widgets.lpmWidget->clearMotionTrace();
}


void LocalMetricPanel::showGlassIntensityChecked(wxCommandEvent& event)
{
    widgets.lpmWidget->showGlassIntensity(event.IsChecked());
}


void LocalMetricPanel::showWallsChecked(wxCommandEvent& event)
{
    widgets.lpmWidget->showGlassWalls(event.IsChecked());
}


void LocalMetricPanel::showAnglesChecked(wxCommandEvent& event)
{
    widgets.lpmWidget->showGlassAngles(event.IsChecked());
}


void LocalMetricPanel::anglesToShowChanged(wxCommandEvent& event)
{
    widgets.lpmWidget->setAnglesToShow(get_angles_from_radio(event.GetSelection()));
}


void LocalMetricPanel::runFlattenMapPressed(wxCommandEvent& event)
{
    widgets.lpmWidget->flattenGlassMap(widgets.flattenThresholdSlider->GetValue());
}


void LocalMetricPanel::runDynamicFilterPressed(wxCommandEvent& event)
{
    widgets.lpmWidget->filterDynamicObjectsFromGlass(widgets.highlyVisibleThresholdSlider->GetValue());
}


void LocalMetricPanel::rotateLPMPressed(wxCommandEvent& event)
{
    wxString angleStr = widgets.rotationAngleText->GetValue();

    double rotationAngle = 0.0;

    if(angleStr.ToDouble(&rotationAngle))
    {
        rotationAngle = rotationAngle * M_PI / 180.0;

        std::cout<<"INFO:LocalMetricPanel:Rotating LPM "<<rotationAngle<<" radians\n";

        widgets.lpmWidget->rotateLPM(rotationAngle);
    }
    else
    {
        std::cerr<<"ERROR:LocalMetricPanel: Invalid rotation angle\n";
    }
}


void LocalMetricPanel::saveLPMPressed(wxCommandEvent& event)
{
    hssh::save_lpm_1_0(widgets.lpmWidget->getLPM(), kLPMFilename);
}


void LocalMetricPanel::saveGlassPressed(wxCommandEvent& event)
{
    std::shared_ptr<hssh::LocalMetricCommand> command = hssh::GlassEvaluationCommand::CreateSaveMapMessage("DebugUI",
        kGlassMapFilename);
    consumer->sendMessage(command);
}


void LocalMetricPanel::loadGlassPressed(wxCommandEvent& event)
{
    hssh::GlassMap glassMap;
    if(utils::load_serializable_from_file(kGlassMapFilename, glassMap))
    {
        widgets.lpmWidget->setGlassMap(glassMap);
    }
    else
    {
        std::cerr << "ERROR: LocalMetricPanel: Failed to load GlassMap from " << kGlassMapFilename << '\n';
    }
}

void LocalMetricPanel::savePosesPressed(wxCommandEvent& event)
{
    wxFileDialog saveDialog(widgets.mainFrame,
                            _("Save File As _?"), 
                            wxEmptyString, 
                            "update_times.plog",
                            _("Pose Log files (*.plog)|*.plog"),
                            kFileSaveFlags,
                            wxDefaultPosition);

    if(saveDialog.ShowModal() == wxID_OK) // If the user clicked "OK"
    {
        std::shared_ptr<hssh::LocalMetricCommand> command = hssh::GlassEvaluationCommand::CreateSavePosesMessage("DebugUI",
                                                                                                                 saveDialog.GetPath().ToStdString());
        consumer->sendMessage(command);
    }
}

void LocalMetricPanel::saveScansPressed(wxCommandEvent& event)
{
    wxFileDialog saveDialog(widgets.mainFrame, _("Save File As _?"), 
                            wxEmptyString, 
                            "scans.scan",
                            _("Scan file (*.scan)|*.scan"),
                            kFileSaveFlags, 
                            wxDefaultPosition);
    // Creates a Save Dialog with 4 file types
    if (saveDialog.ShowModal() == wxID_OK) // If the user clicked "OK"
    {
        std::shared_ptr<hssh::LocalMetricCommand> command = hssh::GlassEvaluationCommand::CreateSaveScansMessage("DebugUI",
                                                                                                                 saveDialog.GetPath().ToStdString());
        consumer->sendMessage(command);
    }
}

LocalMetricMapType get_map_from_radio(int selection)
{
    switch(selection)
    {
    case 0:
        return LocalMetricMapType::LPM;
        
    case 1:
        return LocalMetricMapType::GLASS;
        
    default:
        return LocalMetricMapType::NONE;
    }
    
    return LocalMetricMapType::NONE;
}


LaserToShowType get_laser_from_radio(int selection)
{
    switch(selection)
    {
        case 0:
            return LaserToShowType::raw;
        case 1:
            return LaserToShowType::mapping;
        case 2:
            return LaserToShowType::reflected;
        case 3:
        default:
            return LaserToShowType::none;
    }
    
    return LaserToShowType::none;
}


GlassAnglesToDraw get_angles_from_radio(int selection)
{
    switch(selection)
    {
    case 0:
        return GlassAnglesToDraw::normal;
    case 1:
        return GlassAnglesToDraw::range;
    default:
        std::cerr << "ERROR: LocalMetricPanel: Unknown type of glass angle to be shown. Valid: [0,1]. Provided:"
            << selection << '\n';
    }
    
    return GlassAnglesToDraw::normal;
}

} // namespace ui
} // namespace vulcan

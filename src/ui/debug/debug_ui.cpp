///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version 3.9.0 Dec 31 2020)
// http://www.wxformbuilder.org/
//
// PLEASE DO *NOT* EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "ui/debug/calibration_display_widget.h"
#include "ui/debug/decision_planner_display_widget.h"
#include "ui/debug/evaluation_display_widget.h"
#include "ui/debug/exploration_widget.h"
#include "ui/debug/global_metric_display_widget.h"
#include "ui/debug/global_topo_display_widget.h"
#include "ui/debug/goal_planner_display_widget.h"
#include "ui/debug/local_metric_display_widget.h"
#include "ui/debug/local_topo_display_widget.h"
#include "ui/debug/metric_planner_display_widget.h"
#include "ui/debug/planner_scripting_widget.h"
#include "ui/debug/relocalization_display_widget.h"
#include "ui/debug/tracker_display_widget.h"
#include "ui/debug/vision_display_widget.h"

#include "debug_ui.h"

///////////////////////////////////////////////////////////////////////////
using namespace vulcan::ui;

DebugFrame::DebugFrame(wxWindow* parent,
                       wxWindowID id,
                       const wxString& title,
                       const wxPoint& pos,
                       const wxSize& size,
                       long style)
: UIMainFrame(parent, id, title, pos, size, style)
{
    this->SetSizeHints(wxDefaultSize, wxDefaultSize);

    wxBoxSizer* debugFrameSizer;
    debugFrameSizer = new wxBoxSizer(wxHORIZONTAL);

    frameNotebook = new wxAuiNotebook(this,
                                      wxID_ANY,
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      wxAUI_NB_SCROLL_BUTTONS | wxAUI_NB_TAB_MOVE | wxAUI_NB_TAB_SPLIT);
    localMetricPanel = new wxPanel(frameNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxBoxSizer* localMetricPanelSizer;
    localMetricPanelSizer = new wxBoxSizer(wxHORIZONTAL);

    wxFlexGridSizer* localMetricWidgetSizer;
    localMetricWidgetSizer = new wxFlexGridSizer(2, 1, 0, 0);
    localMetricWidgetSizer->AddGrowableCol(0);
    localMetricWidgetSizer->AddGrowableRow(0);
    localMetricWidgetSizer->SetFlexibleDirection(wxBOTH);
    localMetricWidgetSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    localMetricWidget =
      new LocalMetricDisplayWidget(localMetricPanel, ID_LOCAL_METRIC_WIDGET, wxDefaultPosition, wxDefaultSize);
    localMetricWidget->SetSize(wxSize(800, 600));
    localMetricWidget->SetMinSize(wxSize(0, 0));

    localMetricWidgetSizer->Add(localMetricWidget, 0, wxALIGN_CENTER | wxEXPAND, 0);

    wxFlexGridSizer* localMetricInfoSizer;
    localMetricInfoSizer = new wxFlexGridSizer(3, 6, 0, 0);
    localMetricInfoSizer->SetFlexibleDirection(wxBOTH);
    localMetricInfoSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    poseLabel = new wxStaticText(localMetricPanel, wxID_ANY, wxT("Pose:"), wxDefaultPosition, wxDefaultSize, 0);
    poseLabel->Wrap(-1);
    poseLabel->SetFont(wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxT("Sans")));

    localMetricInfoSizer->Add(poseLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    poseDisplay =
      new wxStaticText(localMetricPanel, wxID_ANY, wxT("(00.00, 00.00, 0.00)"), wxDefaultPosition, wxDefaultSize, 0);
    poseDisplay->Wrap(-1);
    poseDisplay->SetFont(wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("Fixed")));

    localMetricInfoSizer->Add(poseDisplay, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    measuredVelLabel =
      new wxStaticText(localMetricPanel, wxID_ANY, wxT("Velocity:"), wxDefaultPosition, wxDefaultSize, 0);
    measuredVelLabel->Wrap(-1);
    measuredVelLabel->SetFont(
      wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxT("Sans")));

    localMetricInfoSizer->Add(measuredVelLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    velocityDisplay =
      new wxStaticText(localMetricPanel, wxID_ANY, wxT("(+0.00, +0.00)"), wxDefaultPosition, wxDefaultSize, 0);
    velocityDisplay->Wrap(-1);
    velocityDisplay->SetFont(
      wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("Fixed")));

    localMetricInfoSizer->Add(velocityDisplay, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    imuLabel = new wxStaticText(localMetricPanel, wxID_ANY, wxT("IMU: Accel:"), wxDefaultPosition, wxDefaultSize, 0);
    imuLabel->Wrap(-1);
    imuLabel->SetFont(wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxT("Sans")));

    localMetricInfoSizer->Add(imuLabel, 0, wxALIGN_CENTER | wxALIGN_RIGHT | wxALL, 5);

    imuAccelDisplay =
      new wxStaticText(localMetricPanel, wxID_ANY, wxT("(0.00, 0.00, 0.00)"), wxDefaultPosition, wxDefaultSize, 0);
    imuAccelDisplay->Wrap(-1);
    imuAccelDisplay->SetFont(
      wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("Fixed")));

    localMetricInfoSizer->Add(imuAccelDisplay, 0, wxALL, 5);

    commandLabel = new wxStaticText(localMetricPanel, wxID_ANY, wxT("Command:"), wxDefaultPosition, wxDefaultSize, 0);
    commandLabel->Wrap(-1);
    commandLabel->SetFont(wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxT("Sans")));

    localMetricInfoSizer->Add(commandLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    commandDisplay =
      new wxStaticText(localMetricPanel, wxID_ANY, wxT("(0.00, 0.00)"), wxDefaultPosition, wxDefaultSize, 0);
    commandDisplay->Wrap(-1);
    commandDisplay->SetFont(
      wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("Fixed")));

    localMetricInfoSizer->Add(commandDisplay, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    leftWheelLabel =
      new wxStaticText(localMetricPanel, wxID_ANY, wxT("Wheels: Left:"), wxDefaultPosition, wxDefaultSize, 0);
    leftWheelLabel->Wrap(-1);
    leftWheelLabel->SetFont(wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxT("Sans")));

    localMetricInfoSizer->Add(leftWheelLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    leftWheelDisplay =
      new wxStaticText(localMetricPanel, wxID_ANY, wxT("(+0.00, +0.00)"), wxDefaultPosition, wxDefaultSize, 0);
    leftWheelDisplay->Wrap(-1);
    leftWheelDisplay->SetFont(
      wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("Fixed")));

    localMetricInfoSizer->Add(leftWheelDisplay, 0, wxALL, 5);

    imuVelocityLabel =
      new wxStaticText(localMetricPanel, wxID_ANY, wxT("Velocity:"), wxDefaultPosition, wxDefaultSize, 0);
    imuVelocityLabel->Wrap(-1);
    imuVelocityLabel->SetFont(
      wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxT("Sans")));

    localMetricInfoSizer->Add(imuVelocityLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    imuVelocityDisplay =
      new wxStaticText(localMetricPanel, wxID_ANY, wxT("(0.00, 0.00, 0.00)"), wxDefaultPosition, wxDefaultSize, 0);
    imuVelocityDisplay->Wrap(-1);
    imuVelocityDisplay->SetFont(
      wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("Fixed")));

    localMetricInfoSizer->Add(imuVelocityDisplay, 0, wxALL, 5);

    m_staticline2 = new wxStaticLine(localMetricPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL);
    localMetricInfoSizer->Add(m_staticline2, 0, wxEXPAND | wxALL, 5);

    m_staticline3 = new wxStaticLine(localMetricPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL);
    localMetricInfoSizer->Add(m_staticline3, 0, wxEXPAND | wxALL, 5);

    rightWheelLabel = new wxStaticText(localMetricPanel, wxID_ANY, wxT("Right:"), wxDefaultPosition, wxDefaultSize, 0);
    rightWheelLabel->Wrap(-1);
    rightWheelLabel->SetFont(wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxT("Sans")));

    localMetricInfoSizer->Add(rightWheelLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    rightWheelDisplay =
      new wxStaticText(localMetricPanel, wxID_ANY, wxT("(+0.00, +0.00)"), wxDefaultPosition, wxDefaultSize, 0);
    rightWheelDisplay->Wrap(-1);
    rightWheelDisplay->SetFont(
      wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("Fixed")));

    localMetricInfoSizer->Add(rightWheelDisplay, 0, wxALL, 5);

    imuOrientationLabel =
      new wxStaticText(localMetricPanel, wxID_ANY, wxT("Orientation:"), wxDefaultPosition, wxDefaultSize, 0);
    imuOrientationLabel->Wrap(-1);
    imuOrientationLabel->SetFont(
      wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxT("Sans")));

    localMetricInfoSizer->Add(imuOrientationLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    imuOrientationDisplay =
      new wxStaticText(localMetricPanel, wxID_ANY, wxT("(0.00, 0.00, 0.00)"), wxDefaultPosition, wxDefaultSize, 0);
    imuOrientationDisplay->Wrap(-1);
    imuOrientationDisplay->SetFont(
      wxFont(12, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("Fixed")));

    localMetricInfoSizer->Add(imuOrientationDisplay, 0, wxALL, 5);


    localMetricWidgetSizer->Add(localMetricInfoSizer, 1, wxALIGN_CENTER | wxALL | wxEXPAND, 5);


    localMetricPanelSizer->Add(localMetricWidgetSizer, 1, wxALL | wxEXPAND, 5);

    localMetricScrollWindow =
      new wxScrolledWindow(localMetricPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL);
    localMetricScrollWindow->SetScrollRate(0, 5);
    wxFlexGridSizer* localMetricOptionsSizer;
    localMetricOptionsSizer = new wxFlexGridSizer(0, 1, 0, 0);
    localMetricOptionsSizer->SetFlexibleDirection(wxBOTH);
    localMetricOptionsSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxString localMetricModeBoxChoices[] = {wxT("SLAM"),
                                            wxT("SLAM + Glass"),
                                            wxT("Localization Only"),
                                            wxT("High Resolution")};
    int localMetricModeBoxNChoices = sizeof(localMetricModeBoxChoices) / sizeof(wxString);
    localMetricModeBox = new wxRadioBox(localMetricScrollWindow,
                                        ID_LOCAL_METRIC_MODE_BOX,
                                        wxT("Local Metric Mode"),
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        localMetricModeBoxNChoices,
                                        localMetricModeBoxChoices,
                                        1,
                                        wxRA_SPECIFY_COLS);
    localMetricModeBox->SetSelection(0);
    localMetricOptionsSizer->Add(localMetricModeBox, 0, wxALL | wxEXPAND, 5);

    centerOnRobotCheckBox = new wxCheckBox(localMetricScrollWindow,
                                           ID_CENTER_ON_ROBOT,
                                           wxT("Center On Robot"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    centerOnRobotCheckBox->SetValue(true);
    localMetricOptionsSizer->Add(centerOnRobotCheckBox, 0, wxALL, 5);

    wxString metricGridToShowRadioChoices[] = {wxT("LPM"), wxT("Glass"), wxT("None")};
    int metricGridToShowRadioNChoices = sizeof(metricGridToShowRadioChoices) / sizeof(wxString);
    metricGridToShowRadio = new wxRadioBox(localMetricScrollWindow,
                                           ID_METRIC_GRID_TO_SHOW_RADIO,
                                           wxT("Grid to Show"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           metricGridToShowRadioNChoices,
                                           metricGridToShowRadioChoices,
                                           1,
                                           wxRA_SPECIFY_COLS);
    metricGridToShowRadio->SetSelection(0);
    localMetricOptionsSizer->Add(metricGridToShowRadio, 1, wxALL | wxEXPAND, 5);

    wxStaticBoxSizer* laserDisplayOptionsSizer;
    laserDisplayOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(localMetricScrollWindow, wxID_ANY, wxT("Laser Display Options")),
                           wxVERTICAL);

    wxString laserToShowRadioChoices[] = {wxT("Raw Laser"), wxT("Mapping Laser"), wxT("Reflected Laser"), wxT("None")};
    int laserToShowRadioNChoices = sizeof(laserToShowRadioChoices) / sizeof(wxString);
    laserToShowRadio = new wxRadioBox(laserDisplayOptionsSizer->GetStaticBox(),
                                      ID_LASER_TO_SHOW_RADIO,
                                      wxT("Laser To Show:"),
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      laserToShowRadioNChoices,
                                      laserToShowRadioChoices,
                                      1,
                                      wxRA_SPECIFY_COLS);
    laserToShowRadio->SetSelection(0);
    laserDisplayOptionsSizer->Add(laserToShowRadio, 0, wxALL | wxEXPAND, 5);

    scanLineCheckBox = new wxCheckBox(laserDisplayOptionsSizer->GetStaticBox(),
                                      ID_SHOW_LASER_LINES_BOX,
                                      wxT("Show Laser Rays"),
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      0);
    laserDisplayOptionsSizer->Add(scanLineCheckBox, 0, wxALL, 5);

    showExtractedLinesCheckBox = new wxCheckBox(laserDisplayOptionsSizer->GetStaticBox(),
                                                ID_SHOW_EXTRACTED_LINES_BOX,
                                                wxT("Show Extracted Lines"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    laserDisplayOptionsSizer->Add(showExtractedLinesCheckBox, 0, wxALL, 5);

    showIntensityPlotsCheckBox = new wxCheckBox(laserDisplayOptionsSizer->GetStaticBox(),
                                                ID_SHOW_INTENSITY_PLOTS_BOX,
                                                wxT("Show intensity Plots"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    laserDisplayOptionsSizer->Add(showIntensityPlotsCheckBox, 0, wxALL | wxEXPAND, 5);


    localMetricOptionsSizer->Add(laserDisplayOptionsSizer, 1, wxEXPAND, 5);

    wxStaticBoxSizer* localizationOptionsSizer;
    localizationOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(localMetricScrollWindow, wxID_ANY, wxT("Localization Display Options")),
                           wxVERTICAL);

    showPoseTraceCheckBox = new wxCheckBox(localizationOptionsSizer->GetStaticBox(),
                                           ID_SHOW_POSE_TRACE,
                                           wxT("Show Pose Trace"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    showPoseTraceCheckBox->SetValue(true);
    localizationOptionsSizer->Add(showPoseTraceCheckBox, 0, wxALL, 5);

    showMotionTraceCheckBox = new wxCheckBox(localizationOptionsSizer->GetStaticBox(),
                                             ID_SHOW_MOTION_TRACE_BOX,
                                             wxT("Show Motion Trace"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    showMotionTraceCheckBox->SetValue(true);
    localizationOptionsSizer->Add(showMotionTraceCheckBox, 0, wxALL, 5);

    showUncertaintyEllipseCheckBox = new wxCheckBox(localizationOptionsSizer->GetStaticBox(),
                                                    ID_SHOW_UNCERTAINTY_ELLIPSE,
                                                    wxT("Show Error Ellipse"),
                                                    wxDefaultPosition,
                                                    wxDefaultSize,
                                                    0);
    showUncertaintyEllipseCheckBox->SetValue(true);
    localizationOptionsSizer->Add(showUncertaintyEllipseCheckBox, 0, wxALL, 5);

    showParticlesCheckBox = new wxCheckBox(localizationOptionsSizer->GetStaticBox(),
                                           ID_SHOW_PARTICLES,
                                           wxT("Show Particles"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    showParticlesCheckBox->SetValue(true);
    localizationOptionsSizer->Add(showParticlesCheckBox, 0, wxALL, 5);

    wxBoxSizer* cleanButtonsSizer;
    cleanButtonsSizer = new wxBoxSizer(wxHORIZONTAL);

    clearPosesButton = new wxButton(localizationOptionsSizer->GetStaticBox(),
                                    ID_CLEAR_POSES_BUTTON,
                                    wxT("Clear Poses"),
                                    wxDefaultPosition,
                                    wxDefaultSize,
                                    0);
    cleanButtonsSizer->Add(clearPosesButton, 0, wxALIGN_CENTER | wxALL, 5);

    clearMotionButton = new wxButton(localizationOptionsSizer->GetStaticBox(),
                                     ID_CLEAR_MOTION_BUTTON,
                                     wxT("Clear Motion"),
                                     wxDefaultPosition,
                                     wxDefaultSize,
                                     0);
    cleanButtonsSizer->Add(clearMotionButton, 0, wxALL, 5);


    localizationOptionsSizer->Add(cleanButtonsSizer, 1, wxEXPAND, 5);


    localMetricOptionsSizer->Add(localizationOptionsSizer, 1, wxALIGN_RIGHT | wxEXPAND, 5);

    wxStaticBoxSizer* glassMapOptionsSizer;
    glassMapOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(localMetricScrollWindow, wxID_ANY, wxT("Glass Map Options")), wxVERTICAL);

    showGlassIntensityCheckBox = new wxCheckBox(glassMapOptionsSizer->GetStaticBox(),
                                                ID_SHOW_GLASS_INTENSITY_BOX,
                                                wxT("Show Glass Intensity"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    glassMapOptionsSizer->Add(showGlassIntensityCheckBox, 0, wxALL, 5);

    showGlassWallsCheckBox = new wxCheckBox(glassMapOptionsSizer->GetStaticBox(),
                                            ID_SHOW_GLASS_WALLS_BOX,
                                            wxT("Show Glass Walls"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    glassMapOptionsSizer->Add(showGlassWallsCheckBox, 0, wxALL, 5);

    showGlassAnglesCheckBox = new wxCheckBox(glassMapOptionsSizer->GetStaticBox(),
                                             ID_SHOW_GLASS_ANGLES_BOX,
                                             wxT("Show Glass Angles"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    glassMapOptionsSizer->Add(showGlassAnglesCheckBox, 0, wxALL, 5);

    wxString glassAnglesToShowRadioChoices[] = {wxT("Normal"), wxT("Range")};
    int glassAnglesToShowRadioNChoices = sizeof(glassAnglesToShowRadioChoices) / sizeof(wxString);
    glassAnglesToShowRadio = new wxRadioBox(glassMapOptionsSizer->GetStaticBox(),
                                            ID_GLASS_ANGLES_TO_SHOW_RADIO,
                                            wxT("Angles to Show"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            glassAnglesToShowRadioNChoices,
                                            glassAnglesToShowRadioChoices,
                                            1,
                                            wxRA_SPECIFY_COLS);
    glassAnglesToShowRadio->SetSelection(0);
    glassMapOptionsSizer->Add(glassAnglesToShowRadio, 0, wxALL | wxEXPAND, 5);

    wxFlexGridSizer* glassSidersSizer;
    glassSidersSizer = new wxFlexGridSizer(2, 2, 0, 0);
    glassSidersSizer->AddGrowableCol(1);
    glassSidersSizer->SetFlexibleDirection(wxHORIZONTAL);
    glassSidersSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    flattenThreshLabel = new wxStaticText(glassMapOptionsSizer->GetStaticBox(),
                                          wxID_ANY,
                                          wxT("Flatten:"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    flattenThreshLabel->Wrap(-1);
    glassSidersSizer->Add(flattenThreshLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    glassFlattenThreshSlider = new wxSlider(glassMapOptionsSizer->GetStaticBox(),
                                            wxID_ANY,
                                            2,
                                            1,
                                            25,
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            wxSL_AUTOTICKS | wxSL_HORIZONTAL | wxSL_LABELS);
    glassSidersSizer->Add(glassFlattenThreshSlider, 1, wxALL | wxEXPAND, 5);

    highlyVisibleGlassThreshLabel = new wxStaticText(glassMapOptionsSizer->GetStaticBox(),
                                                     wxID_ANY,
                                                     wxT("Highly-visible:"),
                                                     wxDefaultPosition,
                                                     wxDefaultSize,
                                                     0);
    highlyVisibleGlassThreshLabel->Wrap(-1);
    glassSidersSizer->Add(highlyVisibleGlassThreshLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    glassHighlyVisibleThreshSlider = new wxSlider(glassMapOptionsSizer->GetStaticBox(),
                                                  wxID_ANY,
                                                  10,
                                                  5,
                                                  100,
                                                  wxDefaultPosition,
                                                  wxDefaultSize,
                                                  wxSL_AUTOTICKS | wxSL_HORIZONTAL | wxSL_LABELS);
    glassSidersSizer->Add(glassHighlyVisibleThreshSlider, 1, wxALL | wxEXPAND, 5);


    glassMapOptionsSizer->Add(glassSidersSizer, 1, wxALIGN_CENTER | wxALL | wxEXPAND, 5);

    runFlattenMapButton = new wxButton(glassMapOptionsSizer->GetStaticBox(),
                                       ID_RUN_FLATTEN_MAP_BUTTON,
                                       wxT("Run Flatten Map"),
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0);
    glassMapOptionsSizer->Add(runFlattenMapButton, 0, wxALL | wxEXPAND, 5);

    runDynamicFilterButton = new wxButton(glassMapOptionsSizer->GetStaticBox(),
                                          ID_RUN_DYNAMIC_FILTER_BUTTON,
                                          wxT("Run Dynamic Filter"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    glassMapOptionsSizer->Add(runDynamicFilterButton, 0, wxALL | wxEXPAND, 5);


    localMetricOptionsSizer->Add(glassMapOptionsSizer, 1, wxEXPAND, 5);

    wxStaticBoxSizer* rotateSizer;
    rotateSizer =
      new wxStaticBoxSizer(new wxStaticBox(localMetricScrollWindow, wxID_ANY, wxT("Rotation Test (Degrees)")),
                           wxHORIZONTAL);

    rotateLPMText =
      new wxTextCtrl(rotateSizer->GetStaticBox(), wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
#ifdef __WXGTK__
    if (!rotateLPMText->HasFlag(wxTE_MULTILINE)) {
        rotateLPMText->SetMaxLength(5);
    }
#else
    rotateLPMText->SetMaxLength(5);
#endif
    rotateSizer->Add(rotateLPMText, 0, wxALIGN_CENTER | wxALL, 5);

    rotateLPMButton = new wxButton(rotateSizer->GetStaticBox(),
                                   ID_ROTATE_LPM_BUTTON,
                                   wxT("Rotate"),
                                   wxDefaultPosition,
                                   wxDefaultSize,
                                   0);
    rotateSizer->Add(rotateLPMButton, 0, wxALL, 5);


    localMetricOptionsSizer->Add(rotateSizer, 1, wxEXPAND, 5);

    saveCurrentLPMButton = new wxButton(localMetricScrollWindow,
                                        ID_SAVE_CURRENT_LPM_BUTTON,
                                        wxT("Save Current LPM"),
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        0);
    saveCurrentLPMButton->SetToolTip(
      wxT("Save the current LPM being displayed as \"current.lpm\" in the current working directory."));

    localMetricOptionsSizer->Add(saveCurrentLPMButton, 0, wxALL | wxEXPAND, 5);

    saveGlassMapButton = new wxButton(localMetricScrollWindow,
                                      ID_SAVE_GLASS_MAP_BUTTON,
                                      wxT("Save Glass Map"),
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      0);
    saveGlassMapButton->SetToolTip(wxT("Tell local_metric_hssh to save the current glass map as \"current_glass.map\" "
                                       "in the working directory of local_metric_hssh."));

    localMetricOptionsSizer->Add(saveGlassMapButton, 0, wxALL | wxEXPAND, 5);

    loadGlassMapButton = new wxButton(localMetricScrollWindow,
                                      ID_LOAD_GLASS_MAP_BUTTON,
                                      wxT("Load Glass Map"),
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      0);
    localMetricOptionsSizer->Add(loadGlassMapButton, 0, wxALL | wxEXPAND, 5);

    savePosesButton = new wxButton(localMetricScrollWindow,
                                   ID_SAVE_POSES_BUTTON,
                                   wxT("Save Glass Poses"),
                                   wxDefaultPosition,
                                   wxDefaultSize,
                                   0);
    savePosesButton->SetToolTip(wxT(
      "Tell local_metric_hssh to save all poses from which the map is updated. Needed for glass mapping evaluation."));

    localMetricOptionsSizer->Add(savePosesButton, 0, wxALL | wxEXPAND, 5);

    saveScansButton = new wxButton(localMetricScrollWindow,
                                   ID_SAVE_SCANS_BUTTON,
                                   wxT("Save Glass Scans"),
                                   wxDefaultPosition,
                                   wxDefaultSize,
                                   0);
    saveScansButton->SetToolTip(wxT("Tell local_metric_hssh to save every scan it uses to update the map. Needed for "
                                    "helping with the glass mapping evaluation."));

    localMetricOptionsSizer->Add(saveScansButton, 0, wxALL | wxEXPAND, 5);


    localMetricScrollWindow->SetSizer(localMetricOptionsSizer);
    localMetricScrollWindow->Layout();
    localMetricOptionsSizer->Fit(localMetricScrollWindow);
    localMetricPanelSizer->Add(localMetricScrollWindow, 0, wxALL | wxEXPAND, 5);


    localMetricPanel->SetSizer(localMetricPanelSizer);
    localMetricPanel->Layout();
    localMetricPanelSizer->Fit(localMetricPanel);
    frameNotebook->AddPage(localMetricPanel, wxT("Local Metric"), false, wxNullBitmap);
    metricPlannerPanel = new wxPanel(frameNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxBoxSizer* metricPlannerSizer;
    metricPlannerSizer = new wxBoxSizer(wxHORIZONTAL);

    wxBoxSizer* metricPlannerWidgetSizer;
    metricPlannerWidgetSizer = new wxBoxSizer(wxVERTICAL);

    metricPlannerWidget =
      new MetricPlannerDisplayWidget(metricPlannerPanel, ID_METRIC_PLANNER_WIDGET, wxDefaultPosition, wxDefaultSize);
    metricPlannerWidget->SetSize(wxSize(800, 600));
    metricPlannerWidget->SetMinSize(wxSize(0, 0));

    metricPlannerWidgetSizer->Add(metricPlannerWidget, 1, wxALIGN_CENTER | wxEXPAND, 0);


    metricPlannerSizer->Add(metricPlannerWidgetSizer, 1, wxALL | wxEXPAND, 5);

    metricPlannerScrollWindow =
      new wxScrolledWindow(metricPlannerPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL);
    metricPlannerScrollWindow->SetScrollRate(0, 5);
    wxFlexGridSizer* metricPlannerOptionsSizer;
    metricPlannerOptionsSizer = new wxFlexGridSizer(0, 1, 0, 0);
    metricPlannerOptionsSizer->SetFlexibleDirection(wxBOTH);
    metricPlannerOptionsSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* mainDisplayOptionsSizer;
    mainDisplayOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(metricPlannerScrollWindow, wxID_ANY, wxT("Main Display Options")),
                           wxVERTICAL);

    wxFlexGridSizer* displayOptionsGridSizer;
    displayOptionsGridSizer = new wxFlexGridSizer(0, 2, 0, 0);
    displayOptionsGridSizer->SetFlexibleDirection(wxBOTH);
    displayOptionsGridSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    showRobotCheckBox = new wxCheckBox(mainDisplayOptionsSizer->GetStaticBox(),
                                       ID_SHOW_ROBOT_POSE_CHECKBOX,
                                       wxT("Robot"),
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0);
    showRobotCheckBox->SetValue(true);
    displayOptionsGridSizer->Add(showRobotCheckBox, 0, wxALL, 5);

    showTrackedObjectsCheckBox = new wxCheckBox(mainDisplayOptionsSizer->GetStaticBox(),
                                                ID_SHOW_TRACKED_OBJECTS_CHECKBOX,
                                                wxT("Tracked Objects"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    showTrackedObjectsCheckBox->SetValue(true);
    displayOptionsGridSizer->Add(showTrackedObjectsCheckBox, 0, wxALL, 5);

    showDestinationPoseCheckBox = new wxCheckBox(mainDisplayOptionsSizer->GetStaticBox(),
                                                 ID_SHOW_DESTINATION_POSE_CHECKBOX,
                                                 wxT("Destination"),
                                                 wxDefaultPosition,
                                                 wxDefaultSize,
                                                 0);
    showDestinationPoseCheckBox->SetValue(true);
    displayOptionsGridSizer->Add(showDestinationPoseCheckBox, 0, wxALL, 5);

    showObjectsMotionCheckBox = new wxCheckBox(mainDisplayOptionsSizer->GetStaticBox(),
                                               ID_SHOW_OBJECTS_MOTION_CHECKBOX,
                                               wxT("Estimated Objects Motion"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    showObjectsMotionCheckBox->SetValue(true);
    displayOptionsGridSizer->Add(showObjectsMotionCheckBox, 0, wxALL, 5);

    showOptimalPathCheckBox = new wxCheckBox(mainDisplayOptionsSizer->GetStaticBox(),
                                             ID_SHOW_OPTIMAL_PATH_CHECKBOX,
                                             wxT("Optimal Path"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    displayOptionsGridSizer->Add(showOptimalPathCheckBox, 0, wxALL | wxEXPAND, 5);

    showVisibilityAnalysisCheckBox = new wxCheckBox(mainDisplayOptionsSizer->GetStaticBox(),
                                                    ID_SHOW_VISIBILITY_ANALYSIS_CHECKBOX,
                                                    wxT("Visibility Analysis"),
                                                    wxDefaultPosition,
                                                    wxDefaultSize,
                                                    0);
    displayOptionsGridSizer->Add(showVisibilityAnalysisCheckBox, 0, wxALL, 5);

    showSituationsCheckBox = new wxCheckBox(mainDisplayOptionsSizer->GetStaticBox(),
                                            ID_SHOW_SITUATIONS_CHECKBOX,
                                            wxT("Situations"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    displayOptionsGridSizer->Add(showSituationsCheckBox, 0, wxALL | wxEXPAND, 5);


    mainDisplayOptionsSizer->Add(displayOptionsGridSizer, 1, wxEXPAND, 5);


    metricPlannerOptionsSizer->Add(mainDisplayOptionsSizer, 1, wxEXPAND, 5);

    wxStaticBoxSizer* destinationPoseSizer;
    destinationPoseSizer =
      new wxStaticBoxSizer(new wxStaticBox(metricPlannerScrollWindow, wxID_ANY, wxT("Destination Pose")), wxHORIZONTAL);

    selectDestinationPoseButton = new wxButton(destinationPoseSizer->GetStaticBox(),
                                               ID_SELECT_DESTINATION_POSE_BUTTON,
                                               wxT("Select"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    destinationPoseSizer->Add(selectDestinationPoseButton, 0, wxALIGN_CENTER | wxALL, 5);

    sendDestinationPoseButton = new wxButton(destinationPoseSizer->GetStaticBox(),
                                             ID_SEND_DESTINATION_POSE_BUTTON,
                                             wxT("Send"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    destinationPoseSizer->Add(sendDestinationPoseButton, 0, wxALIGN_CENTER | wxALL, 5);

    cancelDestinationPoseButton = new wxButton(destinationPoseSizer->GetStaticBox(),
                                               ID_CANCEL_DESTINATION_POSE_BUTTON,
                                               wxT("Cancel"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    destinationPoseSizer->Add(cancelDestinationPoseButton, 0, wxALIGN_CENTER | wxALL, 5);


    metricPlannerOptionsSizer->Add(destinationPoseSizer, 0, wxEXPAND, 5);

    wxStaticBoxSizer* metricScriptOptionsSizer;
    metricScriptOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(metricPlannerScrollWindow, wxID_ANY, wxT("Script Options")), wxVERTICAL);

    wxGridSizer* metricScriptSelectionSizer;
    metricScriptSelectionSizer = new wxGridSizer(0, 2, 0, 0);

    metricScriptFileText = new wxTextCtrl(metricScriptOptionsSizer->GetStaticBox(),
                                          wxID_ANY,
                                          wxT("Enter script name"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    metricScriptSelectionSizer->Add(metricScriptFileText, 1, wxALL | wxEXPAND, 5);

    metricLoadScriptButton = new wxButton(metricScriptOptionsSizer->GetStaticBox(),
                                          ID_METRIC_LOAD_SCRIPT_BUTTON,
                                          wxT("Load Script"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    metricScriptSelectionSizer->Add(metricLoadScriptButton, 0, wxALL | wxEXPAND, 5);

    sendWaypointsButton = new wxButton(metricScriptOptionsSizer->GetStaticBox(),
                                       ID_SEND_WAYPOINTS_BUTTON,
                                       wxT("Send"),
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0);
    metricScriptSelectionSizer->Add(sendWaypointsButton, 0, wxALIGN_CENTER | wxALL | wxEXPAND, 5);

    skipWaypointButton = new wxButton(metricScriptOptionsSizer->GetStaticBox(),
                                      ID_SKIP_WAYPOINT_BUTTON,
                                      wxT("Skip"),
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      0);
    metricScriptSelectionSizer->Add(skipWaypointButton, 0, wxALL | wxEXPAND, 5);

    cancelFollowingButton = new wxButton(metricScriptOptionsSizer->GetStaticBox(),
                                         ID_CANCEL_FOLLOWING_BUTTON,
                                         wxT("Stop/Cancel"),
                                         wxDefaultPosition,
                                         wxDefaultSize,
                                         0);
    metricScriptSelectionSizer->Add(cancelFollowingButton, 0, wxALL | wxEXPAND, 5);

    loopWaypointsButton = new wxButton(metricScriptOptionsSizer->GetStaticBox(),
                                       ID_LOOP_WAYPOINTS_BUTTON,
                                       wxT("Loop"),
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0);
    metricScriptSelectionSizer->Add(loopWaypointsButton, 0, wxALL | wxEXPAND, 5);


    metricScriptOptionsSizer->Add(metricScriptSelectionSizer, 1, wxEXPAND, 5);


    metricPlannerOptionsSizer->Add(metricScriptOptionsSizer, 0, wxEXPAND, 5);

    wxStaticBoxSizer* mpepcOptionsSizer;
    mpepcOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(metricPlannerScrollWindow, wxID_ANY, wxT("MPEPC Options")), wxVERTICAL);

    wxGridBagSizer* mpepcGridBag;
    mpepcGridBag = new wxGridBagSizer(0, 0);
    mpepcGridBag->SetFlexibleDirection(wxBOTH);
    mpepcGridBag->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxString selectMapTypeRadioBoxChoices[] = {wxT("LPM"),
                                               wxT("Obstacle Distance"),
                                               wxT("Cost Map"),
                                               wxT("Navigation Function"),
                                               wxT("Flow Grid"),
                                               wxT("None")};
    int selectMapTypeRadioBoxNChoices = sizeof(selectMapTypeRadioBoxChoices) / sizeof(wxString);
    selectMapTypeRadioBox = new wxRadioBox(mpepcOptionsSizer->GetStaticBox(),
                                           ID_SELECT_MAP_TYPE_RADIOBOX,
                                           wxT("Map Type"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           selectMapTypeRadioBoxNChoices,
                                           selectMapTypeRadioBoxChoices,
                                           1,
                                           wxRA_SPECIFY_COLS);
    selectMapTypeRadioBox->SetSelection(0);
    mpepcGridBag->Add(selectMapTypeRadioBox, wxGBPosition(0, 0), wxGBSpan(1, 1), wxALL | wxEXPAND, 5);

    wxString selectTrjGroupRadioBoxChoices[] =
      {wxT("Current"), wxT("Optimal"), wxT("History"), wxT("None"), wxT("Last 3 Sec"), wxT("Last 5 Sec")};
    int selectTrjGroupRadioBoxNChoices = sizeof(selectTrjGroupRadioBoxChoices) / sizeof(wxString);
    selectTrjGroupRadioBox = new wxRadioBox(mpepcOptionsSizer->GetStaticBox(),
                                            ID_SELECT_TRJ_GROUP_RADIOBOX,
                                            wxT("Trajectory Type"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            selectTrjGroupRadioBoxNChoices,
                                            selectTrjGroupRadioBoxChoices,
                                            1,
                                            wxRA_SPECIFY_COLS);
    selectTrjGroupRadioBox->SetSelection(0);
    mpepcGridBag->Add(selectTrjGroupRadioBox, wxGBPosition(0, 1), wxGBSpan(1, 2), wxALL | wxEXPAND, 5);

    trjCostChoiceLabel = new wxStaticText(mpepcOptionsSizer->GetStaticBox(),
                                          wxID_ANY,
                                          wxT("Cost Type Displayed:"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    trjCostChoiceLabel->Wrap(-1);
    mpepcGridBag->Add(trjCostChoiceLabel,
                      wxGBPosition(1, 0),
                      wxGBSpan(1, 1),
                      wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL,
                      5);

    wxArrayString trajectoryCostChoiceChoices;
    trajectoryCostChoice = new wxChoice(mpepcOptionsSizer->GetStaticBox(),
                                        ID_SELECT_TRJ_COST_CHOICE,
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        trajectoryCostChoiceChoices,
                                        0);
    trajectoryCostChoice->SetSelection(0);
    mpepcGridBag->Add(trajectoryCostChoice, wxGBPosition(1, 1), wxGBSpan(1, 2), wxALL | wxEXPAND, 5);

    wxString trjDisplayModeRadioBoxChoices[] = {wxT("All"), wxT("Last N"), wxT("Single")};
    int trjDisplayModeRadioBoxNChoices = sizeof(trjDisplayModeRadioBoxChoices) / sizeof(wxString);
    trjDisplayModeRadioBox = new wxRadioBox(mpepcOptionsSizer->GetStaticBox(),
                                            ID_SELECT_TRJ_DISPLAY_MODE_RADIOBOX,
                                            wxT("Trajectories Displayed"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            trjDisplayModeRadioBoxNChoices,
                                            trjDisplayModeRadioBoxChoices,
                                            1,
                                            wxRA_SPECIFY_COLS);
    trjDisplayModeRadioBox->SetSelection(0);
    mpepcGridBag->Add(trjDisplayModeRadioBox, wxGBPosition(2, 0), wxGBSpan(3, 1), wxALL, 5);

    trajectoryCountLabel = new wxStaticText(mpepcOptionsSizer->GetStaticBox(),
                                            wxID_ANY,
                                            wxT("Total Count (Current): 0"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    trajectoryCountLabel->Wrap(-1);
    mpepcGridBag->Add(trajectoryCountLabel, wxGBPosition(2, 1), wxGBSpan(1, 2), wxALIGN_LEFT | wxALL, 5);

    numTrajectoriesText = new wxTextCtrl(mpepcOptionsSizer->GetStaticBox(),
                                         ID_CHANGE_TRJ_NUM_TEXT,
                                         wxEmptyString,
                                         wxDefaultPosition,
                                         wxDefaultSize,
                                         wxTE_PROCESS_ENTER | wxTE_RIGHT);
#ifdef __WXGTK__
    if (!numTrajectoriesText->HasFlag(wxTE_MULTILINE)) {
        numTrajectoriesText->SetMaxLength(5);
    }
#else
    numTrajectoriesText->SetMaxLength(5);
#endif
    mpepcGridBag->Add(numTrajectoriesText, wxGBPosition(3, 1), wxGBSpan(1, 2), wxALIGN_CENTER | wxALL | wxEXPAND, 5);

    decreaseTrajectoriesButton = new wxButton(mpepcOptionsSizer->GetStaticBox(),
                                              ID_DECREASE_TRJ_NUM_BUTTON,
                                              wxT("<"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              wxBU_EXACTFIT);
    mpepcGridBag->Add(decreaseTrajectoriesButton, wxGBPosition(4, 1), wxGBSpan(1, 1), wxALL, 5);

    increaseTrajectoriesButton = new wxButton(mpepcOptionsSizer->GetStaticBox(),
                                              ID_INCREASE_TRJ_NUM_BUTTON,
                                              wxT(">"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              wxBU_EXACTFIT);
    mpepcGridBag->Add(increaseTrajectoriesButton, wxGBPosition(4, 2), wxGBSpan(1, 1), wxALIGN_RIGHT | wxALL, 5);

    wxStaticBoxSizer* trajectoryDisplayOptionsSizer;
    trajectoryDisplayOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(mpepcOptionsSizer->GetStaticBox(), wxID_ANY, wxT("Display Options")),
                           wxVERTICAL);

    showMotionTargetCheckBox = new wxCheckBox(trajectoryDisplayOptionsSizer->GetStaticBox(),
                                              ID_SHOW_MOTION_TARGETS_CHECKBOX,
                                              wxT("Motion Targets"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    trajectoryDisplayOptionsSizer->Add(showMotionTargetCheckBox, 0, wxALL, 5);

    overlayRobotPosesCheckBox = new wxCheckBox(trajectoryDisplayOptionsSizer->GetStaticBox(),
                                               ID_OVERLAY_ROBOT_POSES_CHECKBOX,
                                               wxT("Overlay Pose"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    trajectoryDisplayOptionsSizer->Add(overlayRobotPosesCheckBox, 0, wxALL, 5);


    mpepcGridBag->Add(trajectoryDisplayOptionsSizer, wxGBPosition(5, 0), wxGBSpan(2, 1), wxALL | wxEXPAND, 5);

    evaluatedCostLabel = new wxStaticText(mpepcOptionsSizer->GetStaticBox(),
                                          wxID_ANY,
                                          wxT("Evaluated Cost (Single)"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    evaluatedCostLabel->Wrap(-1);
    mpepcGridBag->Add(evaluatedCostLabel,
                      wxGBPosition(5, 1),
                      wxGBSpan(1, 2),
                      wxALIGN_BOTTOM | wxALIGN_CENTER | wxALL,
                      5);

    evaluatedCostText =
      new wxStaticText(mpepcOptionsSizer->GetStaticBox(), wxID_ANY, wxT("0"), wxDefaultPosition, wxDefaultSize, 0);
    evaluatedCostText->Wrap(-1);
    mpepcGridBag->Add(evaluatedCostText, wxGBPosition(6, 1), wxGBSpan(1, 2), wxALIGN_CENTER | wxALIGN_TOP | wxALL, 5);


    mpepcOptionsSizer->Add(mpepcGridBag, 1, wxALIGN_RIGHT | wxEXPAND, 5);


    metricPlannerOptionsSizer->Add(mpepcOptionsSizer, 1, wxALL | wxEXPAND, 5);

    wxStaticBoxSizer* simulationTargetOptionsSizer;
    simulationTargetOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(metricPlannerScrollWindow, wxID_ANY, wxT("Simultation Target Options")),
                           wxVERTICAL);

    wxFlexGridSizer* motionTargetCoordsSizer;
    motionTargetCoordsSizer = new wxFlexGridSizer(0, 0, 0, 0);
    motionTargetCoordsSizer->SetFlexibleDirection(wxBOTH);
    motionTargetCoordsSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    motionTargetRLabel = new wxStaticText(simulationTargetOptionsSizer->GetStaticBox(),
                                          wxID_ANY,
                                          wxT(" r:"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    motionTargetRLabel->Wrap(-1);
    motionTargetCoordsSizer->Add(motionTargetRLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 0);

    motionTargetRText = new wxTextCtrl(simulationTargetOptionsSizer->GetStaticBox(),
                                       ID_MOTION_TARGET_R_TEXT,
                                       wxEmptyString,
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0);
    motionTargetRText->SetMaxSize(wxSize(80, -1));

    motionTargetCoordsSizer->Add(motionTargetRText, 0, wxALL, 5);

    motionTargetThetaLabel = new wxStaticText(simulationTargetOptionsSizer->GetStaticBox(),
                                              wxID_ANY,
                                              wxT("θ:   "),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    motionTargetThetaLabel->Wrap(-1);
    motionTargetCoordsSizer->Add(motionTargetThetaLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 0);

    motionTargetThetaText = new wxTextCtrl(simulationTargetOptionsSizer->GetStaticBox(),
                                           ID_MOTION_TARGET_THETA_TEXT,
                                           wxEmptyString,
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    motionTargetThetaText->SetMaxSize(wxSize(80, -1));

    motionTargetCoordsSizer->Add(motionTargetThetaText, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    motionTargetDeltaLabel = new wxStaticText(simulationTargetOptionsSizer->GetStaticBox(),
                                              wxID_ANY,
                                              wxT("δ:  "),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    motionTargetDeltaLabel->Wrap(-1);
    motionTargetCoordsSizer->Add(motionTargetDeltaLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 0);

    motionTargetDeltaText = new wxTextCtrl(simulationTargetOptionsSizer->GetStaticBox(),
                                           ID_MOTION_TARGET_DELTA_TEXT,
                                           wxEmptyString,
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    motionTargetDeltaText->SetMaxSize(wxSize(80, -1));

    motionTargetCoordsSizer->Add(motionTargetDeltaText, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);


    simulationTargetOptionsSizer->Add(motionTargetCoordsSizer, 0, wxEXPAND, 5);

    wxFlexGridSizer* motionTargetGainsSizer;
    motionTargetGainsSizer = new wxFlexGridSizer(0, 0, 0, 0);
    motionTargetGainsSizer->SetFlexibleDirection(wxBOTH);
    motionTargetGainsSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    motionTargetGainLabel = new wxStaticText(simulationTargetOptionsSizer->GetStaticBox(),
                                             wxID_ANY,
                                             wxT(" v:"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    motionTargetGainLabel->Wrap(-1);
    motionTargetGainsSizer->Add(motionTargetGainLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 0);

    motionTargetGainText = new wxTextCtrl(simulationTargetOptionsSizer->GetStaticBox(),
                                          ID_MOTION_TARGET_GAIN_TEXT,
                                          wxEmptyString,
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    motionTargetGainText->SetMaxSize(wxSize(80, -1));

    motionTargetGainsSizer->Add(motionTargetGainText, 0, wxALL, 5);

    motionTargetK1Label = new wxStaticText(simulationTargetOptionsSizer->GetStaticBox(),
                                           wxID_ANY,
                                           wxT("k1:"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    motionTargetK1Label->Wrap(-1);
    motionTargetGainsSizer->Add(motionTargetK1Label, 0, wxALIGN_CENTER_VERTICAL | wxALL, 0);

    motionTargetK1Text = new wxTextCtrl(simulationTargetOptionsSizer->GetStaticBox(),
                                        ID_MOTION_TARGET_K1_TEXT,
                                        wxEmptyString,
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        0);
    motionTargetK1Text->SetMaxSize(wxSize(80, -1));

    motionTargetGainsSizer->Add(motionTargetK1Text, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    motionTargetK2Label = new wxStaticText(simulationTargetOptionsSizer->GetStaticBox(),
                                           wxID_ANY,
                                           wxT("k2:"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    motionTargetK2Label->Wrap(-1);
    motionTargetGainsSizer->Add(motionTargetK2Label, 0, wxALIGN_CENTER_VERTICAL | wxALL, 0);

    motionTargetK2Text = new wxTextCtrl(simulationTargetOptionsSizer->GetStaticBox(),
                                        ID_MOTION_TARGET_K2_TEXT,
                                        wxEmptyString,
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        0);
    motionTargetK2Text->SetMaxSize(wxSize(80, -1));

    motionTargetGainsSizer->Add(motionTargetK2Text, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);


    simulationTargetOptionsSizer->Add(motionTargetGainsSizer, 0, wxEXPAND, 5);

    motionTargetCostText = new wxStaticText(simulationTargetOptionsSizer->GetStaticBox(),
                                            wxID_ANY,
                                            wxT("Evaluated Expected Cost: 0"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    motionTargetCostText->Wrap(-1);
    simulationTargetOptionsSizer->Add(motionTargetCostText, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 5);

    wxStaticBoxSizer* motionTargetSizer;
    motionTargetSizer = new wxStaticBoxSizer(
      new wxStaticBox(simulationTargetOptionsSizer->GetStaticBox(), wxID_ANY, wxT("Evaluate Motion Target")),
      wxHORIZONTAL);

    selectMotionTargetButton = new wxButton(motionTargetSizer->GetStaticBox(),
                                            ID_SELECT_MOTION_TARGET_BUTTON,
                                            wxT("Select"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    motionTargetSizer->Add(selectMotionTargetButton, 0, wxALIGN_CENTER | wxALL, 5);

    evaluateMotionTargetButton = new wxButton(motionTargetSizer->GetStaticBox(),
                                              ID_EVALUATE_MOTION_TARGET_BUTTON,
                                              wxT("Evaluate"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    motionTargetSizer->Add(evaluateMotionTargetButton, 0, wxALIGN_CENTER | wxALL, 5);

    clearMotionTargetButton = new wxButton(motionTargetSizer->GetStaticBox(),
                                           ID_CLEAR_MOTION_TARGET_BUTTON,
                                           wxT("Clear"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    motionTargetSizer->Add(clearMotionTargetButton, 0, wxALIGN_CENTER | wxALL, 5);


    simulationTargetOptionsSizer->Add(motionTargetSizer, 1, wxEXPAND, 5);


    metricPlannerOptionsSizer->Add(simulationTargetOptionsSizer, 1, wxEXPAND, 5);

    wxStaticBoxSizer* pathFollowingOptionsSizer;
    pathFollowingOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(metricPlannerScrollWindow, wxID_ANY, wxT("Path Following Options")),
                           wxVERTICAL);

    wxFlexGridSizer* pathFollowingDisplayOptionsSizer;
    pathFollowingDisplayOptionsSizer = new wxFlexGridSizer(3, 2, 0, 0);
    pathFollowingDisplayOptionsSizer->SetFlexibleDirection(wxBOTH);
    pathFollowingDisplayOptionsSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    showWaypointsCheckBox = new wxCheckBox(pathFollowingOptionsSizer->GetStaticBox(),
                                           ID_SHOW_WAYPOINTS_BOX,
                                           wxT("Show Waypoints"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    showWaypointsCheckBox->SetValue(true);
    pathFollowingDisplayOptionsSizer->Add(showWaypointsCheckBox, 0, wxALL, 5);

    useRRTStarCheckBOx = new wxCheckBox(pathFollowingOptionsSizer->GetStaticBox(),
                                        ID_USE_RRT_STAR_BOX,
                                        wxT("Use RRT*"),
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        0);
    pathFollowingDisplayOptionsSizer->Add(useRRTStarCheckBOx, 0, wxALL, 5);

    stopAtWaypointsCheckBox = new wxCheckBox(pathFollowingOptionsSizer->GetStaticBox(),
                                             ID_STOP_AT_WAYPOINTS_BOX,
                                             wxT("Stop at Waypoint"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    stopAtWaypointsCheckBox->SetValue(true);
    pathFollowingDisplayOptionsSizer->Add(stopAtWaypointsCheckBox, 0, wxALL, 5);

    showGraphCheckBox = new wxCheckBox(pathFollowingOptionsSizer->GetStaticBox(),
                                       ID_SHOW_GRAPH_BOX,
                                       wxT("Show Graph"),
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0);
    pathFollowingDisplayOptionsSizer->Add(showGraphCheckBox, 0, wxALL, 5);

    SimpleFollowingCheckBox = new wxCheckBox(pathFollowingOptionsSizer->GetStaticBox(),
                                             ID_SIMPLE_FOLLOWING_BOX,
                                             wxT("Simple Following"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    pathFollowingDisplayOptionsSizer->Add(SimpleFollowingCheckBox, 0, wxALL, 5);

    ignoreObjectSpeedCheckBox = new wxCheckBox(pathFollowingOptionsSizer->GetStaticBox(),
                                               ID_IGNORE_OBJECT_SPEED_BOX,
                                               wxT("Ignore Object Speed"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    pathFollowingDisplayOptionsSizer->Add(ignoreObjectSpeedCheckBox, 0, wxALL, 5);


    pathFollowingOptionsSizer->Add(pathFollowingDisplayOptionsSizer, 0, wxALL, 5);

    wxFlexGridSizer* pathPlanningOptionsSizer;
    pathPlanningOptionsSizer = new wxFlexGridSizer(2, 3, 0, 0);
    pathPlanningOptionsSizer->SetFlexibleDirection(wxBOTH);
    pathPlanningOptionsSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_NONE);

    updatePlanTimeText = new wxTextCtrl(pathFollowingOptionsSizer->GetStaticBox(),
                                        ID_UPDATE_PLAN_TIME_TEXT,
                                        wxT("500"),
                                        wxDefaultPosition,
                                        wxSize(-1, -1),
                                        wxTE_RIGHT);
    pathPlanningOptionsSizer->Add(updatePlanTimeText, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    updateWaypointsButton = new wxButton(pathFollowingOptionsSizer->GetStaticBox(),
                                         ID_UPDATE_WAYPOINTS_BUTTON,
                                         wxT("Update (ms)"),
                                         wxDefaultPosition,
                                         wxDefaultSize,
                                         0);
    pathPlanningOptionsSizer->Add(updateWaypointsButton, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);


    pathFollowingOptionsSizer->Add(pathPlanningOptionsSizer, 1, wxALL | wxEXPAND, 5);


    metricPlannerOptionsSizer->Add(pathFollowingOptionsSizer, 0, wxALL | wxEXPAND, 5);


    metricPlannerScrollWindow->SetSizer(metricPlannerOptionsSizer);
    metricPlannerScrollWindow->Layout();
    metricPlannerOptionsSizer->Fit(metricPlannerScrollWindow);
    metricPlannerSizer->Add(metricPlannerScrollWindow, 0, wxEXPAND | wxALL, 5);


    metricPlannerPanel->SetSizer(metricPlannerSizer);
    metricPlannerPanel->Layout();
    metricPlannerSizer->Fit(metricPlannerPanel);
    frameNotebook->AddPage(metricPlannerPanel, wxT("Metric Planner"), false, wxNullBitmap);
    trackerPanel = new wxPanel(frameNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxBoxSizer* trackerPanelSizer;
    trackerPanelSizer = new wxBoxSizer(wxHORIZONTAL);

    wxBoxSizer* trackerWidgetSizer;
    trackerWidgetSizer = new wxBoxSizer(wxVERTICAL);

    trackerWidget = new TrackerDisplayWidget(trackerPanel, ID_TRACKER_WIDGET, wxDefaultPosition, wxDefaultSize);
    trackerWidgetSizer->Add(trackerWidget, 1, wxALL | wxEXPAND, 5);


    trackerPanelSizer->Add(trackerWidgetSizer, 1, wxEXPAND, 5);

    trackerPanelScrollWindow =
      new wxScrolledWindow(trackerPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL);
    trackerPanelScrollWindow->SetScrollRate(0, 5);
    wxBoxSizer* trackerPanelScrollSizer;
    trackerPanelScrollSizer = new wxBoxSizer(wxVERTICAL);

    trackerFollowRobotCheckbox = new wxCheckBox(trackerPanelScrollWindow,
                                                ID_TRACKER_FOLLOW_ROBOT_BOX,
                                                wxT("Follow Robot"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    trackerFollowRobotCheckbox->SetValue(true);
    trackerPanelScrollSizer->Add(trackerFollowRobotCheckbox, 0, wxALL | wxEXPAND, 5);

    wxStaticBoxSizer* laserObjectOptionsSizer;
    laserObjectOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(trackerPanelScrollWindow, wxID_ANY, wxT("Laser Object Options")),
                           wxVERTICAL);

    showLaserObjectsCheckbox = new wxCheckBox(laserObjectOptionsSizer->GetStaticBox(),
                                              ID_SHOW_LASER_OBJECTS_BOX,
                                              wxT("Show Laser Objects"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    showLaserObjectsCheckbox->SetValue(true);
    laserObjectOptionsSizer->Add(showLaserObjectsCheckbox, 0, wxALL | wxEXPAND, 5);

    showLaserObjectPointsCheckbox = new wxCheckBox(laserObjectOptionsSizer->GetStaticBox(),
                                                   ID_SHOW_LASER_OBJECT_POINTS_BOX,
                                                   wxT("Show Laser Points"),
                                                   wxDefaultPosition,
                                                   wxDefaultSize,
                                                   0);
    showLaserObjectPointsCheckbox->SetValue(true);
    laserObjectOptionsSizer->Add(showLaserObjectPointsCheckbox, 0, wxALL | wxEXPAND, 5);

    showLaserUncertaintyCheckbox = new wxCheckBox(laserObjectOptionsSizer->GetStaticBox(),
                                                  ID_SHOW_LASER_UNCERTAINTY_BOX,
                                                  wxT("Show Laser Uncertainty"),
                                                  wxDefaultPosition,
                                                  wxDefaultSize,
                                                  0);
    laserObjectOptionsSizer->Add(showLaserUncertaintyCheckbox, 0, wxALL, 5);

    wxString laserObjBoundaryRadioChoices[] = {wxT("Best"), wxT("Rectangle"), wxT("One Circle"), wxT("Two Circles")};
    int laserObjBoundaryRadioNChoices = sizeof(laserObjBoundaryRadioChoices) / sizeof(wxString);
    laserObjBoundaryRadio = new wxRadioBox(laserObjectOptionsSizer->GetStaticBox(),
                                           ID_LASER_OBJ_BOUNDARY_RADIO,
                                           wxT("Boundary:"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           laserObjBoundaryRadioNChoices,
                                           laserObjBoundaryRadioChoices,
                                           1,
                                           wxRA_SPECIFY_COLS);
    laserObjBoundaryRadio->SetSelection(0);
    laserObjectOptionsSizer->Add(laserObjBoundaryRadio, 0, wxALL | wxEXPAND, 5);


    trackerPanelScrollSizer->Add(laserObjectOptionsSizer, 0, wxALL | wxEXPAND, 5);

    wxStaticBoxSizer* trackedObjectOptionsSizer;
    trackedObjectOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(trackerPanelScrollWindow, wxID_ANY, wxT("Tracked Object Options")),
                           wxVERTICAL);

    showTrackedObjectsCheckbox = new wxCheckBox(trackedObjectOptionsSizer->GetStaticBox(),
                                                ID_SHOW_TRACKED_OBJECTS_BOX,
                                                wxT("Show Tracked Objects"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    showTrackedObjectsCheckbox->SetValue(true);
    trackedObjectOptionsSizer->Add(showTrackedObjectsCheckbox, 0, wxALL | wxEXPAND, 5);

    showTrackedAcclerationCheckbox = new wxCheckBox(trackedObjectOptionsSizer->GetStaticBox(),
                                                    ID_SHOW_TRACKED_ACCELERATION_BOX,
                                                    wxT("Show Acceleration"),
                                                    wxDefaultPosition,
                                                    wxDefaultSize,
                                                    0);
    showTrackedAcclerationCheckbox->SetValue(true);
    trackedObjectOptionsSizer->Add(showTrackedAcclerationCheckbox, 0, wxALL, 5);

    wxString rigidObjectStateRadioChoices[] = {wxT("Fast"), wxT("Slow")};
    int rigidObjectStateRadioNChoices = sizeof(rigidObjectStateRadioChoices) / sizeof(wxString);
    rigidObjectStateRadio = new wxRadioBox(trackedObjectOptionsSizer->GetStaticBox(),
                                           ID_RIGID_OBJECT_STATE_RADIO,
                                           wxT("Rigid Object State"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           rigidObjectStateRadioNChoices,
                                           rigidObjectStateRadioChoices,
                                           1,
                                           wxRA_SPECIFY_COLS);
    rigidObjectStateRadio->SetSelection(0);
    trackedObjectOptionsSizer->Add(rigidObjectStateRadio, 0, wxALL | wxEXPAND, 5);

    wxString trackingUncertaintyRadioChoices[] = {wxT("Position"), wxT("Velocity"), wxT("Acceleration"), wxT("None")};
    int trackingUncertaintyRadioNChoices = sizeof(trackingUncertaintyRadioChoices) / sizeof(wxString);
    trackingUncertaintyRadio = new wxRadioBox(trackedObjectOptionsSizer->GetStaticBox(),
                                              ID_TRACKING_UNCERTAINTY_RADIO,
                                              wxT("Uncertainty to Show"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              trackingUncertaintyRadioNChoices,
                                              trackingUncertaintyRadioChoices,
                                              1,
                                              wxRA_SPECIFY_COLS);
    trackingUncertaintyRadio->SetSelection(0);
    trackedObjectOptionsSizer->Add(trackingUncertaintyRadio, 0, wxALL | wxEXPAND, 5);

    showRecentObjectTrajectoryCheckbox = new wxCheckBox(trackedObjectOptionsSizer->GetStaticBox(),
                                                        ID_SHOW_RECENT_OBJECT_TRAJECTORY_BOX,
                                                        wxT("Show Recent Trajectory"),
                                                        wxDefaultPosition,
                                                        wxDefaultSize,
                                                        0);
    trackedObjectOptionsSizer->Add(showRecentObjectTrajectoryCheckbox, 0, wxALL | wxEXPAND, 5);


    trackerPanelScrollSizer->Add(trackedObjectOptionsSizer, 0, wxALL | wxEXPAND, 5);

    wxStaticBoxSizer* objectGoalsOptionsSizer;
    objectGoalsOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(trackerPanelScrollWindow, wxID_ANY, wxT("Object Goals Options")),
                           wxVERTICAL);

    wxString objectGoalsToShowRadioChoices[] = {wxT("Best"), wxT("All"), wxT("None")};
    int objectGoalsToShowRadioNChoices = sizeof(objectGoalsToShowRadioChoices) / sizeof(wxString);
    objectGoalsToShowRadio = new wxRadioBox(objectGoalsOptionsSizer->GetStaticBox(),
                                            ID_OBJECT_GOALS_TO_SHOW_RADIO,
                                            wxT("Goals to Show"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            objectGoalsToShowRadioNChoices,
                                            objectGoalsToShowRadioChoices,
                                            1,
                                            wxRA_SPECIFY_COLS);
    objectGoalsToShowRadio->SetSelection(2);
    objectGoalsOptionsSizer->Add(objectGoalsToShowRadio, 0, wxALL | wxEXPAND, 5);

    evaluateObjectGoalsButton = new wxToggleButton(objectGoalsOptionsSizer->GetStaticBox(),
                                                   ID_EVALUATE_OBJECT_GOALS_BUTTON,
                                                   wxT("Evaluate"),
                                                   wxDefaultPosition,
                                                   wxDefaultSize,
                                                   0);
    objectGoalsOptionsSizer->Add(evaluateObjectGoalsButton, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 5);


    trackerPanelScrollSizer->Add(objectGoalsOptionsSizer, 0, wxEXPAND, 5);

    wxStaticBoxSizer* motionPredictionOptionsSizer;
    motionPredictionOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(trackerPanelScrollWindow, wxID_ANY, wxT("Motion Prediction Options")),
                           wxVERTICAL);

    wxString motionPredictionsToShowRadioChoices[] = {wxT("Best"), wxT("All"), wxT("None")};
    int motionPredictionsToShowRadioNChoices = sizeof(motionPredictionsToShowRadioChoices) / sizeof(wxString);
    motionPredictionsToShowRadio = new wxRadioBox(motionPredictionOptionsSizer->GetStaticBox(),
                                                  ID_MOTION_PREDICTIONS_TO_SHOW_RADIO,
                                                  wxT("Prediction to Show"),
                                                  wxDefaultPosition,
                                                  wxDefaultSize,
                                                  motionPredictionsToShowRadioNChoices,
                                                  motionPredictionsToShowRadioChoices,
                                                  1,
                                                  wxRA_SPECIFY_COLS);
    motionPredictionsToShowRadio->SetSelection(2);
    motionPredictionOptionsSizer->Add(motionPredictionsToShowRadio, 0, wxALL | wxEXPAND, 5);

    wxBoxSizer* predictionDurationSizer;
    predictionDurationSizer = new wxBoxSizer(wxHORIZONTAL);

    predictionDurationLabel = new wxStaticText(motionPredictionOptionsSizer->GetStaticBox(),
                                               wxID_ANY,
                                               wxT("Duration (ms):"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    predictionDurationLabel->Wrap(-1);
    predictionDurationSizer->Add(predictionDurationLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    predictionDurationText = new wxTextCtrl(motionPredictionOptionsSizer->GetStaticBox(),
                                            wxID_ANY,
                                            wxT("5000"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    predictionDurationSizer->Add(predictionDurationText,
                                 0,
                                 wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL | wxEXPAND,
                                 5);


    motionPredictionOptionsSizer->Add(predictionDurationSizer, 1, wxEXPAND, 5);


    trackerPanelScrollSizer->Add(motionPredictionOptionsSizer, 0, wxEXPAND, 5);


    trackerPanelScrollWindow->SetSizer(trackerPanelScrollSizer);
    trackerPanelScrollWindow->Layout();
    trackerPanelScrollSizer->Fit(trackerPanelScrollWindow);
    trackerPanelSizer->Add(trackerPanelScrollWindow, 0, wxALL | wxEXPAND, 5);


    trackerPanel->SetSizer(trackerPanelSizer);
    trackerPanel->Layout();
    trackerPanelSizer->Fit(trackerPanel);
    frameNotebook->AddPage(trackerPanel, wxT("Tracker"), false, wxNullBitmap);
    localTopologyPanel = new wxPanel(frameNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxBoxSizer* localTopologySizer;
    localTopologySizer = new wxBoxSizer(wxHORIZONTAL);

    wxFlexGridSizer* localTopoWidgetSizer;
    localTopoWidgetSizer = new wxFlexGridSizer(1, 1, 0, 0);
    localTopoWidgetSizer->AddGrowableCol(0);
    localTopoWidgetSizer->AddGrowableRow(0);
    localTopoWidgetSizer->SetFlexibleDirection(wxBOTH);
    localTopoWidgetSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    localTopoWidget =
      new LocalTopoDisplayWidget(localTopologyPanel, ID_LOCAL_TOPO_WIDGET, wxDefaultPosition, wxDefaultSize);
    localTopoWidget->SetMinSize(wxSize(0, 0));

    localTopoWidgetSizer->Add(localTopoWidget, 1, wxALIGN_CENTER | wxEXPAND, 0);


    localTopologySizer->Add(localTopoWidgetSizer, 1, wxALL | wxEXPAND, 5);

    localTopoScrollWindow =
      new wxScrolledWindow(localTopologyPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL);
    localTopoScrollWindow->SetScrollRate(0, 5);
    wxFlexGridSizer* localTopoOptionsSizer;
    localTopoOptionsSizer = new wxFlexGridSizer(0, 1, 0, 0);
    localTopoOptionsSizer->SetFlexibleDirection(wxBOTH);
    localTopoOptionsSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxString localTopoModeRadioChoices[] = {wxT("Event Detection"), wxT("Labeling Only")};
    int localTopoModeRadioNChoices = sizeof(localTopoModeRadioChoices) / sizeof(wxString);
    localTopoModeRadio = new wxRadioBox(localTopoScrollWindow,
                                        ID_LOCAL_TOPO_MODE_RADIO,
                                        wxT("Local Topo Mode:"),
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        localTopoModeRadioNChoices,
                                        localTopoModeRadioChoices,
                                        1,
                                        wxRA_SPECIFY_COLS);
    localTopoModeRadio->SetSelection(0);
    localTopoOptionsSizer->Add(localTopoModeRadio, 0, wxALL | wxEXPAND, 5);

    wxStaticBoxSizer* placeGridOptionsSizer;
    placeGridOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(localTopoScrollWindow, wxID_ANY, wxT("Grid Options")), wxVERTICAL);

    showVoronoiGridCheckbox = new wxCheckBox(placeGridOptionsSizer->GetStaticBox(),
                                             ID_SHOW_VORONOI_GRID_BOX,
                                             wxT("Show Voronoi Grid"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    showVoronoiGridCheckbox->SetValue(true);
    placeGridOptionsSizer->Add(showVoronoiGridCheckbox, 0, wxALL | wxEXPAND, 5);

    showDistanceGradientCheckBox = new wxCheckBox(placeGridOptionsSizer->GetStaticBox(),
                                                  ID_SHOW_DISTANCE_GRADIENT_BOX,
                                                  wxT("Show Distance Gradient"),
                                                  wxDefaultPosition,
                                                  wxDefaultSize,
                                                  0);
    placeGridOptionsSizer->Add(showDistanceGradientCheckBox, 0, wxALL | wxEXPAND, 5);

    showFullSkeletonCheckBox = new wxCheckBox(placeGridOptionsSizer->GetStaticBox(),
                                              ID_SHOW_FULL_SKELETON_BOX,
                                              wxT("Show Full Skeleton"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    placeGridOptionsSizer->Add(showFullSkeletonCheckBox, 0, wxALL, 5);

    showReducedSkeletonCheckBox = new wxCheckBox(placeGridOptionsSizer->GetStaticBox(),
                                                 ID_SHOW_REDUCED_SKELETON_BOX,
                                                 wxT("Show Reduced Skeleton"),
                                                 wxDefaultPosition,
                                                 wxDefaultSize,
                                                 0);
    showReducedSkeletonCheckBox->SetValue(true);
    placeGridOptionsSizer->Add(showReducedSkeletonCheckBox, 0, wxALL, 5);

    followRobotTopoCheckBox = new wxCheckBox(placeGridOptionsSizer->GetStaticBox(),
                                             ID_FOLLOW_ROBOT_TOPO_BOX,
                                             wxT("Center on Robot"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    followRobotTopoCheckBox->SetValue(true);
    placeGridOptionsSizer->Add(followRobotTopoCheckBox, 0, wxALL, 5);

    showFrontiersCheckbox = new wxCheckBox(placeGridOptionsSizer->GetStaticBox(),
                                           ID_SHOW_FRONTIERS_BOX,
                                           wxT("Show Frontiers"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    showFrontiersCheckbox->SetValue(true);
    placeGridOptionsSizer->Add(showFrontiersCheckbox, 0, wxALL, 5);


    localTopoOptionsSizer->Add(placeGridOptionsSizer, 0, wxALIGN_RIGHT | wxEXPAND, 5);

    wxStaticBoxSizer* gatewayOptionsSizer;
    gatewayOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(localTopoScrollWindow, wxID_ANY, wxT("Gateways Display Options")),
                           wxVERTICAL);

    showGatewaysCheckBox = new wxCheckBox(gatewayOptionsSizer->GetStaticBox(),
                                          ID_SHOW_GATEWAYS_BOX,
                                          wxT("Show Gateways"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    gatewayOptionsSizer->Add(showGatewaysCheckBox, 0, wxALL, 5);

    wxArrayString gatewayTypeChoiceChoices;
    gatewayTypeChoice = new wxChoice(gatewayOptionsSizer->GetStaticBox(),
                                     ID_GATEWAY_TYPE_CHOICE,
                                     wxDefaultPosition,
                                     wxDefaultSize,
                                     gatewayTypeChoiceChoices,
                                     0);
    gatewayTypeChoice->SetSelection(0);
    gatewayOptionsSizer->Add(gatewayTypeChoice, 0, wxALL | wxEXPAND, 5);

    showNormalsCheckbox = new wxCheckBox(gatewayOptionsSizer->GetStaticBox(),
                                         ID_SHOW_NORMALS_BOX,
                                         wxT("Show Normals"),
                                         wxDefaultPosition,
                                         wxDefaultSize,
                                         0);
    gatewayOptionsSizer->Add(showNormalsCheckbox, 0, wxALL, 5);


    localTopoOptionsSizer->Add(gatewayOptionsSizer, 0, wxEXPAND, 5);

    wxStaticBoxSizer* areaOptionsSizer;
    areaOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(localTopoScrollWindow, wxID_ANY, wxT("Area Display Options")), wxVERTICAL);

    wxString showAreasRadioBoxChoices[] = {wxT("Local Topo Map"),
                                           wxT("Local Topo Graph"),
                                           wxT("HSSH Graph"),
                                           wxT("None")};
    int showAreasRadioBoxNChoices = sizeof(showAreasRadioBoxChoices) / sizeof(wxString);
    showAreasRadioBox = new wxRadioBox(areaOptionsSizer->GetStaticBox(),
                                       ID_SHOW_AREAS_BOX,
                                       wxT("Areas To Show"),
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       showAreasRadioBoxNChoices,
                                       showAreasRadioBoxChoices,
                                       1,
                                       wxRA_SPECIFY_COLS);
    showAreasRadioBox->SetSelection(0);
    areaOptionsSizer->Add(showAreasRadioBox, 0, wxALL | wxEXPAND, 5);

    saveLocalTopoMapButton = new wxButton(areaOptionsSizer->GetStaticBox(),
                                          ID_SAVE_LOCAL_TOPO_MAP_BUTTON,
                                          wxT("Save Local Topo Map"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    areaOptionsSizer->Add(saveLocalTopoMapButton, 0, wxALL | wxEXPAND, 5);

    loadLocalTopoMapButton = new wxButton(areaOptionsSizer->GetStaticBox(),
                                          ID_LOAD_LOCAL_TOPO_MAP_BUTTON,
                                          wxT("Load Local Topo Map"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    areaOptionsSizer->Add(loadLocalTopoMapButton, 0, wxALL | wxEXPAND, 5);

    sendLocalTopoMapButton = new wxButton(areaOptionsSizer->GetStaticBox(),
                                          ID_SEND_LOCAL_TOPO_MAP_BUTTON,
                                          wxT("Send Local Topo Map"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    areaOptionsSizer->Add(sendLocalTopoMapButton, 0, wxALL | wxEXPAND, 5);

    showSmallStarCheckBox = new wxCheckBox(areaOptionsSizer->GetStaticBox(),
                                           ID_SHOW_SMALL_STAR_BOX,
                                           wxT("Show Small-Scale Star"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    areaOptionsSizer->Add(showSmallStarCheckBox, 0, wxALL, 5);

    showAreaGatewaysCheckBox = new wxCheckBox(areaOptionsSizer->GetStaticBox(),
                                              ID_SHOW_AREA_GATEWAYS_BOX,
                                              wxT("Show Area Gateways"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    areaOptionsSizer->Add(showAreaGatewaysCheckBox, 0, wxALL, 5);

    showAreaGraphCheckBox = new wxCheckBox(areaOptionsSizer->GetStaticBox(),
                                           ID_SHOW_AREA_GRAPH_BOX,
                                           wxT("Show Area Graph"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    areaOptionsSizer->Add(showAreaGraphCheckBox, 0, wxALL, 5);

    wxString areaHypothesisValueRadioChoices[] = {wxT("Label"), wxT("Distribution"), wxT("Feature")};
    int areaHypothesisValueRadioNChoices = sizeof(areaHypothesisValueRadioChoices) / sizeof(wxString);
    areaHypothesisValueRadio = new wxRadioBox(areaOptionsSizer->GetStaticBox(),
                                              ID_AREA_HYPOTHESIS_VALUE_RADIO,
                                              wxT("Displayed Value"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              areaHypothesisValueRadioNChoices,
                                              areaHypothesisValueRadioChoices,
                                              1,
                                              wxRA_SPECIFY_COLS);
    areaHypothesisValueRadio->SetSelection(0);
    areaOptionsSizer->Add(areaHypothesisValueRadio, 0, wxALL | wxEXPAND, 5);

    wxBoxSizer* hypFeatureSizer;
    hypFeatureSizer = new wxBoxSizer(wxHORIZONTAL);

    hypFeatureLabel = new wxStaticText(areaOptionsSizer->GetStaticBox(),
                                       wxID_ANY,
                                       wxT("Hypothesis Feature:"),
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0);
    hypFeatureLabel->Wrap(-1);
    hypFeatureSizer->Add(hypFeatureLabel, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 5);

    hypFeatureValueText =
      new wxStaticText(areaOptionsSizer->GetStaticBox(), wxID_ANY, wxT("0.0"), wxDefaultPosition, wxDefaultSize, 0);
    hypFeatureValueText->Wrap(-1);
    hypFeatureSizer->Add(hypFeatureValueText, 0, wxALL, 5);


    areaOptionsSizer->Add(hypFeatureSizer, 0, wxEXPAND, 5);

    wxArrayString hypFeatureChoiceChoices;
    hypFeatureChoice = new wxChoice(areaOptionsSizer->GetStaticBox(),
                                    ID_HYP_FEATURE_CHOICE,
                                    wxDefaultPosition,
                                    wxDefaultSize,
                                    hypFeatureChoiceChoices,
                                    0);
    hypFeatureChoice->SetSelection(0);
    areaOptionsSizer->Add(hypFeatureChoice, 0, wxALL | wxEXPAND, 5);

    wxString hypothesesToShowRadioChoices[] = {wxT("None"),
                                               wxT("Max Likelihood"),
                                               wxT("Unnormalized"),
                                               wxT("Boosting"),
                                               wxT("Isovist")};
    int hypothesesToShowRadioNChoices = sizeof(hypothesesToShowRadioChoices) / sizeof(wxString);
    hypothesesToShowRadio = new wxRadioBox(areaOptionsSizer->GetStaticBox(),
                                           ID_HYPOTHESES_TO_SHOW_RADIO,
                                           wxT("Hypotheses to Show"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           hypothesesToShowRadioNChoices,
                                           hypothesesToShowRadioChoices,
                                           1,
                                           wxRA_SPECIFY_COLS);
    hypothesesToShowRadio->SetSelection(0);
    areaOptionsSizer->Add(hypothesesToShowRadio, 0, wxALL | wxEXPAND, 5);

    labelDistributionText =
      new wxStaticText(areaOptionsSizer->GetStaticBox(), wxID_ANY, wxT("0 0 0"), wxDefaultPosition, wxDefaultSize, 0);
    labelDistributionText->Wrap(-1);
    areaOptionsSizer->Add(labelDistributionText, 0, wxALL | wxEXPAND, 5);

    wxStaticBoxSizer* labelingCSPSizer;
    labelingCSPSizer =
      new wxStaticBoxSizer(new wxStaticBox(areaOptionsSizer->GetStaticBox(), wxID_ANY, wxT("CSP Options")), wxVERTICAL);

    cspLoadButton = new wxButton(labelingCSPSizer->GetStaticBox(),
                                 ID_CSP_LOAD_BUTTON,
                                 wxT("Load CSP Info"),
                                 wxDefaultPosition,
                                 wxDefaultSize,
                                 0);
    labelingCSPSizer->Add(cspLoadButton, 0, wxALL | wxEXPAND, 5);

    wxGridSizer* cspControlButtonSizer;
    cspControlButtonSizer = new wxGridSizer(0, 3, 0, 0);

    cspPlayButton = new wxButton(labelingCSPSizer->GetStaticBox(),
                                 ID_CSP_PLAY_BUTTON,
                                 wxT("Play"),
                                 wxDefaultPosition,
                                 wxDefaultSize,
                                 wxBU_EXACTFIT);
    cspControlButtonSizer->Add(cspPlayButton, 0, wxALL | wxEXPAND, 5);

    cspPauseButton = new wxButton(labelingCSPSizer->GetStaticBox(),
                                  ID_CSP_PAUSE_BUTTON,
                                  wxT("Pause"),
                                  wxDefaultPosition,
                                  wxDefaultSize,
                                  wxBU_EXACTFIT);
    cspControlButtonSizer->Add(cspPauseButton, 0, wxALL | wxEXPAND, 5);

    cspStopButton = new wxButton(labelingCSPSizer->GetStaticBox(),
                                 ID_CSP_STOP_BUTTON,
                                 wxT("Stop"),
                                 wxDefaultPosition,
                                 wxDefaultSize,
                                 wxBU_EXACTFIT);
    cspControlButtonSizer->Add(cspStopButton, 0, wxALL | wxEXPAND, 5);


    labelingCSPSizer->Add(cspControlButtonSizer, 0, wxEXPAND, 5);

    wxGridSizer* cspIterationControlSizer;
    cspIterationControlSizer = new wxGridSizer(1, 4, 0, 0);

    cspJumpToStartButton = new wxButton(labelingCSPSizer->GetStaticBox(),
                                        ID_CSP_JUMP_TO_START_BUTTON,
                                        wxT("<<"),
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        wxBU_EXACTFIT);
    cspIterationControlSizer->Add(cspJumpToStartButton, 0, wxALL | wxEXPAND, 5);

    cspPrevIterationButton = new wxButton(labelingCSPSizer->GetStaticBox(),
                                          ID_CSP_PREV_ITERATION_BUTTON,
                                          wxT(" <"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          wxBU_EXACTFIT);
    cspIterationControlSizer->Add(cspPrevIterationButton, 0, wxALL | wxEXPAND, 5);

    cspNextIterationButton = new wxButton(labelingCSPSizer->GetStaticBox(),
                                          ID_CSP_NEXT_ITERATION_BUTTON,
                                          wxT("> "),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          wxBU_EXACTFIT);
    cspIterationControlSizer->Add(cspNextIterationButton, 0, wxALL | wxEXPAND, 5);

    cspJumpToEndButton = new wxButton(labelingCSPSizer->GetStaticBox(),
                                      ID_CSP_JUMP_TO_END_BUTTON,
                                      wxT(">>"),
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      wxBU_EXACTFIT);
    cspIterationControlSizer->Add(cspJumpToEndButton, 0, wxALL | wxEXPAND, 5);


    labelingCSPSizer->Add(cspIterationControlSizer, 0, wxEXPAND, 5);

    cspIterationSlider = new wxSlider(labelingCSPSizer->GetStaticBox(),
                                      ID_CSP_ITERATION_SLIDER,
                                      50,
                                      0,
                                      100,
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      wxSL_HORIZONTAL | wxSL_LABELS);
    cspIterationSlider->SetToolTip(wxT("Current iteration of CSP search being shown"));

    labelingCSPSizer->Add(cspIterationSlider, 0, wxALL | wxEXPAND, 5);

    cspSpeedSlider = new wxSlider(labelingCSPSizer->GetStaticBox(),
                                  ID_CSP_SPEED_SLIDER,
                                  10,
                                  1,
                                  100,
                                  wxDefaultPosition,
                                  wxDefaultSize,
                                  wxSL_AUTOTICKS | wxSL_HORIZONTAL | wxSL_LABELS);
    cspSpeedSlider->SetToolTip(wxT("Adjust speed of playback in frames per iteration"));

    labelingCSPSizer->Add(cspSpeedSlider, 0, wxALL | wxEXPAND, 5);


    areaOptionsSizer->Add(labelingCSPSizer, 1, wxEXPAND, 5);

    wxStaticBoxSizer* localTopoHeatmapSizer;
    localTopoHeatmapSizer =
      new wxStaticBoxSizer(new wxStaticBox(areaOptionsSizer->GetStaticBox(), wxID_ANY, wxT("Heat Map Options")),
                           wxVERTICAL);

    showLocalHeatMapCheckBox = new wxCheckBox(localTopoHeatmapSizer->GetStaticBox(),
                                              ID_SHOW_LOCAL_HEAT_MAP_BOX,
                                              wxT("Show Heat Map"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    localTopoHeatmapSizer->Add(showLocalHeatMapCheckBox, 0, wxALL, 5);

    wxBoxSizer* numHeatMapPathsSizer;
    numHeatMapPathsSizer = new wxBoxSizer(wxHORIZONTAL);

    numPathsHeatMapLabel = new wxStaticText(localTopoHeatmapSizer->GetStaticBox(),
                                            wxID_ANY,
                                            wxT("Num Paths:"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    numPathsHeatMapLabel->Wrap(-1);
    numHeatMapPathsSizer->Add(numPathsHeatMapLabel, 0, wxALL, 5);

    numHeatMapPathsText = new wxTextCtrl(localTopoHeatmapSizer->GetStaticBox(),
                                         wxID_ANY,
                                         wxT("10000"),
                                         wxDefaultPosition,
                                         wxDefaultSize,
                                         0);
    numHeatMapPathsText->SetToolTip(wxT("Number of paths to generate for creating the heat map"));

    numHeatMapPathsSizer->Add(numHeatMapPathsText, 0, wxALL, 5);


    localTopoHeatmapSizer->Add(numHeatMapPathsSizer, 1, wxEXPAND, 5);

    generateLocalHeatMapButton = new wxButton(localTopoHeatmapSizer->GetStaticBox(),
                                              ID_GENERATE_LOCAL_HEAT_MAP_BUTTON,
                                              wxT("Generate Heat Map"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    localTopoHeatmapSizer->Add(generateLocalHeatMapButton, 0, wxALL | wxEXPAND, 5);


    areaOptionsSizer->Add(localTopoHeatmapSizer, 0, wxEXPAND, 5);


    localTopoOptionsSizer->Add(areaOptionsSizer, 0, wxALIGN_RIGHT | wxEXPAND, 5);

    wxStaticBoxSizer* localTopoEventsSizer;
    localTopoEventsSizer =
      new wxStaticBoxSizer(new wxStaticBox(localTopoScrollWindow, wxID_ANY, wxT("Event Options")), wxVERTICAL);

    localTopoEventList = new wxListBox(localTopoEventsSizer->GetStaticBox(),
                                       ID_LOCAL_TOPO_EVENT_LIST,
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0,
                                       NULL,
                                       wxLB_ALWAYS_SB | wxLB_SINGLE);
    localTopoEventList->Append(wxT("First"));
    localTopoEventList->Append(wxT("Second"));
    localTopoEventList->Append(wxT("Third"));
    localTopoEventList->Append(wxT("Fourth"));
    localTopoEventList->Append(wxT("Fifth"));
    localTopoEventList->Append(wxT("Sixth"));
    localTopoEventList->SetMinSize(wxSize(225, -1));

    localTopoEventsSizer->Add(localTopoEventList, 0, wxALL, 5);

    showLocalTopoEventCheckbox = new wxCheckBox(localTopoEventsSizer->GetStaticBox(),
                                                ID_SHOW_LOCAL_TOPO_EVENT_BOX,
                                                wxT("Show Event Visualization"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    localTopoEventsSizer->Add(showLocalTopoEventCheckbox, 0, wxALL, 5);


    localTopoOptionsSizer->Add(localTopoEventsSizer, 0, wxEXPAND, 5);

    wxStaticBoxSizer* isovistOptionsSizer;
    isovistOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(localTopoScrollWindow, wxID_ANY, wxT("Isovist Options")), wxVERTICAL);

    showIsovistBox = new wxCheckBox(isovistOptionsSizer->GetStaticBox(),
                                    ID_SHOW_ISOVIST_BOX,
                                    wxT("Show Isovist"),
                                    wxDefaultPosition,
                                    wxDefaultSize,
                                    0);
    isovistOptionsSizer->Add(showIsovistBox, 0, wxALL, 5);

    showIsovistFieldBox = new wxCheckBox(isovistOptionsSizer->GetStaticBox(),
                                         ID_SHOW_ISOVIST_FIELD_BOX,
                                         wxT("Show Isovist Field"),
                                         wxDefaultPosition,
                                         wxDefaultSize,
                                         0);
    isovistOptionsSizer->Add(showIsovistFieldBox, 0, wxALL, 5);

    showIsovistFieldDerivBox = new wxCheckBox(isovistOptionsSizer->GetStaticBox(),
                                              ID_SHOW_ISOVIST_DERIV_FIELD_BOX,
                                              wxT("Show Isovist Deriv Field"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    isovistOptionsSizer->Add(showIsovistFieldDerivBox, 0, wxALL | wxEXPAND, 5);

    wxString isovistLocationRadioChoices[] = {wxT("Free Space"), wxT("Skeleton"), wxT("Reduced Skeleton")};
    int isovistLocationRadioNChoices = sizeof(isovistLocationRadioChoices) / sizeof(wxString);
    isovistLocationRadio = new wxRadioBox(isovistOptionsSizer->GetStaticBox(),
                                          wxID_ANY,
                                          wxT("Location"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          isovistLocationRadioNChoices,
                                          isovistLocationRadioChoices,
                                          1,
                                          wxRA_SPECIFY_COLS);
    isovistLocationRadio->SetSelection(2);
    isovistOptionsSizer->Add(isovistLocationRadio, 0, wxALL | wxEXPAND, 5);

    calculateIsovistsButton = new wxButton(isovistOptionsSizer->GetStaticBox(),
                                           ID_CALCULATE_ISOVISTS_BUTTON,
                                           wxT("Calculate"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    isovistOptionsSizer->Add(calculateIsovistsButton, 0, wxALL | wxEXPAND, 5);

    selectIsovistsButton = new wxToggleButton(isovistOptionsSizer->GetStaticBox(),
                                              ID_SELECT_ISOVISTS_BUTTON,
                                              wxT("Select Isovists"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    isovistOptionsSizer->Add(selectIsovistsButton, 0, wxALL | wxEXPAND, 5);

    wxBoxSizer* scalarOptionSizer;
    scalarOptionSizer = new wxBoxSizer(wxHORIZONTAL);

    scalarChoiceLabel = new wxStaticText(isovistOptionsSizer->GetStaticBox(),
                                         wxID_ANY,
                                         wxT("Scalar:"),
                                         wxDefaultPosition,
                                         wxDefaultSize,
                                         0);
    scalarChoiceLabel->Wrap(-1);
    scalarOptionSizer->Add(scalarChoiceLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    wxArrayString fieldScalarChoiceChoices;
    fieldScalarChoice = new wxChoice(isovistOptionsSizer->GetStaticBox(),
                                     ID_FIELD_SCALAR_CHOICE,
                                     wxDefaultPosition,
                                     wxDefaultSize,
                                     fieldScalarChoiceChoices,
                                     0);
    fieldScalarChoice->SetSelection(0);
    scalarOptionSizer->Add(fieldScalarChoice, 0, wxALL | wxEXPAND, 5);


    isovistOptionsSizer->Add(scalarOptionSizer, 0, wxEXPAND, 5);

    calculateGradientsButton = new wxButton(isovistOptionsSizer->GetStaticBox(),
                                            ID_CALCULATE_GRADIENTS_BUTTON,
                                            wxT("Calculate Gradients"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    isovistOptionsSizer->Add(calculateGradientsButton, 0, wxALL | wxEXPAND, 5);

    showCellGradientsCheckbox = new wxCheckBox(isovistOptionsSizer->GetStaticBox(),
                                               ID_SHOW_CELL_GRADIENTS_BOX,
                                               wxT("Show Cell Gradients"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    isovistOptionsSizer->Add(showCellGradientsCheckbox, 0, wxALL, 5);

    showIsovistMaximaCheckbox = new wxCheckBox(isovistOptionsSizer->GetStaticBox(),
                                               ID_SHOW_ISOVIST_MAXIMA_BOX,
                                               wxT("Show Gradient Maxima"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    isovistOptionsSizer->Add(showIsovistMaximaCheckbox, 0, wxALL, 5);

    loadGatewayClassifierButton = new wxButton(isovistOptionsSizer->GetStaticBox(),
                                               ID_LOAD_GATEWAY_CLASSIFIER_BUTTON,
                                               wxT("Load Gateway Classifier"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    isovistOptionsSizer->Add(loadGatewayClassifierButton, 0, wxALL | wxEXPAND, 5);

    calculateGatewayProbabilityButton = new wxButton(isovistOptionsSizer->GetStaticBox(),
                                                     ID_CALCULATE_GATEWAY_PROBABILITIES_BUTTON,
                                                     wxT("Calculate Probabilities"),
                                                     wxDefaultPosition,
                                                     wxDefaultSize,
                                                     0);
    isovistOptionsSizer->Add(calculateGatewayProbabilityButton, 0, wxALL | wxEXPAND, 5);

    showGatewayProbabilitiesBox = new wxCheckBox(isovistOptionsSizer->GetStaticBox(),
                                                 ID_SHOW_GATEWAY_PROBABILITIES_BOX,
                                                 wxT("Show Probabilities"),
                                                 wxDefaultPosition,
                                                 wxDefaultSize,
                                                 0);
    isovistOptionsSizer->Add(showGatewayProbabilitiesBox, 0, wxALL | wxEXPAND, 5);

    gatewayProbCutoffLabel = new wxStaticText(isovistOptionsSizer->GetStaticBox(),
                                              wxID_ANY,
                                              wxT("Probability Cutoff:"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              wxALIGN_CENTER_HORIZONTAL);
    gatewayProbCutoffLabel->Wrap(-1);
    isovistOptionsSizer->Add(gatewayProbCutoffLabel, 0, wxALL | wxEXPAND, 5);

    gatewayCutoffSlider = new wxSlider(isovistOptionsSizer->GetStaticBox(),
                                       ID_GATEWAY_CUTOFF_SLIDER,
                                       50,
                                       0,
                                       100,
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       wxSL_HORIZONTAL | wxSL_LABELS);
    isovistOptionsSizer->Add(gatewayCutoffSlider, 0, wxALL | wxEXPAND, 5);

    wxString gatewayClassifierRadioChoices[] = {wxT("Logistic"), wxT("SVM"), wxT("AdaBoost")};
    int gatewayClassifierRadioNChoices = sizeof(gatewayClassifierRadioChoices) / sizeof(wxString);
    gatewayClassifierRadio = new wxRadioBox(isovistOptionsSizer->GetStaticBox(),
                                            wxID_ANY,
                                            wxT("Gateway Classifier To Use:"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            gatewayClassifierRadioNChoices,
                                            gatewayClassifierRadioChoices,
                                            1,
                                            wxRA_SPECIFY_COLS);
    gatewayClassifierRadio->SetSelection(2);
    isovistOptionsSizer->Add(gatewayClassifierRadio, 0, wxALL | wxEXPAND, 5);


    localTopoOptionsSizer->Add(isovistOptionsSizer, 0, wxEXPAND, 5);

    wxStaticBoxSizer* visibilityGraphOptionsSizer;
    visibilityGraphOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(localTopoScrollWindow, wxID_ANY, wxT("Visibility Graph Options")),
                           wxVERTICAL);

    showVisibilityGraphBox = new wxCheckBox(visibilityGraphOptionsSizer->GetStaticBox(),
                                            ID_SHOW_VISIBILITY_GRAPH_BOX,
                                            wxT("Show Visibility Graph"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    visibilityGraphOptionsSizer->Add(showVisibilityGraphBox, 0, wxALL, 5);

    wxBoxSizer* visibilityFeatureSizer;
    visibilityFeatureSizer = new wxBoxSizer(wxHORIZONTAL);

    visibilityFeatureChoiceLabel = new wxStaticText(visibilityGraphOptionsSizer->GetStaticBox(),
                                                    wxID_ANY,
                                                    wxT("Feature:"),
                                                    wxDefaultPosition,
                                                    wxDefaultSize,
                                                    0);
    visibilityFeatureChoiceLabel->Wrap(-1);
    visibilityFeatureSizer->Add(visibilityFeatureChoiceLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    wxArrayString visibilityFeatureChoiceChoices;
    visibilityFeatureChoice = new wxChoice(visibilityGraphOptionsSizer->GetStaticBox(),
                                           ID_VISIBILITY_FEATURE_CHOICE,
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           visibilityFeatureChoiceChoices,
                                           0);
    visibilityFeatureChoice->SetSelection(0);
    visibilityFeatureSizer->Add(visibilityFeatureChoice, 0, wxALL | wxEXPAND, 5);


    visibilityGraphOptionsSizer->Add(visibilityFeatureSizer, 1, wxEXPAND, 5);


    localTopoOptionsSizer->Add(visibilityGraphOptionsSizer, 0, wxEXPAND, 5);


    localTopoScrollWindow->SetSizer(localTopoOptionsSizer);
    localTopoScrollWindow->Layout();
    localTopoOptionsSizer->Fit(localTopoScrollWindow);
    localTopologySizer->Add(localTopoScrollWindow, 0, wxEXPAND | wxALL, 5);


    localTopologyPanel->SetSizer(localTopologySizer);
    localTopologyPanel->Layout();
    localTopologySizer->Fit(localTopologyPanel);
    frameNotebook->AddPage(localTopologyPanel, wxT("Local Topology"), false, wxNullBitmap);
    globalTopologyPanel = new wxPanel(frameNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxBoxSizer* globalTopologySizer;
    globalTopologySizer = new wxBoxSizer(wxHORIZONTAL);

    wxBoxSizer* globalTopoWidgetSizer;
    globalTopoWidgetSizer = new wxBoxSizer(wxVERTICAL);

    globalTopoWidget =
      new GlobalTopoDisplayWidget(globalTopologyPanel, ID_GLOBAL_TOPO_WIDGET, wxDefaultPosition, wxDefaultSize);
    globalTopoWidget->SetMinSize(wxSize(0, 0));

    globalTopoWidgetSizer->Add(globalTopoWidget, 1, wxALL | wxEXPAND, 5);


    globalTopologySizer->Add(globalTopoWidgetSizer, 1, wxEXPAND, 5);

    globalTopoScrolledWindow =
      new wxScrolledWindow(globalTopologyPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL);
    globalTopoScrolledWindow->SetScrollRate(0, 5);
    wxStaticBoxSizer* globalTopologyOptionsSizer;
    globalTopologyOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(globalTopoScrolledWindow, wxID_ANY, wxT("Global Topology Options")),
                           wxVERTICAL);

    wxString globalTopoMapViewRadioBoxChoices[] = {wxT("Graph View"), wxT("Place View"), wxT("Hypothesis Tree")};
    int globalTopoMapViewRadioBoxNChoices = sizeof(globalTopoMapViewRadioBoxChoices) / sizeof(wxString);
    globalTopoMapViewRadioBox = new wxRadioBox(globalTopologyOptionsSizer->GetStaticBox(),
                                               ID_GLOBAL_TOPO_MAP_VIEW_RADIO_BOX,
                                               wxT("Map View"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               globalTopoMapViewRadioBoxNChoices,
                                               globalTopoMapViewRadioBoxChoices,
                                               1,
                                               wxRA_SPECIFY_COLS);
    globalTopoMapViewRadioBox->SetSelection(0);
    globalTopologyOptionsSizer->Add(globalTopoMapViewRadioBox, 0, wxALL | wxEXPAND, 5);

    wxFlexGridSizer* numHypothesesSizer;
    numHypothesesSizer = new wxFlexGridSizer(0, 2, 0, 0);
    numHypothesesSizer->SetFlexibleDirection(wxHORIZONTAL);
    numHypothesesSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    activeHypothesesText = new wxStaticText(globalTopologyOptionsSizer->GetStaticBox(),
                                            wxID_ANY,
                                            wxT("Num Leaves:"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    activeHypothesesText->Wrap(-1);
    numHypothesesSizer->Add(activeHypothesesText, 0, wxALL, 5);

    numActiveHypothesesLabel = new wxStaticText(globalTopologyOptionsSizer->GetStaticBox(),
                                                wxID_ANY,
                                                wxT("0"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    numActiveHypothesesLabel->Wrap(-1);
    numHypothesesSizer->Add(numActiveHypothesesLabel, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 5);

    numCompleteHypothesesText = new wxStaticText(globalTopologyOptionsSizer->GetStaticBox(),
                                                 wxID_ANY,
                                                 wxT("Num Complete:"),
                                                 wxDefaultPosition,
                                                 wxDefaultSize,
                                                 0);
    numCompleteHypothesesText->Wrap(-1);
    numHypothesesSizer->Add(numCompleteHypothesesText, 0, wxALL, 5);

    numCompleteHypothesesLabel = new wxStaticText(globalTopologyOptionsSizer->GetStaticBox(),
                                                  wxID_ANY,
                                                  wxT("0"),
                                                  wxDefaultPosition,
                                                  wxDefaultSize,
                                                  0);
    numCompleteHypothesesLabel->Wrap(-1);
    numHypothesesSizer->Add(numCompleteHypothesesLabel, 0, wxALL, 5);


    globalTopologyOptionsSizer->Add(numHypothesesSizer, 0, wxEXPAND, 5);

    showBestTopoMapCheckBox = new wxCheckBox(globalTopologyOptionsSizer->GetStaticBox(),
                                             ID_SHOW_BEST_TOPO_MAP_CHECK_BOX,
                                             wxT("Show Best Map"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    globalTopologyOptionsSizer->Add(showBestTopoMapCheckBox, 0, wxALL, 5);

    mapToShowLabel = new wxStaticText(globalTopologyOptionsSizer->GetStaticBox(),
                                      wxID_ANY,
                                      wxT("Map To Show:"),
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      0);
    mapToShowLabel->Wrap(-1);
    globalTopologyOptionsSizer->Add(mapToShowLabel, 0, wxALL, 5);

    topoHypothesisComboBox = new wxComboBox(globalTopologyOptionsSizer->GetStaticBox(),
                                            ID_TOPO_HYPOTHESIS_COMBO_BOX,
                                            wxEmptyString,
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0,
                                            NULL,
                                            wxCB_DROPDOWN);
    globalTopologyOptionsSizer->Add(topoHypothesisComboBox, 0, wxALL | wxEXPAND, 5);

    wxBoxSizer* switchMapsSizer;
    switchMapsSizer = new wxBoxSizer(wxHORIZONTAL);

    previousHypothesisButton = new wxButton(globalTopologyOptionsSizer->GetStaticBox(),
                                            ID_PREVIOUS_HYPOTHESIS_BUTTON,
                                            wxT("Prev"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    switchMapsSizer->Add(previousHypothesisButton, 0, wxALL, 5);

    nextHypothesisButton = new wxButton(globalTopologyOptionsSizer->GetStaticBox(),
                                        ID_NEXT_HYPOTHESIS_BUTTON,
                                        wxT("Next"),
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        0);
    switchMapsSizer->Add(nextHypothesisButton, 0, wxALL, 5);


    globalTopologyOptionsSizer->Add(switchMapsSizer, 0, 0, 5);

    useGlobalTopoMapButton = new wxButton(globalTopologyOptionsSizer->GetStaticBox(),
                                          ID_USE_GLOBAL_TOPO_MAP_BUTTON,
                                          wxT("Use Current Map"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    globalTopologyOptionsSizer->Add(useGlobalTopoMapButton, 0, wxALIGN_CENTER | wxALL, 5);

    loadGlobalTopoMapButton = new wxButton(globalTopologyOptionsSizer->GetStaticBox(),
                                           ID_LOAD_GLOBAL_TOPO_MAP_FROM_FILE_BUTTON,
                                           wxT("Load From File"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    globalTopologyOptionsSizer->Add(loadGlobalTopoMapButton, 0, wxALIGN_CENTER | wxALL, 5);

    saveCurrentMapButton = new wxButton(globalTopologyOptionsSizer->GetStaticBox(),
                                        ID_SAVE_CURRENT_MAP_BUTTON,
                                        wxT("Save Current Map"),
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        0);
    globalTopologyOptionsSizer->Add(saveCurrentMapButton, 0, wxALIGN_CENTER | wxALL, 5);

    wxStaticBoxSizer* treeOfMapsCommandsSizer;
    treeOfMapsCommandsSizer = new wxStaticBoxSizer(
      new wxStaticBox(globalTopologyOptionsSizer->GetStaticBox(), wxID_ANY, wxT("Tree of Maps Options")),
      wxVERTICAL);

    wxGridSizer* treeOfMapOptionsButtonSizer;
    treeOfMapOptionsButtonSizer = new wxGridSizer(2, 2, 0, 0);

    saveTreeButton = new wxButton(treeOfMapsCommandsSizer->GetStaticBox(),
                                  ID_SAVE_TREE_BUTTON,
                                  wxT("Save Tree"),
                                  wxDefaultPosition,
                                  wxDefaultSize,
                                  0);
    saveTreeButton->SetToolTip(wxT("Save the tree of maps in global_topo_hssh to current_tree_of_maps.tom"));

    treeOfMapOptionsButtonSizer->Add(saveTreeButton, 0, wxALL, 5);

    loadTreeButton = new wxButton(treeOfMapsCommandsSizer->GetStaticBox(),
                                  ID_LOAD_TREE_BUTTON,
                                  wxT("Load Tree"),
                                  wxDefaultPosition,
                                  wxDefaultSize,
                                  0);
    loadTreeButton->SetToolTip(wxT("Load the tree of maps saved at current_tree_of_maps.tom"));

    treeOfMapOptionsButtonSizer->Add(loadTreeButton, 0, wxALL, 5);

    saveMapCacheButton = new wxButton(treeOfMapsCommandsSizer->GetStaticBox(),
                                      ID_SAVE_MAP_CACHE_BUTTON,
                                      wxT("Save Cache"),
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      0);
    saveMapCacheButton->SetToolTip(wxT("Save current MetricMapCache to current_map_cache.mmc"));

    treeOfMapOptionsButtonSizer->Add(saveMapCacheButton, 0, wxALL, 5);

    loadMapCacheButton = new wxButton(treeOfMapsCommandsSizer->GetStaticBox(),
                                      ID_LOAD_MAP_CACHE_BUTTON,
                                      wxT("Load Cache"),
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      0);
    loadMapCacheButton->SetToolTip(wxT("Load current MetricMapCache from current_map_cache.mmc"));

    treeOfMapOptionsButtonSizer->Add(loadMapCacheButton, 0, wxALL, 5);


    treeOfMapsCommandsSizer->Add(treeOfMapOptionsButtonSizer, 1, wxEXPAND, 5);


    globalTopologyOptionsSizer->Add(treeOfMapsCommandsSizer, 0, wxEXPAND, 5);

    wxStaticBoxSizer* hypothesisInfoSizer;
    hypothesisInfoSizer = new wxStaticBoxSizer(
      new wxStaticBox(globalTopologyOptionsSizer->GetStaticBox(), wxID_ANY, wxT("Hypothesis Info:")),
      wxVERTICAL);

    hypothesisInfoGrid =
      new wxGrid(hypothesisInfoSizer->GetStaticBox(), ID_HYPOTHESIS_INFO_GRID, wxDefaultPosition, wxDefaultSize, 0);

    // Grid
    hypothesisInfoGrid->CreateGrid(7, 1);
    hypothesisInfoGrid->EnableEditing(true);
    hypothesisInfoGrid->EnableGridLines(true);
    hypothesisInfoGrid->EnableDragGridSize(false);
    hypothesisInfoGrid->SetMargins(0, 0);

    // Columns
    hypothesisInfoGrid->EnableDragColMove(false);
    hypothesisInfoGrid->EnableDragColSize(false);
    hypothesisInfoGrid->SetColLabelSize(3);
    hypothesisInfoGrid->SetColLabelAlignment(wxALIGN_CENTER, wxALIGN_CENTER);

    // Rows
    hypothesisInfoGrid->EnableDragRowSize(true);
    hypothesisInfoGrid->SetRowLabelValue(0, wxT("Id:"));
    hypothesisInfoGrid->SetRowLabelValue(1, wxT("Depth:"));
    hypothesisInfoGrid->SetRowLabelValue(2, wxT("Posterior:"));
    hypothesisInfoGrid->SetRowLabelValue(3, wxT("Likelihood:"));
    hypothesisInfoGrid->SetRowLabelValue(4, wxT("Prior:"));
    hypothesisInfoGrid->SetRowLabelValue(5, wxT("Heuristic Likelihood:"));
    hypothesisInfoGrid->SetRowLabelValue(6, wxT("Heuristic Prior:"));
    hypothesisInfoGrid->SetRowLabelValue(7, wxEmptyString);
    hypothesisInfoGrid->SetRowLabelSize(80);
    hypothesisInfoGrid->SetRowLabelAlignment(wxALIGN_RIGHT, wxALIGN_CENTER);

    // Label Appearance

    // Cell Defaults
    hypothesisInfoGrid->SetDefaultCellAlignment(wxALIGN_LEFT, wxALIGN_CENTER);
    hypothesisInfoSizer->Add(hypothesisInfoGrid, 0, wxALL | wxEXPAND, 5);


    globalTopologyOptionsSizer->Add(hypothesisInfoSizer, 0, 0, 5);

    clearGlobalTopoMapsButton = new wxButton(globalTopologyOptionsSizer->GetStaticBox(),
                                             ID_CLEAR_GLOBAL_TOPO_MAPS_BUTTON,
                                             wxT("Clear Maps"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    globalTopologyOptionsSizer->Add(clearGlobalTopoMapsButton, 0, wxALIGN_CENTER | wxALL | wxEXPAND, 5);


    globalTopoScrolledWindow->SetSizer(globalTopologyOptionsSizer);
    globalTopoScrolledWindow->Layout();
    globalTopologyOptionsSizer->Fit(globalTopoScrolledWindow);
    globalTopologySizer->Add(globalTopoScrolledWindow, 0, wxEXPAND | wxALL, 5);


    globalTopologyPanel->SetSizer(globalTopologySizer);
    globalTopologyPanel->Layout();
    globalTopologySizer->Fit(globalTopologyPanel);
    frameNotebook->AddPage(globalTopologyPanel, wxT("Global Topology"), false, wxNullBitmap);
    evaluationPanel =
      new wxPanel(frameNotebook, ID_EVALUATION_PANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxBoxSizer* evaluationSizer;
    evaluationSizer = new wxBoxSizer(wxHORIZONTAL);

    wxFlexGridSizer* evaluationWidgetSizer;
    evaluationWidgetSizer = new wxFlexGridSizer(1, 1, 0, 0);
    evaluationWidgetSizer->AddGrowableCol(0);
    evaluationWidgetSizer->AddGrowableRow(0);
    evaluationWidgetSizer->SetFlexibleDirection(wxBOTH);
    evaluationWidgetSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    evaluationWidget =
      new EvaluationDisplayWidget(evaluationPanel, ID_EVALUATION_WIDGET, wxDefaultPosition, wxDefaultSize);
    evaluationWidget->SetMinSize(wxSize(0, 0));

    evaluationWidgetSizer->Add(evaluationWidget, 1, wxALIGN_CENTER | wxEXPAND, 0);


    evaluationSizer->Add(evaluationWidgetSizer, 1, wxALL | wxEXPAND, 5);

    evaluationScrollWindow =
      new wxScrolledWindow(evaluationPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL);
    evaluationScrollWindow->SetScrollRate(0, 5);
    wxFlexGridSizer* evaluationOptionsSizer;
    evaluationOptionsSizer = new wxFlexGridSizer(0, 1, 0, 0);
    evaluationOptionsSizer->SetFlexibleDirection(wxBOTH);
    evaluationOptionsSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* stabilityEvalSizer;
    stabilityEvalSizer =
      new wxStaticBoxSizer(new wxStaticBox(evaluationScrollWindow, wxID_ANY, wxT("Stability:")), wxVERTICAL);

    loadEvalMapButton = new wxButton(stabilityEvalSizer->GetStaticBox(),
                                     ID_LOAD_EVAL_MAP_BUTTON,
                                     wxT("Load Map"),
                                     wxDefaultPosition,
                                     wxDefaultSize,
                                     0);
    stabilityEvalSizer->Add(loadEvalMapButton, 0, wxALL | wxEXPAND, 5);

    importStabilityLogButton = new wxButton(stabilityEvalSizer->GetStaticBox(),
                                            ID_IMPORT_STABILITY_LOG_BUTTON,
                                            wxT("Import Log"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    stabilityEvalSizer->Add(importStabilityLogButton, 0, wxALL | wxEXPAND, 5);

    clearStabilityEvalButton = new wxButton(stabilityEvalSizer->GetStaticBox(),
                                            ID_CLEAR_STABILITY_EVAL_BUTTON,
                                            wxT("Clear All"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    stabilityEvalSizer->Add(clearStabilityEvalButton, 0, wxALL | wxEXPAND, 5);

    wxStaticBoxSizer* evalRenderOptionsSizer;
    evalRenderOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(stabilityEvalSizer->GetStaticBox(), wxID_ANY, wxT("Rendering Options:")),
                           wxVERTICAL);

    drawEvalBoundaryBox = new wxCheckBox(evalRenderOptionsSizer->GetStaticBox(),
                                         ID_DRAW_EVAL_BOUNDARY_BOX,
                                         wxT("Draw Boundary"),
                                         wxDefaultPosition,
                                         wxDefaultSize,
                                         0);
    drawEvalBoundaryBox->SetValue(true);
    evalRenderOptionsSizer->Add(drawEvalBoundaryBox, 0, wxALL | wxEXPAND, 5);

    drawEvalStarBox = new wxCheckBox(evalRenderOptionsSizer->GetStaticBox(),
                                     ID_DRAW_EVAL_STAR_BOX,
                                     wxT("Draw Star"),
                                     wxDefaultPosition,
                                     wxDefaultSize,
                                     0);
    evalRenderOptionsSizer->Add(drawEvalStarBox, 0, wxALL | wxEXPAND, 5);


    stabilityEvalSizer->Add(evalRenderOptionsSizer, 0, wxEXPAND, 5);


    evaluationOptionsSizer->Add(stabilityEvalSizer, 1, wxEXPAND, 5);

    wxStaticBoxSizer* mpepcTrajectoryEvalSizer;
    mpepcTrajectoryEvalSizer =
      new wxStaticBoxSizer(new wxStaticBox(evaluationScrollWindow, wxID_ANY, wxT("MPEPC Trajectory:")), wxHORIZONTAL);

    loadMPEPCResultsFileButton = new wxButton(mpepcTrajectoryEvalSizer->GetStaticBox(),
                                              ID_LOAD_MPEPC_RESULTS_FILE_BUTTON,
                                              wxT("Load Results File"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    mpepcTrajectoryEvalSizer->Add(loadMPEPCResultsFileButton, 0, wxALL | wxEXPAND, 5);


    evaluationOptionsSizer->Add(mpepcTrajectoryEvalSizer, 1, wxEXPAND, 5);


    evaluationScrollWindow->SetSizer(evaluationOptionsSizer);
    evaluationScrollWindow->Layout();
    evaluationOptionsSizer->Fit(evaluationScrollWindow);
    evaluationSizer->Add(evaluationScrollWindow, 0, wxEXPAND | wxALL, 5);


    evaluationPanel->SetSizer(evaluationSizer);
    evaluationPanel->Layout();
    evaluationSizer->Fit(evaluationPanel);
    frameNotebook->AddPage(evaluationPanel, wxT("Evaluation"), true, wxNullBitmap);
    explorationPanel =
      new wxPanel(frameNotebook, ID_EXPLORATION_PANEL, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxBoxSizer* explorationPanelSizer;
    explorationPanelSizer = new wxBoxSizer(wxHORIZONTAL);

    explorationWidget =
      new ExplorationDisplayWidget(explorationPanel, ID_EXPLORATION_WIDGET, wxDefaultPosition, wxDefaultSize);
    localMetricWidget->SetSize(wxSize(800, 600));
    explorationWidget->SetMinSize(wxSize(0, 0));

    explorationPanelSizer->Add(explorationWidget, 1, wxALL | wxEXPAND, 5);

    wxBoxSizer* explorationOptionsSizer;
    explorationOptionsSizer = new wxBoxSizer(wxVERTICAL);

    wxStaticBoxSizer* localTopoExplorationSizer;
    localTopoExplorationSizer =
      new wxStaticBoxSizer(new wxStaticBox(explorationPanel, wxID_ANY, wxT("Local Topo Exploration:")), wxVERTICAL);

    wxGridSizer* localTopoExplorationStatusSizer;
    localTopoExplorationStatusSizer = new wxGridSizer(0, 2, 0, 0);

    totalLocalTopoAreasLabel = new wxStaticText(localTopoExplorationSizer->GetStaticBox(),
                                                wxID_ANY,
                                                wxT("Total:"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    totalLocalTopoAreasLabel->Wrap(-1);
    localTopoExplorationStatusSizer->Add(totalLocalTopoAreasLabel, 0, wxALIGN_RIGHT | wxALL, 5);

    totalLocalTopoAreasText = new wxStaticText(localTopoExplorationSizer->GetStaticBox(),
                                               wxID_ANY,
                                               wxT("0"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    totalLocalTopoAreasText->Wrap(-1);
    localTopoExplorationStatusSizer->Add(totalLocalTopoAreasText, 0, wxALL | wxLEFT, 5);

    visitedLocalTopoAreasLabel = new wxStaticText(localTopoExplorationSizer->GetStaticBox(),
                                                  wxID_ANY,
                                                  wxT("Visited:"),
                                                  wxDefaultPosition,
                                                  wxDefaultSize,
                                                  0);
    visitedLocalTopoAreasLabel->Wrap(-1);
    localTopoExplorationStatusSizer->Add(visitedLocalTopoAreasLabel, 0, wxALIGN_RIGHT | wxALL, 5);

    visitedLocalTopoAreasText = new wxStaticText(localTopoExplorationSizer->GetStaticBox(),
                                                 wxID_ANY,
                                                 wxT("0"),
                                                 wxDefaultPosition,
                                                 wxDefaultSize,
                                                 0);
    visitedLocalTopoAreasText->Wrap(-1);
    localTopoExplorationStatusSizer->Add(visitedLocalTopoAreasText, 0, wxALL | wxLEFT, 5);

    remainingLocalTopoAreasLabel = new wxStaticText(localTopoExplorationSizer->GetStaticBox(),
                                                    wxID_ANY,
                                                    wxT("Remaining:"),
                                                    wxDefaultPosition,
                                                    wxDefaultSize,
                                                    0);
    remainingLocalTopoAreasLabel->Wrap(-1);
    localTopoExplorationStatusSizer->Add(remainingLocalTopoAreasLabel, 0, wxALIGN_RIGHT | wxALL, 5);

    remainingLocalTopoAreasText = new wxStaticText(localTopoExplorationSizer->GetStaticBox(),
                                                   wxID_ANY,
                                                   wxT("0"),
                                                   wxDefaultPosition,
                                                   wxDefaultSize,
                                                   0);
    remainingLocalTopoAreasText->Wrap(-1);
    localTopoExplorationStatusSizer->Add(remainingLocalTopoAreasText, 0, wxALL, 5);


    localTopoExplorationSizer->Add(localTopoExplorationStatusSizer, 0, wxALL | wxEXPAND, 5);


    explorationOptionsSizer->Add(localTopoExplorationSizer, 0, wxALL | wxEXPAND, 5);

    explorationCenterOnRobotCheckbox = new wxCheckBox(explorationPanel,
                                                      ID_EXPLORATION_CENTER_ON_ROBOT_BOX,
                                                      wxT("Center On Robot"),
                                                      wxDefaultPosition,
                                                      wxDefaultSize,
                                                      0);
    explorationCenterOnRobotCheckbox->SetValue(true);
    explorationOptionsSizer->Add(explorationCenterOnRobotCheckbox, 0, wxALL | wxEXPAND, 5);


    explorationPanelSizer->Add(explorationOptionsSizer, 0, wxEXPAND, 5);


    explorationPanel->SetSizer(explorationPanelSizer);
    explorationPanel->Layout();
    explorationPanelSizer->Fit(explorationPanel);
    frameNotebook->AddPage(explorationPanel, wxT("Exploration"), false, wxNullBitmap);
    relocalizationPanel = new wxPanel(frameNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxBoxSizer* relocalizationPanelSizer;
    relocalizationPanelSizer = new wxBoxSizer(wxHORIZONTAL);

    relocalizationWidget =
      new RelocalizationDisplayWidget(relocalizationPanel, ID_RELOCALIZATION_WIDGET, wxDefaultPosition, wxDefaultSize);
    relocalizationPanelSizer->Add(relocalizationWidget, 1, wxALL | wxEXPAND, 5);

    relocalizationScrollWindow =
      new wxScrolledWindow(relocalizationPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL);
    relocalizationScrollWindow->SetScrollRate(0, 5);
    wxGridBagSizer* relocalizationOptionsSizer;
    relocalizationOptionsSizer = new wxGridBagSizer(0, 0);
    relocalizationOptionsSizer->SetFlexibleDirection(wxBOTH);
    relocalizationOptionsSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* relocalizeDataSizer;
    relocalizeDataSizer =
      new wxStaticBoxSizer(new wxStaticBox(relocalizationScrollWindow, wxID_ANY, wxT("Data to Show:")), wxHORIZONTAL);

    relocalizeShowLaserCheckBox = new wxCheckBox(relocalizeDataSizer->GetStaticBox(),
                                                 ID_RELOCALIZE_SHOW_LASER_BOX,
                                                 wxT("Laser"),
                                                 wxDefaultPosition,
                                                 wxDefaultSize,
                                                 0);
    relocalizeShowLaserCheckBox->SetValue(true);
    relocalizeDataSizer->Add(relocalizeShowLaserCheckBox, 0, wxALL, 5);

    relocalizeShowErrorCheckBox = new wxCheckBox(relocalizeDataSizer->GetStaticBox(),
                                                 ID_RELOCALIZE_SHOW_ERROR_BOX,
                                                 wxT("Error"),
                                                 wxDefaultPosition,
                                                 wxDefaultSize,
                                                 0);
    relocalizeShowErrorCheckBox->SetValue(true);
    relocalizeDataSizer->Add(relocalizeShowErrorCheckBox, 0, wxALL, 5);

    relocalizeShowParticlesCheckBox = new wxCheckBox(relocalizeDataSizer->GetStaticBox(),
                                                     ID_RELOCALIZE_SHOW_PARTICLES_BOX,
                                                     wxT("Particles"),
                                                     wxDefaultPosition,
                                                     wxDefaultSize,
                                                     0);
    relocalizeShowParticlesCheckBox->SetValue(true);
    relocalizeDataSizer->Add(relocalizeShowParticlesCheckBox, 0, wxALL, 5);


    relocalizationOptionsSizer->Add(relocalizeDataSizer, wxGBPosition(2, 0), wxGBSpan(1, 2), wxEXPAND, 5);

    relocalizeLoadLPMButton = new wxButton(relocalizationScrollWindow,
                                           ID_RELOCALIZE_LOAD_LPM_BUTTON,
                                           wxT("Load LPM"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    relocalizationOptionsSizer->Add(relocalizeLoadLPMButton, wxGBPosition(1, 0), wxGBSpan(1, 1), wxALL, 5);

    relocalizeLoadGMMButton = new wxButton(relocalizationScrollWindow,
                                           ID_RELOCALIZE_LOAD_GMM_BUTTON,
                                           wxT("Load GMM"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    relocalizationOptionsSizer->Add(relocalizeLoadGMMButton,
                                    wxGBPosition(1, 1),
                                    wxGBSpan(1, 1),
                                    wxALIGN_CENTER | wxALIGN_LEFT | wxALL,
                                    5);

    wxString relocalizeModeRadioChoices[] = {wxT("Local"), wxT("Global")};
    int relocalizeModeRadioNChoices = sizeof(relocalizeModeRadioChoices) / sizeof(wxString);
    relocalizeModeRadio = new wxRadioBox(relocalizationScrollWindow,
                                         ID_RELOCALIZATION_MODE_RADIO,
                                         wxT("Relocalization Mode:"),
                                         wxDefaultPosition,
                                         wxDefaultSize,
                                         relocalizeModeRadioNChoices,
                                         relocalizeModeRadioChoices,
                                         1,
                                         wxRA_SPECIFY_ROWS);
    relocalizeModeRadio->SetSelection(0);
    relocalizationOptionsSizer->Add(relocalizeModeRadio, wxGBPosition(0, 0), wxGBSpan(1, 2), wxALL | wxEXPAND, 5);

    wxStaticBoxSizer* initializerOptionsSizer;
    initializerOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(relocalizationScrollWindow, wxID_ANY, wxT("Initializer Options:")),
                           wxVERTICAL);

    wxStaticBoxSizer* regionInitializerOptionsSizer;
    regionInitializerOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(initializerOptionsSizer->GetStaticBox(), wxID_ANY, wxT("Region:")),
                           wxVERTICAL);

    wxBoxSizer* regionNumSamplesSizer;
    regionNumSamplesSizer = new wxBoxSizer(wxHORIZONTAL);

    regionNumSamplesLabel = new wxStaticText(regionInitializerOptionsSizer->GetStaticBox(),
                                             wxID_ANY,
                                             wxT("Num Samples:"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    regionNumSamplesLabel->Wrap(-1);
    regionNumSamplesSizer->Add(regionNumSamplesLabel, 0, wxALIGN_CENTER | wxALL, 5);

    regionNumSamplesText = new wxTextCtrl(regionInitializerOptionsSizer->GetStaticBox(),
                                          wxID_ANY,
                                          wxT("30000"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    regionNumSamplesSizer->Add(regionNumSamplesText, 0, wxALL, 5);


    regionInitializerOptionsSizer->Add(regionNumSamplesSizer, 1, wxEXPAND, 5);

    wxBoxSizer* regionPosesPerPosSizer;
    regionPosesPerPosSizer = new wxBoxSizer(wxHORIZONTAL);

    regionPosesPerPosLabel = new wxStaticText(regionInitializerOptionsSizer->GetStaticBox(),
                                              wxID_ANY,
                                              wxT("Poses Per Position:"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    regionPosesPerPosLabel->Wrap(-1);
    regionPosesPerPosSizer->Add(regionPosesPerPosLabel, 0, wxALIGN_CENTER | wxALL, 5);

    regionPosesPerPositionText = new wxTextCtrl(regionInitializerOptionsSizer->GetStaticBox(),
                                                wxID_ANY,
                                                wxT("60"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    regionPosesPerPosSizer->Add(regionPosesPerPositionText, 0, wxALIGN_CENTER | wxALL, 5);


    regionInitializerOptionsSizer->Add(regionPosesPerPosSizer, 1, wxEXPAND, 5);

    setRegionRelocalizeButton = new wxToggleButton(regionInitializerOptionsSizer->GetStaticBox(),
                                                   ID_SET_REGION_RELOCALIZE_BUTTON,
                                                   wxT("Set Region"),
                                                   wxDefaultPosition,
                                                   wxDefaultSize,
                                                   0);
    setRegionRelocalizeButton->SetFont(
      wxFont(12, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    regionInitializerOptionsSizer->Add(setRegionRelocalizeButton, 0, wxALL | wxEXPAND, 5);

    sendRegionMessageButton = new wxButton(regionInitializerOptionsSizer->GetStaticBox(),
                                           ID_SEND_REGION_MESSAGE_BUTTON,
                                           wxT("Send Message"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    sendRegionMessageButton->SetFont(
      wxFont(14, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    regionInitializerOptionsSizer->Add(sendRegionMessageButton, 0, wxALL | wxEXPAND, 5);


    initializerOptionsSizer->Add(regionInitializerOptionsSizer, 0, wxEXPAND, 5);

    wxStaticBoxSizer* freeSpaceInitializerSizer;
    freeSpaceInitializerSizer =
      new wxStaticBoxSizer(new wxStaticBox(initializerOptionsSizer->GetStaticBox(), wxID_ANY, wxT("Free Space:")),
                           wxVERTICAL);

    wxBoxSizer* freeSpaceCellStrideSizer;
    freeSpaceCellStrideSizer = new wxBoxSizer(wxHORIZONTAL);

    freeSpaceCellStrideLabel = new wxStaticText(freeSpaceInitializerSizer->GetStaticBox(),
                                                wxID_ANY,
                                                wxT("Cell Stride:"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    freeSpaceCellStrideLabel->Wrap(-1);
    freeSpaceCellStrideSizer->Add(freeSpaceCellStrideLabel, 0, wxALIGN_CENTER | wxALL, 5);

    freeSpaceCellStrideText = new wxTextCtrl(freeSpaceInitializerSizer->GetStaticBox(),
                                             wxID_ANY,
                                             wxT("10"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    freeSpaceCellStrideSizer->Add(freeSpaceCellStrideText, 0, wxALIGN_CENTER | wxALL, 5);


    freeSpaceInitializerSizer->Add(freeSpaceCellStrideSizer, 0, wxEXPAND, 5);

    wxBoxSizer* freeSpacePosesPerCellSizer;
    freeSpacePosesPerCellSizer = new wxBoxSizer(wxHORIZONTAL);

    freeSpacePosesPerCellLabel = new wxStaticText(freeSpaceInitializerSizer->GetStaticBox(),
                                                  wxID_ANY,
                                                  wxT("Poses Per Cell:"),
                                                  wxDefaultPosition,
                                                  wxDefaultSize,
                                                  0);
    freeSpacePosesPerCellLabel->Wrap(-1);
    freeSpacePosesPerCellSizer->Add(freeSpacePosesPerCellLabel, 0, wxALIGN_CENTER | wxALL, 5);

    freeSpacePosesPerCellText = new wxTextCtrl(freeSpaceInitializerSizer->GetStaticBox(),
                                               wxID_ANY,
                                               wxT("10"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    freeSpacePosesPerCellSizer->Add(freeSpacePosesPerCellText, 0, wxALL, 5);


    freeSpaceInitializerSizer->Add(freeSpacePosesPerCellSizer, 1, wxEXPAND, 5);

    sendFreeSpaceMessageButton = new wxButton(freeSpaceInitializerSizer->GetStaticBox(),
                                              ID_SEND_FREE_SPACE_MESSAGE_BUTTON,
                                              wxT("Send Message"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    sendFreeSpaceMessageButton->SetFont(
      wxFont(14, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    freeSpaceInitializerSizer->Add(sendFreeSpaceMessageButton, 0, wxALL | wxEXPAND, 5);


    initializerOptionsSizer->Add(freeSpaceInitializerSizer, 0, wxEXPAND, 5);

    wxStaticBoxSizer* scanMatchingInitializerSizer;
    scanMatchingInitializerSizer =
      new wxStaticBoxSizer(new wxStaticBox(initializerOptionsSizer->GetStaticBox(), wxID_ANY, wxT("Scan Matching:")),
                           wxVERTICAL);

    sendScanMatchingMessageButton = new wxButton(scanMatchingInitializerSizer->GetStaticBox(),
                                                 ID_SEND_SCAN_MATCHING_MESSAGE_BUTTON,
                                                 wxT("Send Message"),
                                                 wxDefaultPosition,
                                                 wxDefaultSize,
                                                 0);
    sendScanMatchingMessageButton->SetFont(
      wxFont(14, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxEmptyString));

    scanMatchingInitializerSizer->Add(sendScanMatchingMessageButton, 0, wxALIGN_CENTER | wxALL | wxEXPAND, 5);


    initializerOptionsSizer->Add(scanMatchingInitializerSizer, 0, wxEXPAND, 5);


    relocalizationOptionsSizer->Add(initializerOptionsSizer, wxGBPosition(3, 0), wxGBSpan(1, 2), wxEXPAND, 5);


    relocalizationScrollWindow->SetSizer(relocalizationOptionsSizer);
    relocalizationScrollWindow->Layout();
    relocalizationOptionsSizer->Fit(relocalizationScrollWindow);
    relocalizationPanelSizer->Add(relocalizationScrollWindow, 0, wxEXPAND | wxALL, 5);


    relocalizationPanel->SetSizer(relocalizationPanelSizer);
    relocalizationPanel->Layout();
    relocalizationPanelSizer->Fit(relocalizationPanel);
    frameNotebook->AddPage(relocalizationPanel, wxT("Relocalization"), false, wxNullBitmap);
    plannerScriptingPanel = new wxPanel(frameNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxBoxSizer* scriptingPanelSizer;
    scriptingPanelSizer = new wxBoxSizer(wxHORIZONTAL);

    scriptingWidget =
      new PlannerScriptingWidget(plannerScriptingPanel, ID_PLANNER_SCRIPTING_WIDGET, wxDefaultPosition, wxDefaultSize);
    scriptingPanelSizer->Add(scriptingWidget, 1, wxALL | wxEXPAND, 5);

    scriptingOptionsScrolledWindow =
      new wxScrolledWindow(plannerScriptingPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxHSCROLL | wxVSCROLL);
    scriptingOptionsScrolledWindow->SetScrollRate(0, 5);
    wxBoxSizer* scriptingOptionsSizer;
    scriptingOptionsSizer = new wxBoxSizer(wxVERTICAL);

    wxStaticBoxSizer* scriptMapOptionsSizer;
    scriptMapOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(scriptingOptionsScrolledWindow, wxID_ANY, wxT("Map Options")), wxHORIZONTAL);

    scriptingLoadMapButton = new wxButton(scriptMapOptionsSizer->GetStaticBox(),
                                          ID_SCRIPTING_LOAD_MAP_BUTTON,
                                          wxT("Load"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    scriptMapOptionsSizer->Add(scriptingLoadMapButton, 0, wxALL, 5);

    scriptingCaptureMapButton = new wxButton(scriptMapOptionsSizer->GetStaticBox(),
                                             ID_SCRIPTING_CAPTURE_MAP_BUTTON,
                                             wxT("Capture"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    scriptMapOptionsSizer->Add(scriptingCaptureMapButton, 0, wxALL, 5);


    scriptingOptionsSizer->Add(scriptMapOptionsSizer, 0, wxEXPAND, 5);

    wxStaticBoxSizer* scriptingTargetsSizer;
    scriptingTargetsSizer =
      new wxStaticBoxSizer(new wxStaticBox(scriptingOptionsScrolledWindow, wxID_ANY, wxT("Target Options")),
                           wxVERTICAL);

    scriptingTargetSetList = new wxListCtrl(scriptingTargetsSizer->GetStaticBox(),
                                            ID_SCRIPTING_TARGET_SET_LIST,
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            wxLC_HRULES | wxLC_REPORT | wxLC_SINGLE_SEL);
    scriptingTargetsSizer->Add(scriptingTargetSetList, 1, wxALL | wxEXPAND, 5);

    wxFlexGridSizer* targetDescriptionSizer;
    targetDescriptionSizer = new wxFlexGridSizer(0, 2, 0, 0);
    targetDescriptionSizer->SetFlexibleDirection(wxBOTH);
    targetDescriptionSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    scriptTargetNameLabel = new wxStaticText(scriptingTargetsSizer->GetStaticBox(),
                                             wxID_ANY,
                                             wxT("Name:"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    scriptTargetNameLabel->Wrap(-1);
    targetDescriptionSizer->Add(scriptTargetNameLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    scriptTargetNameText = new wxTextCtrl(scriptingTargetsSizer->GetStaticBox(),
                                          wxID_ANY,
                                          wxEmptyString,
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    targetDescriptionSizer->Add(scriptTargetNameText, 1, wxALL | wxEXPAND, 5);

    scriptTargetTypeLael = new wxStaticText(scriptingTargetsSizer->GetStaticBox(),
                                            wxID_ANY,
                                            wxT("Type:"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    scriptTargetTypeLael->Wrap(-1);
    targetDescriptionSizer->Add(scriptTargetTypeLael, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    wxBoxSizer* scriptTargetTypeRadioSizer;
    scriptTargetTypeRadioSizer = new wxBoxSizer(wxHORIZONTAL);

    scriptPoseTargetButton = new wxRadioButton(scriptingTargetsSizer->GetStaticBox(),
                                               ID_SCRIPT_POSE_TARGET_BUTTON,
                                               wxT("Pose"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               wxRB_GROUP);
    scriptPoseTargetButton->SetValue(true);
    scriptTargetTypeRadioSizer->Add(scriptPoseTargetButton, 0, wxALL, 5);

    scriptElevatorTargetButton = new wxRadioButton(scriptingTargetsSizer->GetStaticBox(),
                                                   ID_SCRIPT_ELEVATOR_TARGET_BUTTON,
                                                   wxT("Elevator"),
                                                   wxDefaultPosition,
                                                   wxDefaultSize,
                                                   0);
    scriptTargetTypeRadioSizer->Add(scriptElevatorTargetButton, 0, wxALL, 5);


    targetDescriptionSizer->Add(scriptTargetTypeRadioSizer, 1, wxEXPAND, 5);

    scriptTargetPoseLabel = new wxStaticText(scriptingTargetsSizer->GetStaticBox(),
                                             wxID_ANY,
                                             wxT("Pose:"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    scriptTargetPoseLabel->Wrap(-1);
    targetDescriptionSizer->Add(scriptTargetPoseLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    scriptTargetPoseText = new wxTextCtrl(scriptingTargetsSizer->GetStaticBox(),
                                          ID_SCRIPT_TARGET_POSE_TEXT,
                                          wxEmptyString,
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          wxTE_READONLY);
    targetDescriptionSizer->Add(scriptTargetPoseText, 0, wxALL | wxEXPAND, 5);

    scriptTargetSelectPoseButton = new wxButton(scriptingTargetsSizer->GetStaticBox(),
                                                ID_SCRIPT_TARGET_SELECT_POSE_BUTTON,
                                                wxT("Select Pose"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    targetDescriptionSizer->Add(scriptTargetSelectPoseButton,
                                0,
                                wxALIGN_CENTER | wxALIGN_CENTER_VERTICAL | wxALL | wxEXPAND,
                                5);

    scriptTargetCurrentButton = new wxButton(scriptingTargetsSizer->GetStaticBox(),
                                             ID_SCRIPT_TARGET_CURRENT_BUTTON,
                                             wxT("Current Pose"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    targetDescriptionSizer->Add(scriptTargetCurrentButton, 0, wxALL, 5);


    scriptingTargetsSizer->Add(targetDescriptionSizer, 0, wxEXPAND, 5);

    wxGridSizer* scriptTargetCreationSizer;
    scriptTargetCreationSizer = new wxGridSizer(2, 2, 0, 0);

    scriptCreateTargetButton = new wxButton(scriptingTargetsSizer->GetStaticBox(),
                                            ID_SCRIPT_CREATE_TARGET_BUTTON,
                                            wxT("Add"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    scriptTargetCreationSizer->Add(scriptCreateTargetButton, 0, wxALL, 5);

    scriptEraseTargetButton = new wxButton(scriptingTargetsSizer->GetStaticBox(),
                                           ID_SCRIPT_ERASE_TARGET_BUTTON,
                                           wxT("Remove"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    scriptTargetCreationSizer->Add(scriptEraseTargetButton, 0, wxALL, 5);

    scriptSaveTargetsButton = new wxButton(scriptingTargetsSizer->GetStaticBox(),
                                           ID_SCRIPT_SAVE_TARGETS_BUTTON,
                                           wxT("Save"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    scriptTargetCreationSizer->Add(scriptSaveTargetsButton, 0, wxALL, 5);

    scriptLoadTargetsButton = new wxButton(scriptingTargetsSizer->GetStaticBox(),
                                           ID_SCRIPT_LOAD_TARGETS_BUTTON,
                                           wxT("Load"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    scriptTargetCreationSizer->Add(scriptLoadTargetsButton, 0, wxALL, 5);


    scriptingTargetsSizer->Add(scriptTargetCreationSizer, 0, wxALIGN_CENTER, 5);


    scriptingOptionsSizer->Add(scriptingTargetsSizer, 0, wxEXPAND, 5);

    wxStaticBoxSizer* scriptOptionsSizer;
    scriptOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(scriptingOptionsScrolledWindow, wxID_ANY, wxT("Script Options")),
                           wxVERTICAL);

    scriptTargetsList = new wxListBox(scriptOptionsSizer->GetStaticBox(),
                                      ID_SCRIPT_TARGETS_LIST,
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      0,
                                      NULL,
                                      wxLB_SINGLE);
    scriptOptionsSizer->Add(scriptTargetsList, 1, wxALL | wxEXPAND, 5);

    wxGridSizer* scriptTargetAdditionSizer;
    scriptTargetAdditionSizer = new wxGridSizer(2, 2, 0, 0);

    scriptAddTargetButton = new wxButton(scriptOptionsSizer->GetStaticBox(),
                                         ID_SCRIPT_ADD_TARGET_BUTTON,
                                         wxT("Add"),
                                         wxDefaultPosition,
                                         wxDefaultSize,
                                         0);
    scriptTargetAdditionSizer->Add(scriptAddTargetButton, 0, wxALL, 5);

    scriptRemoveTargetButton = new wxButton(scriptOptionsSizer->GetStaticBox(),
                                            ID_SCRIPT_REMOVE_TARGET_BUTTON,
                                            wxT("Remove"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    scriptTargetAdditionSizer->Add(scriptRemoveTargetButton, 0, wxALL, 5);

    scriptSaveButton = new wxButton(scriptOptionsSizer->GetStaticBox(),
                                    ID_SCRIPT_SAVE_BUTTON,
                                    wxT("Save"),
                                    wxDefaultPosition,
                                    wxDefaultSize,
                                    0);
    scriptTargetAdditionSizer->Add(scriptSaveButton, 0, wxALL, 5);

    scriptLoadButton = new wxButton(scriptOptionsSizer->GetStaticBox(),
                                    ID_SCRIPT_LOAD_BUTTON,
                                    wxT("Load"),
                                    wxDefaultPosition,
                                    wxDefaultSize,
                                    0);
    scriptTargetAdditionSizer->Add(scriptLoadButton, 0, wxALL, 5);


    scriptOptionsSizer->Add(scriptTargetAdditionSizer, 0, wxEXPAND, 5);

    scriptSendButton = new wxButton(scriptOptionsSizer->GetStaticBox(),
                                    ID_SCRIPT_SEND_BUTTON,
                                    wxT("Send"),
                                    wxDefaultPosition,
                                    wxDefaultSize,
                                    0);
    scriptOptionsSizer->Add(scriptSendButton, 0, wxALL | wxEXPAND, 5);


    scriptingOptionsSizer->Add(scriptOptionsSizer, 1, wxEXPAND, 5);


    scriptingOptionsScrolledWindow->SetSizer(scriptingOptionsSizer);
    scriptingOptionsScrolledWindow->Layout();
    scriptingOptionsSizer->Fit(scriptingOptionsScrolledWindow);
    scriptingPanelSizer->Add(scriptingOptionsScrolledWindow, 0, wxALL | wxEXPAND, 5);


    plannerScriptingPanel->SetSizer(scriptingPanelSizer);
    plannerScriptingPanel->Layout();
    scriptingPanelSizer->Fit(plannerScriptingPanel);
    frameNotebook->AddPage(plannerScriptingPanel, wxT("Scripting"), false, wxNullBitmap);
    globalMetricPanel = new wxPanel(frameNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxBoxSizer* globalMetricPanelSizer;
    globalMetricPanelSizer = new wxBoxSizer(wxHORIZONTAL);

    globalMetricWidget =
      new GlobalMetricDisplayWidget(globalMetricPanel, ID_GLOBAL_METRIC_WIDGET, wxDefaultPosition, wxDefaultSize);
    globalMetricPanelSizer->Add(globalMetricWidget, 1, wxALL | wxEXPAND, 5);

    globalMetricScrolledWindow =
      new wxScrolledWindow(globalMetricPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxHSCROLL | wxVSCROLL);
    globalMetricScrolledWindow->SetScrollRate(5, 5);
    wxBoxSizer* globalMetricOptionsSizer;
    globalMetricOptionsSizer = new wxBoxSizer(wxVERTICAL);

    wxStaticBoxSizer* globalMetricLoadOptions;
    globalMetricLoadOptions =
      new wxStaticBoxSizer(new wxStaticBox(globalMetricScrolledWindow, wxID_ANY, wxT("Loading Options:")), wxVERTICAL);

    globalMetricCaptureLPMButton = new wxButton(globalMetricLoadOptions->GetStaticBox(),
                                                ID_GLOBAL_METRIC_CAPTURE_LPM_BUTTON,
                                                wxT("Capture LPM"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    globalMetricLoadOptions->Add(globalMetricCaptureLPMButton, 0, wxALL | wxEXPAND, 5);

    loadLPMForGlobalMetricButton = new wxButton(globalMetricLoadOptions->GetStaticBox(),
                                                ID_LOAD_LPM_FOR_GLOBAL_METRIC_BUTTON,
                                                wxT("Load LPM"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    globalMetricLoadOptions->Add(loadLPMForGlobalMetricButton, 0, wxALL | wxEXPAND, 5);

    loadGlobalMetricMapButton = new wxButton(globalMetricLoadOptions->GetStaticBox(),
                                             ID_LOAD_GLOBAL_METRIC_MAP_BUTTON,
                                             wxT("Local Global Map"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    globalMetricLoadOptions->Add(loadGlobalMetricMapButton, 0, wxALL | wxEXPAND, 5);


    globalMetricOptionsSizer->Add(globalMetricLoadOptions, 0, wxALL | wxEXPAND, 5);

    wxStaticBoxSizer* globalMetricSaveOptions;
    globalMetricSaveOptions =
      new wxStaticBoxSizer(new wxStaticBox(globalMetricScrolledWindow, wxID_ANY, wxT("Save Options:")), wxVERTICAL);

    globalMetricMapNameText = new wxTextCtrl(globalMetricSaveOptions->GetStaticBox(),
                                             ID_GLOBAL_METRIC_MAP_NAME_TEXT,
                                             wxT("Map Name"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    globalMetricSaveOptions->Add(globalMetricMapNameText, 0, wxALL | wxEXPAND, 5);

    saveGlobalMapButton = new wxButton(globalMetricSaveOptions->GetStaticBox(),
                                       ID_SAVE_GLOBAL_MAP_BUTTON,
                                       wxT("Save Global Map"),
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0);
    globalMetricSaveOptions->Add(saveGlobalMapButton, 0, wxALL | wxEXPAND, 5);


    globalMetricOptionsSizer->Add(globalMetricSaveOptions, 0, wxEXPAND, 5);

    globalMetricRelocalizeButton = new wxButton(globalMetricScrolledWindow,
                                                ID_GLOBAL_METRIC_RELOCALIZE_BUTTON,
                                                wxT("Relocalize!"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    globalMetricRelocalizeButton->SetFont(
      wxFont(16, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL, false, wxT("Sans")));

    globalMetricOptionsSizer->Add(globalMetricRelocalizeButton, 0, wxALL | wxEXPAND, 5);


    globalMetricScrolledWindow->SetSizer(globalMetricOptionsSizer);
    globalMetricScrolledWindow->Layout();
    globalMetricOptionsSizer->Fit(globalMetricScrolledWindow);
    globalMetricPanelSizer->Add(globalMetricScrolledWindow, 0, wxALL | wxEXPAND | wxFIXED_MINSIZE, 5);


    globalMetricPanel->SetSizer(globalMetricPanelSizer);
    globalMetricPanel->Layout();
    globalMetricPanelSizer->Fit(globalMetricPanel);
    frameNotebook->AddPage(globalMetricPanel, wxT("Global Metric"), false, wxNullBitmap);
    calibrationPanel = new wxPanel(frameNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxBoxSizer* calibrationPanelSizer;
    calibrationPanelSizer = new wxBoxSizer(wxHORIZONTAL);

    wxBoxSizer* calibrationWidgetSizer;
    calibrationWidgetSizer = new wxBoxSizer(wxHORIZONTAL);

    calibrationWidget =
      new CalibrationDisplayWidget(calibrationPanel, ID_CALIBRATION_WIDGET, wxDefaultPosition, wxDefaultSize);
    calibrationWidget->SetSize(wxSize(800, 600));
    calibrationWidget->SetMinSize(wxSize(0, 0));

    calibrationWidgetSizer->Add(calibrationWidget, 1, wxALIGN_CENTER | wxEXPAND, 0);


    calibrationPanelSizer->Add(calibrationWidgetSizer, 1, wxALL | wxEXPAND, 5);

    calibrationScrollWindow =
      new wxScrolledWindow(calibrationPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxHSCROLL | wxVSCROLL);
    calibrationScrollWindow->SetScrollRate(0, 5);
    wxFlexGridSizer* calibrationOptionsSizer;
    calibrationOptionsSizer = new wxFlexGridSizer(0, 1, 0, 0);
    calibrationOptionsSizer->SetFlexibleDirection(wxBOTH);
    calibrationOptionsSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* laserOptionsSizer;
    laserOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(calibrationScrollWindow, wxID_ANY, wxT("Laser Options")), wxHORIZONTAL);

    wxFlexGridSizer* laserOptionsFlexGridSizer;
    laserOptionsFlexGridSizer = new wxFlexGridSizer(0, 2, 0, 0);
    laserOptionsFlexGridSizer->SetFlexibleDirection(wxBOTH);
    laserOptionsFlexGridSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    showFrontLaserCheckBox = new wxCheckBox(laserOptionsSizer->GetStaticBox(),
                                            ID_SHOW_FRONT_LASER_BOX,
                                            wxT("Show Front Laser"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    showFrontLaserCheckBox->SetValue(true);
    laserOptionsFlexGridSizer->Add(showFrontLaserCheckBox, 0, wxALL, 5);

    showBackLaserCheckBox = new wxCheckBox(laserOptionsSizer->GetStaticBox(),
                                           ID_SHOW_BACK_LASER_BOX,
                                           wxT("Show Back Laser"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    showBackLaserCheckBox->SetValue(true);
    laserOptionsFlexGridSizer->Add(showBackLaserCheckBox, 0, wxALL, 5);

    wxFlexGridSizer* frontLaserCoords;
    frontLaserCoords = new wxFlexGridSizer(0, 2, 0, 0);
    frontLaserCoords->SetFlexibleDirection(wxBOTH);
    frontLaserCoords->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    frontLaserCoordsXLabel =
      new wxStaticText(laserOptionsSizer->GetStaticBox(), wxID_ANY, wxT("  x:"), wxDefaultPosition, wxDefaultSize, 0);
    frontLaserCoordsXLabel->Wrap(-1);
    frontLaserCoords->Add(frontLaserCoordsXLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    frontLaserCoordsXText = new wxTextCtrl(laserOptionsSizer->GetStaticBox(),
                                           ID_FRONT_LASER_COORDS_X,
                                           wxEmptyString,
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    frontLaserCoords->Add(frontLaserCoordsXText, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    frontLaserCoordsYLabel =
      new wxStaticText(laserOptionsSizer->GetStaticBox(), wxID_ANY, wxT("  y:"), wxDefaultPosition, wxDefaultSize, 0);
    frontLaserCoordsYLabel->Wrap(-1);
    frontLaserCoords->Add(frontLaserCoordsYLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    frontLaserCoordsYText = new wxTextCtrl(laserOptionsSizer->GetStaticBox(),
                                           ID_FRONT_LASER_COORDS_Y,
                                           wxEmptyString,
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    frontLaserCoords->Add(frontLaserCoordsYText, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    frontLaserCoordsThetaLabel =
      new wxStaticText(laserOptionsSizer->GetStaticBox(), wxID_ANY, wxT("  θ:"), wxDefaultPosition, wxDefaultSize, 0);
    frontLaserCoordsThetaLabel->Wrap(-1);
    frontLaserCoords->Add(frontLaserCoordsThetaLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    frontLaserCoordsThetaText = new wxTextCtrl(laserOptionsSizer->GetStaticBox(),
                                               ID_FRONT_LASER_COORDS_THETA,
                                               wxEmptyString,
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    frontLaserCoords->Add(frontLaserCoordsThetaText, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    frontLaserPitchLabel =
      new wxStaticText(laserOptionsSizer->GetStaticBox(), wxID_ANY, wxT("Pitch:"), wxDefaultPosition, wxDefaultSize, 0);
    frontLaserPitchLabel->Wrap(-1);
    frontLaserCoords->Add(frontLaserPitchLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    frontLaserPitchText = new wxTextCtrl(laserOptionsSizer->GetStaticBox(),
                                         ID_FRONT_LASER_PITCH_TEXT,
                                         wxEmptyString,
                                         wxDefaultPosition,
                                         wxDefaultSize,
                                         0);
    frontLaserCoords->Add(frontLaserPitchText, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    frontLaserRollLabel =
      new wxStaticText(laserOptionsSizer->GetStaticBox(), wxID_ANY, wxT("Roll:"), wxDefaultPosition, wxDefaultSize, 0);
    frontLaserRollLabel->Wrap(-1);
    frontLaserCoords->Add(frontLaserRollLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    frontLaserRollText = new wxTextCtrl(laserOptionsSizer->GetStaticBox(),
                                        ID_FRONT_LASER_ROLL_TEXT,
                                        wxEmptyString,
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        0);
    frontLaserCoords->Add(frontLaserRollText, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);


    laserOptionsFlexGridSizer->Add(frontLaserCoords, 1, wxEXPAND, 5);

    wxFlexGridSizer* BackLaserCoords;
    BackLaserCoords = new wxFlexGridSizer(0, 2, 0, 0);
    BackLaserCoords->SetFlexibleDirection(wxBOTH);
    BackLaserCoords->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    backLaserCoordsXLabel =
      new wxStaticText(laserOptionsSizer->GetStaticBox(), wxID_ANY, wxT("  x:"), wxDefaultPosition, wxDefaultSize, 0);
    backLaserCoordsXLabel->Wrap(-1);
    BackLaserCoords->Add(backLaserCoordsXLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    backLaserCoordsXText = new wxTextCtrl(laserOptionsSizer->GetStaticBox(),
                                          ID_BACK_LASER_COORDS_X,
                                          wxEmptyString,
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    BackLaserCoords->Add(backLaserCoordsXText, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    backLaserCoordsYLabel =
      new wxStaticText(laserOptionsSizer->GetStaticBox(), wxID_ANY, wxT("  y:"), wxDefaultPosition, wxDefaultSize, 0);
    backLaserCoordsYLabel->Wrap(-1);
    BackLaserCoords->Add(backLaserCoordsYLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    backLaserCoordsYText = new wxTextCtrl(laserOptionsSizer->GetStaticBox(),
                                          ID_BACK_LASER_COORDS_Y,
                                          wxEmptyString,
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    BackLaserCoords->Add(backLaserCoordsYText, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    backLaserCoordsThetaLabel =
      new wxStaticText(laserOptionsSizer->GetStaticBox(), wxID_ANY, wxT("  θ:"), wxDefaultPosition, wxDefaultSize, 0);
    backLaserCoordsThetaLabel->Wrap(-1);
    BackLaserCoords->Add(backLaserCoordsThetaLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    backLaserCoordsThetaText = new wxTextCtrl(laserOptionsSizer->GetStaticBox(),
                                              ID_BACK_LASER_COORDS_THETA,
                                              wxEmptyString,
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    BackLaserCoords->Add(backLaserCoordsThetaText, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    backLaserPitchLabel =
      new wxStaticText(laserOptionsSizer->GetStaticBox(), wxID_ANY, wxT("Pitch:"), wxDefaultPosition, wxDefaultSize, 0);
    backLaserPitchLabel->Wrap(-1);
    BackLaserCoords->Add(backLaserPitchLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    backLaserPitchText = new wxTextCtrl(laserOptionsSizer->GetStaticBox(),
                                        ID_BACK_LASER_PITCH_TEXT,
                                        wxEmptyString,
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        0);
    BackLaserCoords->Add(backLaserPitchText, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    backLaserRollLabel =
      new wxStaticText(laserOptionsSizer->GetStaticBox(), wxID_ANY, wxT("Roll:"), wxDefaultPosition, wxDefaultSize, 0);
    backLaserRollLabel->Wrap(-1);
    BackLaserCoords->Add(backLaserRollLabel, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, 5);

    backLaserRollText = new wxTextCtrl(laserOptionsSizer->GetStaticBox(),
                                       ID_BACK_LASER_ROLL_TEXT,
                                       wxEmptyString,
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0);
    BackLaserCoords->Add(backLaserRollText, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);


    laserOptionsFlexGridSizer->Add(BackLaserCoords, 1, wxEXPAND, 5);


    laserOptionsSizer->Add(laserOptionsFlexGridSizer, 1, wxEXPAND, 5);


    calibrationOptionsSizer->Add(laserOptionsSizer, 1, wxALIGN_RIGHT | wxEXPAND, 5);

    wxStaticBoxSizer* tiltCalibrationSizer;
    tiltCalibrationSizer =
      new wxStaticBoxSizer(new wxStaticBox(calibrationScrollWindow, wxID_ANY, wxT("Tilt Calibration")), wxVERTICAL);

    loadTiltDataButton = new wxButton(tiltCalibrationSizer->GetStaticBox(),
                                      ID_LOAD_TILT_DATA_BUTTON,
                                      wxT("Load Tilt Data"),
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      0);
    tiltCalibrationSizer->Add(loadTiltDataButton, 0, wxALIGN_CENTER | wxALL | wxEXPAND, 5);

    wxString tiltLaserRadioChoices[] = {wxT("Front"), wxT("Back")};
    int tiltLaserRadioNChoices = sizeof(tiltLaserRadioChoices) / sizeof(wxString);
    tiltLaserRadio = new wxRadioBox(tiltCalibrationSizer->GetStaticBox(),
                                    ID_TILT_LASER_RADIO,
                                    wxT("Laser to Calibrate:"),
                                    wxDefaultPosition,
                                    wxDefaultSize,
                                    tiltLaserRadioNChoices,
                                    tiltLaserRadioChoices,
                                    1,
                                    wxRA_SPECIFY_COLS);
    tiltLaserRadio->SetSelection(0);
    tiltCalibrationSizer->Add(tiltLaserRadio, 0, wxALL | wxEXPAND, 5);

    wxBoxSizer* lineStartIndexSizer;
    lineStartIndexSizer = new wxBoxSizer(wxHORIZONTAL);

    lineStartIndexLabel = new wxStaticText(tiltCalibrationSizer->GetStaticBox(),
                                           wxID_ANY,
                                           wxT("Line start index:"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    lineStartIndexLabel->Wrap(-1);
    lineStartIndexSizer->Add(lineStartIndexLabel, 1, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    lineStartIndexText = new wxTextCtrl(tiltCalibrationSizer->GetStaticBox(),
                                        ID_LINE_START_INDEX_TEXT,
                                        wxT("-1"),
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        0);
    lineStartIndexSizer->Add(lineStartIndexText, 1, wxALL, 5);


    tiltCalibrationSizer->Add(lineStartIndexSizer, 1, wxEXPAND, 5);

    wxBoxSizer* lineEndIndexSizer;
    lineEndIndexSizer = new wxBoxSizer(wxHORIZONTAL);

    lineEndIndexLabel = new wxStaticText(tiltCalibrationSizer->GetStaticBox(),
                                         wxID_ANY,
                                         wxT("Line end index:"),
                                         wxDefaultPosition,
                                         wxDefaultSize,
                                         0);
    lineEndIndexLabel->Wrap(-1);
    lineEndIndexSizer->Add(lineEndIndexLabel, 1, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    lineEndIndexText = new wxTextCtrl(tiltCalibrationSizer->GetStaticBox(),
                                      ID_LINE_END_INDEX_TEXT,
                                      wxT("-1"),
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      0);
    lineEndIndexSizer->Add(lineEndIndexText, 1, wxALL, 5);


    tiltCalibrationSizer->Add(lineEndIndexSizer, 1, wxEXPAND, 5);

    calibratePitchButton = new wxButton(tiltCalibrationSizer->GetStaticBox(),
                                        ID_CALIBRATE_PITCH_BUTTON,
                                        wxT("Calibrate Pitch"),
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        0);
    tiltCalibrationSizer->Add(calibratePitchButton, 0, wxALIGN_CENTER | wxALL | wxEXPAND, 5);

    calibrateRollButton = new wxButton(tiltCalibrationSizer->GetStaticBox(),
                                       ID_CALIBRATE_ROLL_BUTTON,
                                       wxT("Calibrate Roll"),
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0);
    tiltCalibrationSizer->Add(calibrateRollButton, 0, wxALL | wxEXPAND, 5);


    calibrationOptionsSizer->Add(tiltCalibrationSizer, 1, wxEXPAND, 5);


    calibrationScrollWindow->SetSizer(calibrationOptionsSizer);
    calibrationScrollWindow->Layout();
    calibrationOptionsSizer->Fit(calibrationScrollWindow);
    calibrationPanelSizer->Add(calibrationScrollWindow, 0, wxALL | wxEXPAND, 5);


    calibrationPanel->SetSizer(calibrationPanelSizer);
    calibrationPanel->Layout();
    calibrationPanelSizer->Fit(calibrationPanel);
    frameNotebook->AddPage(calibrationPanel, wxT("Calibration"), false, wxNullBitmap);
    decisionPlannerPanel = new wxPanel(frameNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    decisionPlannerPanel->Enable(false);
    decisionPlannerPanel->Hide();

    wxBoxSizer* decisionPlannerPanelSizer;
    decisionPlannerPanelSizer = new wxBoxSizer(wxHORIZONTAL);

    wxBoxSizer* decisionPlannerWidgetSizer;
    decisionPlannerWidgetSizer = new wxBoxSizer(wxVERTICAL);

    decisionPlannerWidget = new DecisionPlannerDisplayWidget(decisionPlannerPanel,
                                                             ID_DECISION_PLANNER_WIDGET,
                                                             wxDefaultPosition,
                                                             wxDefaultSize);
    decisionPlannerWidgetSizer->Add(decisionPlannerWidget, 1, wxALL | wxEXPAND, 5);


    decisionPlannerPanelSizer->Add(decisionPlannerWidgetSizer, 1, wxEXPAND, 5);

    decisionPlannerScrollWindow =
      new wxScrolledWindow(decisionPlannerPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL);
    decisionPlannerScrollWindow->SetScrollRate(0, 5);
    wxFlexGridSizer* decisionPlannerOptionsSizer;
    decisionPlannerOptionsSizer = new wxFlexGridSizer(0, 1, 0, 0);
    decisionPlannerOptionsSizer->SetFlexibleDirection(wxBOTH);
    decisionPlannerOptionsSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    decisionCommandQueueList = new wxListBox(decisionPlannerScrollWindow,
                                             ID_DECISION_COMMAND_QUEUE_LIST,
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0,
                                             NULL,
                                             wxLB_EXTENDED | wxLB_MULTIPLE | wxLB_NEEDED_SB);
    decisionCommandQueueList->Append(wxT("1"));
    decisionCommandQueueList->Append(wxT("2"));
    decisionCommandQueueList->Append(wxT("3"));
    decisionCommandQueueList->Append(wxT("4"));
    decisionCommandQueueList->Append(wxT("5"));
    decisionCommandQueueList->Append(wxT("6"));
    decisionCommandQueueList->Append(wxT("7"));
    decisionCommandQueueList->Append(wxT("8"));
    decisionCommandQueueList->Append(wxT("9"));
    decisionCommandQueueList->Append(wxT("10"));
    decisionPlannerOptionsSizer->Add(decisionCommandQueueList, 1, wxALL | wxEXPAND, 5);

    wxStaticBoxSizer* localPlaceStateSizer;
    localPlaceStateSizer =
      new wxStaticBoxSizer(new wxStaticBox(decisionPlannerScrollWindow, wxID_ANY, wxT("Local Place State:")),
                           wxVERTICAL);

    localPlaceStateGrid =
      new wxGrid(localPlaceStateSizer->GetStaticBox(), wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);

    // Grid
    localPlaceStateGrid->CreateGrid(3, 1);
    localPlaceStateGrid->EnableEditing(true);
    localPlaceStateGrid->EnableGridLines(true);
    localPlaceStateGrid->EnableDragGridSize(false);
    localPlaceStateGrid->SetMargins(0, 0);

    // Columns
    localPlaceStateGrid->EnableDragColMove(false);
    localPlaceStateGrid->EnableDragColSize(true);
    localPlaceStateGrid->SetColLabelSize(1);
    localPlaceStateGrid->SetColLabelAlignment(wxALIGN_CENTER, wxALIGN_CENTER);

    // Rows
    localPlaceStateGrid->AutoSizeRows();
    localPlaceStateGrid->EnableDragRowSize(true);
    localPlaceStateGrid->SetRowLabelValue(0, wxT("ID:"));
    localPlaceStateGrid->SetRowLabelValue(1, wxT("Entry:"));
    localPlaceStateGrid->SetRowLabelValue(2, wxT("Exit:"));
    localPlaceStateGrid->SetRowLabelSize(80);
    localPlaceStateGrid->SetRowLabelAlignment(wxALIGN_CENTER, wxALIGN_CENTER);

    // Label Appearance

    // Cell Defaults
    localPlaceStateGrid->SetDefaultCellAlignment(wxALIGN_LEFT, wxALIGN_TOP);
    localPlaceStateSizer->Add(localPlaceStateGrid, 0, wxALIGN_CENTER | wxALL, 5);


    decisionPlannerOptionsSizer->Add(localPlaceStateSizer, 1, wxEXPAND, 5);

    wxStaticBoxSizer* localPathStateSizer;
    localPathStateSizer =
      new wxStaticBoxSizer(new wxStaticBox(decisionPlannerScrollWindow, wxID_ANY, wxT("Local Path State:")),
                           wxVERTICAL);

    localPathStateGrid = new wxGrid(localPathStateSizer->GetStaticBox(), wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);

    // Grid
    localPathStateGrid->CreateGrid(3, 1);
    localPathStateGrid->EnableEditing(true);
    localPathStateGrid->EnableGridLines(true);
    localPathStateGrid->EnableDragGridSize(false);
    localPathStateGrid->SetMargins(0, 0);

    // Columns
    localPathStateGrid->EnableDragColMove(false);
    localPathStateGrid->EnableDragColSize(true);
    localPathStateGrid->SetColLabelSize(1);
    localPathStateGrid->SetColLabelAlignment(wxALIGN_CENTER, wxALIGN_CENTER);

    // Rows
    localPathStateGrid->EnableDragRowSize(true);
    localPathStateGrid->SetRowLabelValue(0, wxT("Start:"));
    localPathStateGrid->SetRowLabelValue(1, wxT("End:"));
    localPathStateGrid->SetRowLabelValue(2, wxT("Direction:"));
    localPathStateGrid->SetRowLabelSize(80);
    localPathStateGrid->SetRowLabelAlignment(wxALIGN_CENTER, wxALIGN_CENTER);

    // Label Appearance

    // Cell Defaults
    localPathStateGrid->SetDefaultCellAlignment(wxALIGN_RIGHT, wxALIGN_CENTER);
    localPathStateSizer->Add(localPathStateGrid, 0, wxALIGN_CENTER | wxALL, 5);


    decisionPlannerOptionsSizer->Add(localPathStateSizer, 1, wxEXPAND, 5);

    wxStaticBoxSizer* relativeTargetSizer;
    relativeTargetSizer =
      new wxStaticBoxSizer(new wxStaticBox(decisionPlannerScrollWindow, wxID_ANY, wxT("Build Target Sequence:")),
                           wxVERTICAL);

    forwardTargetButton = new wxButton(relativeTargetSizer->GetStaticBox(),
                                       ID_FORWARD_TARGET_BUTTON,
                                       wxT("Forward"),
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0);
    relativeTargetSizer->Add(forwardTargetButton, 0, wxALIGN_CENTER | wxALL, 5);

    wxGridSizer* leftRightTargetSizer;
    leftRightTargetSizer = new wxGridSizer(1, 2, 0, 0);

    leftTargetButton = new wxButton(relativeTargetSizer->GetStaticBox(),
                                    ID_LEFT_TARGET_BUTTON,
                                    wxT("Left"),
                                    wxDefaultPosition,
                                    wxDefaultSize,
                                    0);
    leftRightTargetSizer->Add(leftTargetButton, 0, wxALIGN_RIGHT | wxALL, 5);

    rightTargetButton = new wxButton(relativeTargetSizer->GetStaticBox(),
                                     ID_RIGHT_TARGET_BUTTON,
                                     wxT("Right"),
                                     wxDefaultPosition,
                                     wxDefaultSize,
                                     0);
    leftRightTargetSizer->Add(rightTargetButton, 0, wxALIGN_LEFT | wxALL, 5);


    relativeTargetSizer->Add(leftRightTargetSizer, 1, wxEXPAND, 5);

    backTargetButton = new wxButton(relativeTargetSizer->GetStaticBox(),
                                    ID_BACK_TARGET_BUTTON,
                                    wxT("Back"),
                                    wxDefaultPosition,
                                    wxDefaultSize,
                                    0);
    relativeTargetSizer->Add(backTargetButton, 0, wxALIGN_CENTER | wxALL, 5);

    wxGridSizer* pathTargetSizer;
    pathTargetSizer = new wxGridSizer(1, 2, 0, 0);

    pathTargetStartButton = new wxButton(relativeTargetSizer->GetStaticBox(),
                                         ID_PATH_TARGET_START_BUTTON,
                                         wxT("Path Start"),
                                         wxDefaultPosition,
                                         wxDefaultSize,
                                         0);
    pathTargetSizer->Add(pathTargetStartButton, 0, wxALIGN_RIGHT | wxALL, 5);

    pathTargetEndButton = new wxButton(relativeTargetSizer->GetStaticBox(),
                                       ID_PATH_TARGET_END_BUTTON,
                                       wxT("Path End"),
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0);
    pathTargetSizer->Add(pathTargetEndButton, 0, wxALIGN_LEFT | wxALL, 5);


    relativeTargetSizer->Add(pathTargetSizer, 1, wxEXPAND, 5);

    wxGridSizer* targetSequenceOptionsSizer;
    targetSequenceOptionsSizer = new wxGridSizer(2, 3, 0, 0);

    removeRelativeTargetButton = new wxButton(relativeTargetSizer->GetStaticBox(),
                                              ID_REMOVE_RELATIVE_TARGET_BUTTON,
                                              wxT("Remove"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    targetSequenceOptionsSizer->Add(removeRelativeTargetButton, 0, wxALL, 5);

    clearRelativeTargetsButton = new wxButton(relativeTargetSizer->GetStaticBox(),
                                              ID_CLEAR_RELATIVE_TARGETS_BUTTON,
                                              wxT("Clear"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    targetSequenceOptionsSizer->Add(clearRelativeTargetsButton, 0, wxALL, 5);

    sendRelativeTargetsButton = new wxButton(relativeTargetSizer->GetStaticBox(),
                                             ID_SEND_RELATIVE_TARGETS_BUTTON,
                                             wxT("Send"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    targetSequenceOptionsSizer->Add(sendRelativeTargetsButton, 0, wxALL, 5);


    relativeTargetSizer->Add(targetSequenceOptionsSizer, 1, wxEXPAND, 5);


    decisionPlannerOptionsSizer->Add(relativeTargetSizer, 1, wxEXPAND, 5);


    decisionPlannerScrollWindow->SetSizer(decisionPlannerOptionsSizer);
    decisionPlannerScrollWindow->Layout();
    decisionPlannerOptionsSizer->Fit(decisionPlannerScrollWindow);
    decisionPlannerPanelSizer->Add(decisionPlannerScrollWindow, 0, wxEXPAND | wxALL, 5);


    decisionPlannerPanel->SetSizer(decisionPlannerPanelSizer);
    decisionPlannerPanel->Layout();
    decisionPlannerPanelSizer->Fit(decisionPlannerPanel);
    frameNotebook->AddPage(decisionPlannerPanel, wxT("Decision Planner"), false, wxNullBitmap);
    goalPlannerPanel = new wxPanel(frameNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    goalPlannerPanel->Enable(false);
    goalPlannerPanel->Hide();

    wxBoxSizer* goalPlannerPanelSizer;
    goalPlannerPanelSizer = new wxBoxSizer(wxHORIZONTAL);

    goalPlannerWidget =
      new GoalPlannerDisplayWidget(goalPlannerPanel, ID_GOAL_PLANNER_WIDGET, wxDefaultPosition, wxDefaultSize);
    goalPlannerPanelSizer->Add(goalPlannerWidget, 1, wxALL | wxEXPAND, 5);

    goalPlannerScrollWindow =
      new wxScrolledWindow(goalPlannerPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL);
    goalPlannerScrollWindow->SetScrollRate(0, 5);
    wxBoxSizer* goalPlannerOptionsSizer;
    goalPlannerOptionsSizer = new wxBoxSizer(wxVERTICAL);

    wxString mapRepresentationRadioBoxChoices[] = {wxT("Topological"), wxT("Graph")};
    int mapRepresentationRadioBoxNChoices = sizeof(mapRepresentationRadioBoxChoices) / sizeof(wxString);
    mapRepresentationRadioBox = new wxRadioBox(goalPlannerScrollWindow,
                                               ID_MAP_REPRESENTATION_RADIO_BOX,
                                               wxT("Map Representation"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               mapRepresentationRadioBoxNChoices,
                                               mapRepresentationRadioBoxChoices,
                                               1,
                                               wxRA_SPECIFY_COLS);
    mapRepresentationRadioBox->SetSelection(0);
    goalPlannerOptionsSizer->Add(mapRepresentationRadioBox, 0, wxALL | wxEXPAND, 5);

    wxString globalRouteDisplayRadioBoxChoices[] = {wxT("Route"), wxT("Progress")};
    int globalRouteDisplayRadioBoxNChoices = sizeof(globalRouteDisplayRadioBoxChoices) / sizeof(wxString);
    globalRouteDisplayRadioBox = new wxRadioBox(goalPlannerScrollWindow,
                                                ID_GLOBAL_ROUTE_DISPLAY_RADIO_BOX,
                                                wxT("Route Display"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                globalRouteDisplayRadioBoxNChoices,
                                                globalRouteDisplayRadioBoxChoices,
                                                1,
                                                wxRA_SPECIFY_COLS);
    globalRouteDisplayRadioBox->SetSelection(0);
    goalPlannerOptionsSizer->Add(globalRouteDisplayRadioBox, 0, wxALL | wxEXPAND, 5);

    wxStaticBoxSizer* setLocationSizer;
    setLocationSizer =
      new wxStaticBoxSizer(new wxStaticBox(goalPlannerScrollWindow, wxID_ANY, wxT("Global Location")), wxHORIZONTAL);

    setLocationButton = new wxButton(setLocationSizer->GetStaticBox(),
                                     ID_SET_LOCATION_BUTTON,
                                     wxT("Set"),
                                     wxDefaultPosition,
                                     wxDefaultSize,
                                     0);
    setLocationSizer->Add(setLocationButton, 0, wxALL, 5);

    sendLocationButton = new wxButton(setLocationSizer->GetStaticBox(),
                                      ID_SEND_LOCATION_BUTTON,
                                      wxT("Send"),
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      0);
    setLocationSizer->Add(sendLocationButton, 0, wxALL, 5);


    goalPlannerOptionsSizer->Add(setLocationSizer, 0, 0, 5);

    wxStaticBoxSizer* setGoalSizer;
    setGoalSizer = new wxStaticBoxSizer(new wxStaticBox(goalPlannerScrollWindow, wxID_ANY, wxT("Goal")), wxHORIZONTAL);

    setGoalButton =
      new wxButton(setGoalSizer->GetStaticBox(), ID_SET_GOAL_BUTTON, wxT("Set"), wxDefaultPosition, wxDefaultSize, 0);
    setGoalSizer->Add(setGoalButton, 0, wxALL, 5);

    sendGoalButton =
      new wxButton(setGoalSizer->GetStaticBox(), ID_SEND_GOAL_BUTTON, wxT("Send"), wxDefaultPosition, wxDefaultSize, 0);
    setGoalSizer->Add(sendGoalButton, 0, wxALL, 5);


    goalPlannerOptionsSizer->Add(setGoalSizer, 0, 0, 5);

    wxStaticBoxSizer* animateSearchSizer;
    animateSearchSizer =
      new wxStaticBoxSizer(new wxStaticBox(goalPlannerScrollWindow, wxID_ANY, wxT("Animate Search (FPS)")),
                           wxHORIZONTAL);

    animateFPSText = new wxTextCtrl(animateSearchSizer->GetStaticBox(),
                                    ID_ANIMATE_FPS_TEXT,
                                    wxT("15"),
                                    wxDefaultPosition,
                                    wxDefaultSize,
                                    wxTE_RIGHT);
#ifdef __WXGTK__
    if (!animateFPSText->HasFlag(wxTE_MULTILINE)) {
        animateFPSText->SetMaxLength(3);
    }
#else
    animateFPSText->SetMaxLength(3);
#endif
    animateSearchSizer->Add(animateFPSText, 0, wxALIGN_CENTER | wxALL, 5);

    animateSearchButton = new wxButton(animateSearchSizer->GetStaticBox(),
                                       ID_ANIMATE_SEARCH_BUTTON,
                                       wxT("Animate"),
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0);
    animateSearchSizer->Add(animateSearchButton, 0, wxALL, 5);


    goalPlannerOptionsSizer->Add(animateSearchSizer, 0, 0, 5);

    wxStaticBoxSizer* routeControlSizer;
    routeControlSizer =
      new wxStaticBoxSizer(new wxStaticBox(goalPlannerScrollWindow, wxID_ANY, wxT("Route Commands")), wxHORIZONTAL);

    confirmRouteButton = new wxButton(routeControlSizer->GetStaticBox(),
                                      ID_CONFIRM_ROUTE_BUTTON,
                                      wxT("Confirm"),
                                      wxDefaultPosition,
                                      wxDefaultSize,
                                      0);
    routeControlSizer->Add(confirmRouteButton, 0, wxALL, 5);

    cancelRouteButton = new wxButton(routeControlSizer->GetStaticBox(),
                                     ID_CANCEL_ROUTE_BUTTON,
                                     wxT("Cancel"),
                                     wxDefaultPosition,
                                     wxDefaultSize,
                                     0);
    routeControlSizer->Add(cancelRouteButton, 0, wxALL, 5);


    goalPlannerOptionsSizer->Add(routeControlSizer, 0, 0, 5);


    goalPlannerScrollWindow->SetSizer(goalPlannerOptionsSizer);
    goalPlannerScrollWindow->Layout();
    goalPlannerOptionsSizer->Fit(goalPlannerScrollWindow);
    goalPlannerPanelSizer->Add(goalPlannerScrollWindow, 0, wxEXPAND | wxALL, 5);


    goalPlannerPanel->SetSizer(goalPlannerPanelSizer);
    goalPlannerPanel->Layout();
    goalPlannerPanelSizer->Fit(goalPlannerPanel);
    frameNotebook->AddPage(goalPlannerPanel, wxT("Goal Planner"), false, wxNullBitmap);
    visionPanel = new wxPanel(frameNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    wxBoxSizer* visionPanelSizer;
    visionPanelSizer = new wxBoxSizer(wxHORIZONTAL);

    visionWidget = new VisionDisplayWidget(visionPanel, ID_VISION_WIDGET, wxDefaultPosition, wxDefaultSize);
    visionWidget->SetMinSize(wxSize(640, 480));

    visionPanelSizer->Add(visionWidget, 1, wxALIGN_CENTER | wxALL | wxEXPAND, 5);

    wxFlexGridSizer* visionOptionsSizer;
    visionOptionsSizer = new wxFlexGridSizer(0, 1, 0, 0);
    visionOptionsSizer->SetFlexibleDirection(wxBOTH);
    visionOptionsSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    wxStaticBoxSizer* imageSegmentationOptionsSizer;
    imageSegmentationOptionsSizer =
      new wxStaticBoxSizer(new wxStaticBox(visionPanel, wxID_ANY, wxT("Segmentation Options")), wxVERTICAL);

    showSegmentsCheckBox = new wxCheckBox(imageSegmentationOptionsSizer->GetStaticBox(),
                                          ID_SHOW_SEGMENTS_BOX,
                                          wxT("Show Segments"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    imageSegmentationOptionsSizer->Add(showSegmentsCheckBox, 0, wxALL, 5);

    wxFlexGridSizer* segmentationOptionsSizer;
    segmentationOptionsSizer = new wxFlexGridSizer(5, 2, 0, 0);
    segmentationOptionsSizer->SetFlexibleDirection(wxVERTICAL);
    segmentationOptionsSizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    minEdgeWeightLabel = new wxStaticText(imageSegmentationOptionsSizer->GetStaticBox(),
                                          wxID_ANY,
                                          wxT("Min Edge Weight:"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    minEdgeWeightLabel->Wrap(-1);
    segmentationOptionsSizer->Add(minEdgeWeightLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    minEdgeWeightSlider = new wxSlider(imageSegmentationOptionsSizer->GetStaticBox(),
                                       ID_MIN_EDGE_WEIGHT_SLIDER,
                                       42,
                                       0,
                                       50,
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       wxSL_HORIZONTAL | wxSL_LABELS);
    segmentationOptionsSizer->Add(minEdgeWeightSlider, 1, wxALL | wxEXPAND, 5);

    maxEdgeWeightLabel = new wxStaticText(imageSegmentationOptionsSizer->GetStaticBox(),
                                          wxID_ANY,
                                          wxT("Max Edge Weight:"),
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          0);
    maxEdgeWeightLabel->Wrap(-1);
    segmentationOptionsSizer->Add(maxEdgeWeightLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    maxEdgeWeightSlider = new wxSlider(imageSegmentationOptionsSizer->GetStaticBox(),
                                       ID_MAX_EDGE_WEIGHT_SLIDER,
                                       21,
                                       0,
                                       50,
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       wxSL_HORIZONTAL | wxSL_LABELS);
    segmentationOptionsSizer->Add(maxEdgeWeightSlider, 0, wxALL | wxEXPAND, 5);

    pixelSigmaLabel = new wxStaticText(imageSegmentationOptionsSizer->GetStaticBox(),
                                       wxID_ANY,
                                       wxT("Pixel Sigma:"),
                                       wxDefaultPosition,
                                       wxDefaultSize,
                                       0);
    pixelSigmaLabel->Wrap(-1);
    segmentationOptionsSizer->Add(pixelSigmaLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    pixelSigmaSlider = new wxSlider(imageSegmentationOptionsSizer->GetStaticBox(),
                                    ID_PIXEL_SIGMA_SLIDER,
                                    20,
                                    0,
                                    100,
                                    wxDefaultPosition,
                                    wxDefaultSize,
                                    wxSL_HORIZONTAL | wxSL_LABELS);
    segmentationOptionsSizer->Add(pixelSigmaSlider, 0, wxALL | wxEXPAND, 5);

    creditMultiplierLabel = new wxStaticText(imageSegmentationOptionsSizer->GetStaticBox(),
                                             wxID_ANY,
                                             wxT("Credit Multiplier:"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    creditMultiplierLabel->Wrap(-1);
    segmentationOptionsSizer->Add(creditMultiplierLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    creditMultiplierSlider = new wxSlider(imageSegmentationOptionsSizer->GetStaticBox(),
                                          ID_CREDIT_MULTIPLIER_SLIDER,
                                          10,
                                          0,
                                          100,
                                          wxDefaultPosition,
                                          wxDefaultSize,
                                          wxSL_HORIZONTAL | wxSL_LABELS);
    segmentationOptionsSizer->Add(creditMultiplierSlider, 0, wxALL | wxEXPAND, 5);

    filterWidthLabel = new wxStaticText(imageSegmentationOptionsSizer->GetStaticBox(),
                                        wxID_ANY,
                                        wxT("Filter Width:"),
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        0);
    filterWidthLabel->Wrap(-1);
    segmentationOptionsSizer->Add(filterWidthLabel, 0, wxALIGN_CENTER_VERTICAL | wxALL, 5);

    filterWidthSlider = new wxSlider(imageSegmentationOptionsSizer->GetStaticBox(),
                                     ID_FILTER_WIDTH_SLIDER,
                                     13,
                                     0,
                                     100,
                                     wxDefaultPosition,
                                     wxDefaultSize,
                                     wxSL_HORIZONTAL | wxSL_LABELS);
    segmentationOptionsSizer->Add(filterWidthSlider, 0, wxALL | wxEXPAND, 5);


    imageSegmentationOptionsSizer->Add(segmentationOptionsSizer, 1, wxEXPAND, 5);


    visionOptionsSizer->Add(imageSegmentationOptionsSizer, 1, wxEXPAND, 5);


    visionPanelSizer->Add(visionOptionsSizer, 0, wxALL, 5);


    visionPanel->SetSizer(visionPanelSizer);
    visionPanel->Layout();
    visionPanelSizer->Fit(visionPanel);
    frameNotebook->AddPage(visionPanel, wxT("Vision"), false, wxNullBitmap);
    systemPanel = new wxPanel(frameNotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    systemPanel->Enable(false);
    systemPanel->Hide();

    wxBoxSizer* systemPanelSizer;
    systemPanelSizer = new wxBoxSizer(wxVERTICAL);

    moduleStatusGrid = new wxGrid(systemPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0);

    // Grid
    moduleStatusGrid->CreateGrid(5, 5);
    moduleStatusGrid->EnableEditing(false);
    moduleStatusGrid->EnableGridLines(true);
    moduleStatusGrid->EnableDragGridSize(false);
    moduleStatusGrid->SetMargins(0, 0);

    // Columns
    moduleStatusGrid->EnableDragColMove(false);
    moduleStatusGrid->EnableDragColSize(true);
    moduleStatusGrid->SetColLabelSize(30);
    moduleStatusGrid->SetColLabelAlignment(wxALIGN_CENTER, wxALIGN_CENTER);

    // Rows
    moduleStatusGrid->EnableDragRowSize(true);
    moduleStatusGrid->SetRowLabelSize(80);
    moduleStatusGrid->SetRowLabelAlignment(wxALIGN_CENTER, wxALIGN_CENTER);

    // Label Appearance

    // Cell Defaults
    moduleStatusGrid->SetDefaultCellAlignment(wxALIGN_LEFT, wxALIGN_TOP);
    systemPanelSizer->Add(moduleStatusGrid, 0, wxALL, 5);


    systemPanel->SetSizer(systemPanelSizer);
    systemPanel->Layout();
    systemPanelSizer->Fit(systemPanel);
    frameNotebook->AddPage(systemPanel, wxT("System"), false, wxNullBitmap);

    debugFrameSizer->Add(frameNotebook, 1, wxEXPAND | wxALL, 5);


    this->SetSizer(debugFrameSizer);
    this->Layout();
    gridCellStatusBar = this->CreateStatusBar(1, wxSTB_SIZEGRIP, ID_GRID_CELL_STATUS_BAR);
    debugUIMenu = new wxMenuBar(0);
    windowMenu = new wxMenu();
    wxMenuItem* localMetricWindowItem;
    localMetricWindowItem = new wxMenuItem(windowMenu,
                                           ID_LOCAL_METRIC_WINDOW_ITEM,
                                           wxString(wxT("Local Metric")),
                                           wxT("Display Local Metric HSSH tab"),
                                           wxITEM_CHECK);
    windowMenu->Append(localMetricWindowItem);
    localMetricWindowItem->Check(true);

    wxMenuItem* localTopologyWindowItem;
    localTopologyWindowItem = new wxMenuItem(windowMenu,
                                             ID_LOCAL_TOPOLOGY_WINDOW_ITEM,
                                             wxString(wxT("Local Topology")),
                                             wxT("Show Local Topology HSSH tab"),
                                             wxITEM_CHECK);
    windowMenu->Append(localTopologyWindowItem);
    localTopologyWindowItem->Check(true);

    wxMenuItem* globalTopologyWindowItem;
    globalTopologyWindowItem = new wxMenuItem(windowMenu,
                                              ID_GLOBAL_TOPOLOGY_WINDOW_ITEM,
                                              wxString(wxT("Global Topology")),
                                              wxT("Show Global Topology HSSH tab"),
                                              wxITEM_CHECK);
    windowMenu->Append(globalTopologyWindowItem);
    globalTopologyWindowItem->Check(true);

    wxMenuItem* relocalizationWindowItem;
    relocalizationWindowItem = new wxMenuItem(windowMenu,
                                              ID_RELOCALIZATION_WINDOW_ITEM,
                                              wxString(wxT("Relocalization")),
                                              wxT("Show Relocalization tab"),
                                              wxITEM_CHECK);
    windowMenu->Append(relocalizationWindowItem);
    relocalizationWindowItem->Check(true);

    wxMenuItem* metricPlannerWindowItem;
    metricPlannerWindowItem = new wxMenuItem(windowMenu,
                                             ID_METRIC_PLANNER_WINDOW_ITEM,
                                             wxString(wxT("Metric Planner")),
                                             wxT("Show Metric Planner tab"),
                                             wxITEM_CHECK);
    windowMenu->Append(metricPlannerWindowItem);
    metricPlannerWindowItem->Check(true);

    wxMenuItem* decisionPlannerWindowItem;
    decisionPlannerWindowItem = new wxMenuItem(windowMenu,
                                               ID_DECISION_PLANNER_WINDOW_ITEM,
                                               wxString(wxT("Decision Planner")),
                                               wxT("Show Decision Planner tab"),
                                               wxITEM_CHECK);
    windowMenu->Append(decisionPlannerWindowItem);
    decisionPlannerWindowItem->Check(true);

    wxMenuItem* goalPlannerWindowItem;
    goalPlannerWindowItem = new wxMenuItem(windowMenu,
                                           ID_GOAL_PLANNER_WINDOW_ITEM,
                                           wxString(wxT("Goal Planner")),
                                           wxT("Show Goal Planner tab"),
                                           wxITEM_CHECK);
    windowMenu->Append(goalPlannerWindowItem);
    goalPlannerWindowItem->Check(true);

    wxMenuItem* visionWindowItem;
    visionWindowItem =
      new wxMenuItem(windowMenu, ID_VISION_WINDOW_ITEM, wxString(wxT("Vision")), wxT("Show Vision tab"), wxITEM_CHECK);
    windowMenu->Append(visionWindowItem);

    wxMenuItem* systemWindowItem;
    systemWindowItem =
      new wxMenuItem(windowMenu, ID_SYSTEM_WINDOW_ITEM, wxString(wxT("System")), wxT("Show System tab"), wxITEM_CHECK);
    windowMenu->Append(systemWindowItem);

    debugUIMenu->Append(windowMenu, wxT("Window"));

    this->SetMenuBar(debugUIMenu);


    this->Centre(wxBOTH);

    // Connect Events
    numTrajectoriesText->Connect(wxEVT_COMMAND_TEXT_UPDATED,
                                 wxCommandEventHandler(DebugFrame::numTrajectoriesTextOnText),
                                 NULL,
                                 this);
}

DebugFrame::~DebugFrame()
{
    // Disconnect Events
    numTrajectoriesText->Disconnect(wxEVT_COMMAND_TEXT_UPDATED,
                                    wxCommandEventHandler(DebugFrame::numTrajectoriesTextOnText),
                                    NULL,
                                    this);
}

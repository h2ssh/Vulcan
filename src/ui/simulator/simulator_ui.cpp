///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version 3.9.0 Dec 31 2020)
// http://www.wxformbuilder.org/
//
// PLEASE DO *NOT* EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "ui/simulator/simulator_display.h"
#include "ui/simulator/simulator_robot_display.h"

#include "simulator_ui.h"

///////////////////////////////////////////////////////////////////////////
using namespace vulcan::ui;

SimulatorUI::SimulatorUI(wxWindow* parent,
                         wxWindowID id,
                         const wxString& title,
                         const wxPoint& pos,
                         const wxSize& size,
                         long style)
: UIMainFrame(parent, id, title, pos, size, style)
{
    this->SetSizeHints(wxDefaultSize, wxDefaultSize);

    wxBoxSizer* simulatorUiSizer;
    simulatorUiSizer = new wxBoxSizer(wxHORIZONTAL);

    display = new SimulatorDisplay(this, ID_SIMULATOR_DISPLAY, wxDefaultPosition, wxDefaultSize);
    simulatorUiSizer->Add(display, 10, wxALL | wxEXPAND, 5);

    robot_display = new SimulatorRobotDisplay(this, ID_SIMULATOR_ROBOT_DISPLAY, wxDefaultPosition, wxDefaultSize);
    simulatorUiSizer->Add(robot_display, 10, wxALL | wxEXPAND, 5);

    controlsScroller = new wxScrolledWindow(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxHSCROLL | wxVSCROLL);
    controlsScroller->SetScrollRate(5, 5);
    wxBoxSizer* ControlPanel;
    ControlPanel = new wxBoxSizer(wxVERTICAL);

    wxStaticBoxSizer* DestinationSelectionPanel;
    DestinationSelectionPanel =
      new wxStaticBoxSizer(new wxStaticBox(controlsScroller, wxID_ANY, wxT("Main Robot Destination Selection")),
                           wxHORIZONTAL);

    DestinationPoseSelection = new wxButton(DestinationSelectionPanel->GetStaticBox(),
                                            ID_SIMULATOR_SELECT_DESTINATION_POSE_BUTTON,
                                            wxT("Select"),
                                            wxDefaultPosition,
                                            wxDefaultSize,
                                            0);
    DestinationSelectionPanel->Add(DestinationPoseSelection, 0, wxALL, 5);

    DestinationPoseSender = new wxButton(DestinationSelectionPanel->GetStaticBox(),
                                         ID_SIMULATOR_SEND_DESITNATION_POSE_BUTTON,
                                         wxT("Send"),
                                         wxDefaultPosition,
                                         wxDefaultSize,
                                         0);
    DestinationSelectionPanel->Add(DestinationPoseSender, 0, wxALL, 5);

    DestinationPoseCanceler = new wxButton(DestinationSelectionPanel->GetStaticBox(),
                                           ID_SIMULATOR_CANCEL_DESTINATION_POSE_BUTTON,
                                           wxT("Cancel"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    DestinationSelectionPanel->Add(DestinationPoseCanceler, 0, wxALL, 5);


    ControlPanel->Add(DestinationSelectionPanel, 0, wxEXPAND, 5);

    wxStaticBoxSizer* LoadScriptControl;
    LoadScriptControl =
      new wxStaticBoxSizer(new wxStaticBox(controlsScroller, ID_LOAD_SCRIPT_CONTROL, wxT("Load Script")), wxVERTICAL);

    wxGridSizer* LoadScriptSizer;
    LoadScriptSizer = new wxGridSizer(3, 2, 0, 0);

    simulatorScriptFileText = new wxTextCtrl(LoadScriptControl->GetStaticBox(),
                                             wxID_ANY,
                                             wxT("Enter script name"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    LoadScriptSizer->Add(simulatorScriptFileText, 0, wxALL | wxEXPAND, 5);

    simulatorLoadScriptButton = new wxButton(LoadScriptControl->GetStaticBox(),
                                             ID_SIMULATOR_LOAD_SCRIPT_BUTTON,
                                             wxT("Load Script"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    LoadScriptSizer->Add(simulatorLoadScriptButton, 0, wxALL | wxEXPAND, 5);

    simulatorSendScriptButton = new wxButton(LoadScriptControl->GetStaticBox(),
                                             ID_SIMULATOR_SEND_SCRIPT_BUTTON,
                                             wxT("Send"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    LoadScriptSizer->Add(simulatorSendScriptButton, 0, wxALL | wxEXPAND, 5);

    simulatorSkipWayPointButton = new wxButton(LoadScriptControl->GetStaticBox(),
                                               ID_SIMULATOR_SKIP_WAYPOINT_BUTTON,
                                               wxT("Skip"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    LoadScriptSizer->Add(simulatorSkipWayPointButton, 0, wxALL | wxEXPAND, 5);

    simulatorCancelFollowingButton = new wxButton(LoadScriptControl->GetStaticBox(),
                                                  ID_SIMULATOR_CANCEL_FOLLOWING_BUTTON,
                                                  wxT("Stop"),
                                                  wxDefaultPosition,
                                                  wxDefaultSize,
                                                  0);
    LoadScriptSizer->Add(simulatorCancelFollowingButton, 0, wxALL | wxEXPAND, 5);

    simulatorLoopWaypointsButton = new wxButton(LoadScriptControl->GetStaticBox(),
                                                ID_SIMULATOR_LOOP_WAYPOINTS_BUTTON,
                                                wxT("Loop"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    LoadScriptSizer->Add(simulatorLoopWaypointsButton, 0, wxALL | wxEXPAND, 5);


    LoadScriptControl->Add(LoadScriptSizer, 0, wxEXPAND, 5);


    ControlPanel->Add(LoadScriptControl, 0, wxEXPAND, 5);

    wxStaticBoxSizer* AddRobotPanel;
    AddRobotPanel = new wxStaticBoxSizer(new wxStaticBox(controlsScroller, ID_ADD_ROBOT, wxT("Add Robot")), wxVERTICAL);

    wxBoxSizer* Add_Robot_Pose_Select_sizer;
    Add_Robot_Pose_Select_sizer = new wxBoxSizer(wxHORIZONTAL);

    simulatorAddRobotSelectPoseButton11 = new wxButton(AddRobotPanel->GetStaticBox(),
                                                       ID_SIMULATOR_ADD_ROBOT_SELECT_POSE_BUTTON,
                                                       wxT("Select Pose"),
                                                       wxDefaultPosition,
                                                       wxDefaultSize,
                                                       0);
    Add_Robot_Pose_Select_sizer->Add(simulatorAddRobotSelectPoseButton11, 1, wxALL, 5);

    simulatorAddRobotSetPoseButton = new wxButton(AddRobotPanel->GetStaticBox(),
                                                  ID_SIMULATOR_ADD_ROBOT_SET_POSE_BUTTON,
                                                  wxT("Set Pose"),
                                                  wxDefaultPosition,
                                                  wxDefaultSize,
                                                  0);
    Add_Robot_Pose_Select_sizer->Add(simulatorAddRobotSetPoseButton, 1, wxALL, 5);


    AddRobotPanel->Add(Add_Robot_Pose_Select_sizer, 1, wxEXPAND, 5);

    wxBoxSizer* AddRobotConfigSizer;
    AddRobotConfigSizer = new wxBoxSizer(wxHORIZONTAL);

    simulatorRobotConfigText = new wxTextCtrl(AddRobotPanel->GetStaticBox(),
                                              ID_SIMULATOR_LOAD_CONFIG_FILE_TEXT,
                                              wxT("Load Config File"),
                                              wxDefaultPosition,
                                              wxDefaultSize,
                                              0);
    AddRobotConfigSizer->Add(simulatorRobotConfigText, 1, wxALL, 5);

    simulatorLoadConfigButton = new wxButton(AddRobotPanel->GetStaticBox(),
                                             ID_SIMULATOR_LOAD_CONFIG_BUTTON,
                                             wxT("Load Config"),
                                             wxDefaultPosition,
                                             wxDefaultSize,
                                             0);
    AddRobotConfigSizer->Add(simulatorLoadConfigButton, 1, wxALL, 5);


    AddRobotPanel->Add(AddRobotConfigSizer, 0, wxEXPAND, 5);

    simulatorAddRobotButton = new wxButton(AddRobotPanel->GetStaticBox(),
                                           ID_SIMULATOR_ADD_ROBOT_BUTTON,
                                           wxT("Add Robot"),
                                           wxDefaultPosition,
                                           wxDefaultSize,
                                           0);
    AddRobotPanel->Add(simulatorAddRobotButton, 0, wxALL | wxEXPAND, 5);


    ControlPanel->Add(AddRobotPanel, 0, wxEXPAND, 5);

    wxStaticBoxSizer* RobotControlSizer;
    RobotControlSizer =
      new wxStaticBoxSizer(new wxStaticBox(controlsScroller, ID_SIMULATOR_ROBOT_CONTROL_SIZER, wxT("Robot Control")),
                           wxVERTICAL);

    simulatorPauseAllRobotButton = new wxButton(RobotControlSizer->GetStaticBox(),
                                                ID_SIMULATOR_PAUSE_ALL_ROBOT_BUTTON,
                                                wxT("Pause All Robot"),
                                                wxDefaultPosition,
                                                wxDefaultSize,
                                                0);
    RobotControlSizer->Add(simulatorPauseAllRobotButton, 0, wxALL | wxEXPAND, 5);

    simulatorSelectRobotPauseButton = new wxButton(RobotControlSizer->GetStaticBox(),
                                                   ID_SIMULATOR_SELECT_ROBOT_PAUSE_BUTTON,
                                                   wxT("Select Robot to Pause"),
                                                   wxDefaultPosition,
                                                   wxDefaultSize,
                                                   0);
    RobotControlSizer->Add(simulatorSelectRobotPauseButton, 0, wxALL | wxEXPAND, 5);


    ControlPanel->Add(RobotControlSizer, 1, wxEXPAND, 5);

    wxBoxSizer* StartingSimulatorSizer;
    StartingSimulatorSizer = new wxBoxSizer(wxVERTICAL);

    StartNecessaryModularButton = new wxButton(controlsScroller,
                                               ID_START_NEC_MODULAR_BUTTON,
                                               wxT("Start Necessary Modular"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    StartingSimulatorSizer->Add(StartNecessaryModularButton, 0, wxALL | wxEXPAND, 5);

    StartSimulatorCaseOneButton = new wxButton(controlsScroller,
                                               ID_SIMULATOR_CASE_ONE_START_BUTTON,
                                               wxT("Load Simulator Case one"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    StartingSimulatorSizer->Add(StartSimulatorCaseOneButton, 0, wxALL | wxALIGN_CENTER_HORIZONTAL | wxEXPAND, 5);

    StartSimulatorCaseTwoButton = new wxButton(controlsScroller,
                                               ID_SIMULATOR_CASE_TWO_START_BUTTON,
                                               wxT("Load Simulator Case two"),
                                               wxDefaultPosition,
                                               wxDefaultSize,
                                               0);
    StartingSimulatorSizer->Add(StartSimulatorCaseTwoButton, 0, wxALL | wxEXPAND, 5);

    StartSimulatorCaseThreeButton = new wxButton(controlsScroller,
                                                 ID_SIMULATOR_CASE_THREE_START_BUTTON,
                                                 wxT("Load Simulator Case three"),
                                                 wxDefaultPosition,
                                                 wxDefaultSize,
                                                 0);
    StartingSimulatorSizer->Add(StartSimulatorCaseThreeButton, 0, wxALL | wxEXPAND, 5);

    StartSimulatorButton = new wxButton(controlsScroller,
                                        ID_SIMULATOR_START_SIMULATOR_BUTTON,
                                        wxT("Start"),
                                        wxDefaultPosition,
                                        wxDefaultSize,
                                        0);
    StartingSimulatorSizer->Add(StartSimulatorButton, 1, wxALL | wxEXPAND, 5);


    ControlPanel->Add(StartingSimulatorSizer, 0, wxEXPAND, 5);


    controlsScroller->SetSizer(ControlPanel);
    controlsScroller->Layout();
    ControlPanel->Fit(controlsScroller);
    simulatorUiSizer->Add(controlsScroller, 7, wxEXPAND | wxALL, 5);


    this->SetSizer(simulatorUiSizer);
    this->Layout();
    gridCellStatusBar = this->CreateStatusBar(1, wxSTB_SIZEGRIP, ID_GRID_CELL_STATUS_BAR);

    this->Centre(wxBOTH);
}

SimulatorUI::~SimulatorUI()
{
}

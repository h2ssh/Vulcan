///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version 3.9.0 Dec 31 2020)
// http://www.wxformbuilder.org/
//
// PLEASE DO *NOT* EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#pragma once

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
namespace vulcan{ namespace ui{ class SimulatorDisplay; } }
namespace vulcan{ namespace ui{ class SimulatorRobotDisplay; } }

#include "ui/common/ui_main_frame.h"
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/button.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/textctrl.h>
#include <wx/scrolwin.h>
#include <wx/statusbr.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////

namespace vulcan
{
	namespace ui
	{
		#define ID_SIMULATOR_DISPLAY 1000
		#define ID_SIMULATOR_ROBOT_DISPLAY 1001
		#define ID_SIMULATOR_SELECT_DESTINATION_POSE_BUTTON 1002
		#define ID_SIMULATOR_SEND_DESITNATION_POSE_BUTTON 1003
		#define ID_SIMULATOR_CANCEL_DESTINATION_POSE_BUTTON 1004
		#define ID_LOAD_SCRIPT_CONTROL 1005
		#define ID_SIMULATOR_LOAD_SCRIPT_BUTTON 1006
		#define ID_SIMULATOR_SEND_SCRIPT_BUTTON 1007
		#define ID_SIMULATOR_SKIP_WAYPOINT_BUTTON 1008
		#define ID_SIMULATOR_CANCEL_FOLLOWING_BUTTON 1009
		#define ID_SIMULATOR_LOOP_WAYPOINTS_BUTTON 1010
		#define ID_ADD_ROBOT 1011
		#define ID_SIMULATOR_ADD_ROBOT_SELECT_POSE_BUTTON 1012
		#define ID_SIMULATOR_ADD_ROBOT_SET_POSE_BUTTON 1013
		#define ID_SIMULATOR_LOAD_CONFIG_FILE_TEXT 1014
		#define ID_SIMULATOR_LOAD_CONFIG_BUTTON 1015
		#define ID_SIMULATOR_ADD_ROBOT_BUTTON 1016
		#define ID_SIMULATOR_ROBOT_CONTROL_SIZER 1017
		#define ID_SIMULATOR_PAUSE_ALL_ROBOT_BUTTON 1018
		#define ID_SIMULATOR_SELECT_ROBOT_PAUSE_BUTTON 1019
		#define ID_START_NEC_MODULAR_BUTTON 1020
		#define ID_SIMULATOR_CASE_ONE_START_BUTTON 1021
		#define ID_SIMULATOR_CASE_TWO_START_BUTTON 1022
		#define ID_SIMULATOR_CASE_THREE_START_BUTTON 1023
		#define ID_SIMULATOR_START_SIMULATOR_BUTTON 1024
		#define ID_GRID_CELL_STATUS_BAR 1025

		///////////////////////////////////////////////////////////////////////////////
		/// Class SimulatorUI
		///////////////////////////////////////////////////////////////////////////////
		class SimulatorUI : public UIMainFrame
		{
			private:

			protected:
				SimulatorDisplay* display;
				SimulatorRobotDisplay* robot_display;
				wxScrolledWindow* controlsScroller;
				wxButton* DestinationPoseSelection;
				wxButton* DestinationPoseSender;
				wxButton* DestinationPoseCanceler;
				wxTextCtrl* simulatorScriptFileText;
				wxButton* simulatorLoadScriptButton;
				wxButton* simulatorSendScriptButton;
				wxButton* simulatorSkipWayPointButton;
				wxButton* simulatorCancelFollowingButton;
				wxButton* simulatorLoopWaypointsButton;
				wxButton* simulatorAddRobotSelectPoseButton11;
				wxButton* simulatorAddRobotSetPoseButton;
				wxTextCtrl* simulatorRobotConfigText;
				wxButton* simulatorLoadConfigButton;
				wxButton* simulatorAddRobotButton;
				wxButton* simulatorPauseAllRobotButton;
				wxButton* simulatorSelectRobotPauseButton;
				wxButton* StartNecessaryModularButton;
				wxButton* StartSimulatorCaseOneButton;
				wxButton* StartSimulatorCaseTwoButton;
				wxButton* StartSimulatorCaseThreeButton;
				wxButton* StartSimulatorButton;
				wxStatusBar* gridCellStatusBar;

			public:

				SimulatorUI( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Simulator UI"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 1020,800 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );

				~SimulatorUI();

		};

	} // namespace ui
} // namespace vulcan


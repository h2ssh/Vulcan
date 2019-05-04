/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Dec 11 2014)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __NAVIGATION_INTERFACE_H__
#define __NAVIGATION_INTERFACE_H__

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
class UIMainFrame;
namespace vulcan{ namespace ui{ class NavigationInterfaceDisplay; } }

#include "ui/common/ui_main_frame.h"
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/listbox.h>
#include <wx/tglbtn.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/frame.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/dialog.h>

///////////////////////////////////////////////////////////////////////////

namespace vulcan
{
	namespace ui
	{
		#define ID_NAVIGATION_INTERFACE_DISPLAY 1000
		#define ID_NAMED_GOALS_LIST 1001
		#define ID_SELECT_GOAL_BUTTON 1002
		#define ID_ADD_SELECTED_BUTTON 1003
		#define ID_ADD_CURRENT_LOCATION_BUTTON 1004
		#define ID_PREVIEW_ROUTE_BUTTON 1005
		#define ID_GO_BUTTON 1006
		#define ID_NAME_OK_BUTTON 1007
		#define ID_NAME_CANCEL_BUTTON 1008
		
		///////////////////////////////////////////////////////////////////////////////
		/// Class NavigationInterface
		///////////////////////////////////////////////////////////////////////////////
		class NavigationInterface : public UIMainFrame
		{
			private:
			
			protected:
				NavigationInterfaceDisplay* display;
				wxListBox* namedGoalsList;
				wxToggleButton* selectGoalButton;
				wxButton* addSelectedButton;
				wxButton* addCurrentLocationButton;
				wxButton* previewRouteButton;
				wxButton* goButton;
			
			public:
				
				NavigationInterface( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Navigation Interface"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 1000,800 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
				
				~NavigationInterface();
			
		};
		
		///////////////////////////////////////////////////////////////////////////////
		/// Class GoalNameDialogBase
		///////////////////////////////////////////////////////////////////////////////
		class GoalNameDialogBase : public wxDialog 
		{
			private:
			
			protected:
				wxStaticText* goalNameLabel;
				wxTextCtrl* goalNameText;
				wxButton* nameOkButton;
				wxButton* nameCancelButton;
				
				// Virtual event handlers, overide them in your derived class
				virtual void goalNameTextEntered( wxCommandEvent& event ) { event.Skip(); }
				virtual void goalNameEnterPressed( wxCommandEvent& event ) { event.Skip(); }
				virtual void nameOkayPressed( wxCommandEvent& event ) { event.Skip(); }
				virtual void nameCancelPressed( wxCommandEvent& event ) { event.Skip(); }
				
			
			public:
				
				GoalNameDialogBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Name the New Goal"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 444,114 ), long style = wxDEFAULT_DIALOG_STYLE ); 
				~GoalNameDialogBase();
			
		};
		
	} // namespace ui
} // namespace vulcan

#endif //__NAVIGATION_INTERFACE_H__

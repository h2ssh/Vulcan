/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Dec  1 2011)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __LOGPLAYER_UI_H__
#define __LOGPLAYER_UI_H__

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
#include <wx/string.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/menu.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/stattext.h>
#include <wx/slider.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/textctrl.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////

#define ID_FILE_OPEN_MENU_ITEM 1000
#define ID_FILE_QUIT_MENU_ITEM 1001
#define ID_LOGFILE_TEXT 1002
#define ID_LOG_POSITION_SLIDER 1003
#define ID_RESTART_BUTTON 1004
#define ID_PLAY_BUTTON 1005
#define ID_STEP_BUTTON 1006
#define ID_SLOWER_BUTTON 1007
#define ID_PLAYBACK_SPEED_TEXT 1008
#define ID_FASTER_BUTTON 1009

///////////////////////////////////////////////////////////////////////////////
/// Class LogplayerUI
///////////////////////////////////////////////////////////////////////////////
class LogplayerUI : public wxFrame 
{
	private:
	
	protected:
		wxMenuBar* logplayerMenuBar;
		wxMenu* fileMenu;
		wxStaticText* logFileText;
		wxSlider* logPositionSlider;
		wxButton* restartButton;
		wxButton* playButton;
		wxButton* stepButton;
		wxButton* slowerButton;
		wxTextCtrl* playbackSpeedText;
		wxButton* fasterButton;
	
	public:
		
		LogplayerUI( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Logplayer"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 460,223 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		
		~LogplayerUI();
	
};

#endif //__LOGPLAYER_UI_H__

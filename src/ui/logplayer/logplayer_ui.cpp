///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version 3.9.0 Dec 31 2020)
// http://www.wxformbuilder.org/
//
// PLEASE DO *NOT* EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "logplayer_ui.h"

///////////////////////////////////////////////////////////////////////////

LogplayerUI::LogplayerUI( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );

	logplayerMenuBar = new wxMenuBar( 0 );
	fileMenu = new wxMenu();
	wxMenuItem* fileOpenMenuItem;
	fileOpenMenuItem = new wxMenuItem( fileMenu, ID_FILE_OPEN_MENU_ITEM, wxString( wxT("Open") ) , wxEmptyString, wxITEM_NORMAL );
	fileMenu->Append( fileOpenMenuItem );

	wxMenuItem* fileQuitMenuItem;
	fileQuitMenuItem = new wxMenuItem( fileMenu, ID_FILE_QUIT_MENU_ITEM, wxString( wxT("Quit") ) , wxEmptyString, wxITEM_NORMAL );
	fileMenu->Append( fileQuitMenuItem );

	logplayerMenuBar->Append( fileMenu, wxT("File") );

	this->SetMenuBar( logplayerMenuBar );

	wxBoxSizer* logplayerFrameSizer;
	logplayerFrameSizer = new wxBoxSizer( wxVERTICAL );

	logFileText = new wxStaticText( this, ID_LOGFILE_TEXT, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxST_NO_AUTORESIZE|wxALIGN_CENTER_HORIZONTAL );
	logFileText->Wrap( -1 );
	logplayerFrameSizer->Add( logFileText, 0, wxALIGN_CENTER_HORIZONTAL|wxALL|wxEXPAND, 5 );

	logPositionSlider = new wxSlider( this, ID_LOG_POSITION_SLIDER, 50, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_AUTOTICKS|wxSL_BOTTOM|wxSL_HORIZONTAL|wxSL_LABELS );
	logplayerFrameSizer->Add( logPositionSlider, 0, wxALL|wxEXPAND, 5 );

	wxBoxSizer* controlButtonsSizer;
	controlButtonsSizer = new wxBoxSizer( wxHORIZONTAL );


	controlButtonsSizer->Add( 0, 0, 1, wxEXPAND, 5 );

	restartButton = new wxButton( this, ID_RESTART_BUTTON, wxT("Restart"), wxDefaultPosition, wxDefaultSize, 0 );
	controlButtonsSizer->Add( restartButton, 0, wxALL, 5 );

	playButton = new wxButton( this, ID_PLAY_BUTTON, wxT("Play"), wxDefaultPosition, wxDefaultSize, 0 );
	controlButtonsSizer->Add( playButton, 0, wxALIGN_CENTER_HORIZONTAL|wxALL, 5 );

	stepButton = new wxButton( this, ID_STEP_BUTTON, wxT("Step"), wxDefaultPosition, wxDefaultSize, 0 );
	controlButtonsSizer->Add( stepButton, 0, wxALIGN_CENTER_HORIZONTAL|wxALL, 5 );


	controlButtonsSizer->Add( 0, 0, 1, wxEXPAND, 5 );


	logplayerFrameSizer->Add( controlButtonsSizer, 0, wxALIGN_CENTER, 5 );

	wxBoxSizer* speedButtonSizer;
	speedButtonSizer = new wxBoxSizer( wxHORIZONTAL );

	slowerButton = new wxButton( this, ID_SLOWER_BUTTON, wxT("Slower"), wxDefaultPosition, wxDefaultSize, 0 );
	speedButtonSizer->Add( slowerButton, 0, wxALL, 5 );

	playbackSpeedText = new wxTextCtrl( this, ID_PLAYBACK_SPEED_TEXT, wxT("1.0"), wxDefaultPosition, wxDefaultSize, wxTE_NO_VSCROLL|wxTE_PROCESS_ENTER|wxTE_CENTER );
	speedButtonSizer->Add( playbackSpeedText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

	fasterButton = new wxButton( this, ID_FASTER_BUTTON, wxT("Faster"), wxDefaultPosition, wxDefaultSize, 0 );
	speedButtonSizer->Add( fasterButton, 0, wxALL, 5 );


	logplayerFrameSizer->Add( speedButtonSizer, 0, wxALIGN_CENTER_HORIZONTAL, 5 );


	this->SetSizer( logplayerFrameSizer );
	this->Layout();

	this->Centre( wxBOTH );
}

LogplayerUI::~LogplayerUI()
{
}

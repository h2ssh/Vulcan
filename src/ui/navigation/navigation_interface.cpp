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

#include "ui/navigation/navigation_interface_display.h"

#include "navigation_interface.h"

///////////////////////////////////////////////////////////////////////////
using namespace vulcan::ui;

NavigationInterface::NavigationInterface( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : UIMainFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* navInterfaceSizer;
	navInterfaceSizer = new wxBoxSizer( wxHORIZONTAL );
	
	display = new NavigationInterfaceDisplay(this, ID_NAVIGATION_INTERFACE_DISPLAY, wxDefaultPosition,wxDefaultSize);
	navInterfaceSizer->Add( display, 1, wxALL|wxEXPAND, 5 );
	
	wxBoxSizer* goalControlSizer;
	goalControlSizer = new wxBoxSizer( wxVERTICAL );
	
	namedGoalsList = new wxListBox( this, ID_NAMED_GOALS_LIST, wxDefaultPosition, wxDefaultSize, 0, NULL, wxLB_SINGLE ); 
	namedGoalsList->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	namedGoalsList->SetToolTip( wxT("List of available goals for robot navigation") );
	namedGoalsList->SetMinSize( wxSize( 250,-1 ) );
	
	goalControlSizer->Add( namedGoalsList, 1, wxALL|wxEXPAND, 5 );
	
	selectGoalButton = new wxToggleButton( this, ID_SELECT_GOAL_BUTTON, wxT("Select Goal in Map"), wxDefaultPosition, wxDefaultSize, 0 );
	selectGoalButton->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	goalControlSizer->Add( selectGoalButton, 0, wxALL|wxEXPAND, 5 );
	
	addSelectedButton = new wxButton( this, ID_ADD_SELECTED_BUTTON, wxT("Add Selected Goal"), wxDefaultPosition, wxDefaultSize, 0 );
	addSelectedButton->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	goalControlSizer->Add( addSelectedButton, 0, wxALL|wxEXPAND, 5 );
	
	addCurrentLocationButton = new wxButton( this, ID_ADD_CURRENT_LOCATION_BUTTON, wxT("Add Current Location"), wxDefaultPosition, wxDefaultSize, 0 );
	addCurrentLocationButton->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	goalControlSizer->Add( addCurrentLocationButton, 0, wxALL|wxEXPAND, 5 );
	
	previewRouteButton = new wxButton( this, ID_PREVIEW_ROUTE_BUTTON, wxT("Preview"), wxDefaultPosition, wxDefaultSize, 0 );
	previewRouteButton->SetFont( wxFont( 14, 70, 90, 90, false, wxEmptyString ) );
	
	goalControlSizer->Add( previewRouteButton, 0, wxALL|wxEXPAND, 5 );
	
	goButton = new wxButton( this, ID_GO_BUTTON, wxT("GO"), wxDefaultPosition, wxDefaultSize, 0 );
	goButton->SetFont( wxFont( 20, 70, 90, 92, false, wxEmptyString ) );
	
	goalControlSizer->Add( goButton, 0, wxALL|wxEXPAND, 5 );
	
	
	navInterfaceSizer->Add( goalControlSizer, 0, wxEXPAND, 5 );
	
	
	this->SetSizer( navInterfaceSizer );
	this->Layout();
	
	this->Centre( wxBOTH );
}

NavigationInterface::~NavigationInterface()
{
}

GoalNameDialogBase::GoalNameDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* goalNameSizer;
	goalNameSizer = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* nameTextSizer;
	nameTextSizer = new wxBoxSizer( wxHORIZONTAL );
	
	goalNameLabel = new wxStaticText( this, wxID_ANY, wxT("Goal Name:"), wxDefaultPosition, wxDefaultSize, 0 );
	goalNameLabel->Wrap( -1 );
	nameTextSizer->Add( goalNameLabel, 0, wxALIGN_CENTER|wxALL, 5 );
	
	goalNameText = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	nameTextSizer->Add( goalNameText, 1, wxALIGN_CENTER|wxALL, 5 );
	
	
	goalNameSizer->Add( nameTextSizer, 1, wxEXPAND, 5 );
	
	wxBoxSizer* okCancelSizer;
	okCancelSizer = new wxBoxSizer( wxHORIZONTAL );
	
	
	okCancelSizer->Add( 0, 0, 1, wxEXPAND, 5 );
	
	nameOkButton = new wxButton( this, ID_NAME_OK_BUTTON, wxT("Okay"), wxDefaultPosition, wxDefaultSize, 0 );
	okCancelSizer->Add( nameOkButton, 0, wxALIGN_CENTER|wxALL, 5 );
	
	nameCancelButton = new wxButton( this, ID_NAME_CANCEL_BUTTON, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	okCancelSizer->Add( nameCancelButton, 0, wxALIGN_CENTER|wxALL, 5 );
	
	
	okCancelSizer->Add( 0, 0, 1, wxEXPAND, 5 );
	
	
	goalNameSizer->Add( okCancelSizer, 1, wxEXPAND, 5 );
	
	
	this->SetSizer( goalNameSizer );
	this->Layout();
	
	this->Centre( wxBOTH );
	
	// Connect Events
	goalNameText->Connect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( GoalNameDialogBase::goalNameTextEntered ), NULL, this );
	goalNameText->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( GoalNameDialogBase::goalNameEnterPressed ), NULL, this );
	nameOkButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GoalNameDialogBase::nameOkayPressed ), NULL, this );
	nameCancelButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GoalNameDialogBase::nameCancelPressed ), NULL, this );
}

GoalNameDialogBase::~GoalNameDialogBase()
{
	// Disconnect Events
	goalNameText->Disconnect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( GoalNameDialogBase::goalNameTextEntered ), NULL, this );
	goalNameText->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( GoalNameDialogBase::goalNameEnterPressed ), NULL, this );
	nameOkButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GoalNameDialogBase::nameOkayPressed ), NULL, this );
	nameCancelButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GoalNameDialogBase::nameCancelPressed ), NULL, this );
	
}

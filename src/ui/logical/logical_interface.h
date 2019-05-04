/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Nov 19 2012)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __LOGICAL_INTERFACE_H__
#define __LOGICAL_INTERFACE_H__

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
class UIMainFrame;

#include "ui/common/ui_main_frame.h"
#include <ui/debug/goal_planner_display_widget.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/frame.h>
#include <wx/tglbtn.h>
#include <wx/statline.h>
#include <wx/dialog.h>
#include <wx/listbox.h>
#include <wx/stattext.h>

///////////////////////////////////////////////////////////////////////////

namespace vulcan
{
	namespace ui
	{
		#define ID_GOAL_PLANNER_WIDGET 1000
		#define ID_DECISION_BUTTON 1001
		#define ID_GOAL_BUTTON 1002
		#define ID_STOP_BUTTON 1003
		#define ID_DECISION_DIALOG 1004
		#define ID_FORWARD_DECISION_BUTTON 1005
		#define ID_LEFT_DECISION_BUTTON 1006
		#define ID_RIGHT_DECISION_BUTTON 1007
		#define ID_BACK_DECISION_BUTTON 1008
		#define ID_DECISION_GO_BUTTON 1009
		#define ID_GOAL_LIST 1010
		#define ID_GOAL_GO_BUTTON 1011
		#define ID_TASK_OK_BUTTON 1012
		#define ID_TASK_COMPLETE_OK_BUTTON 1013
		#define ID_EXPERIMENT_DONE_BUTTON 1014
		
		///////////////////////////////////////////////////////////////////////////////
		/// Class LogicalFrame
		///////////////////////////////////////////////////////////////////////////////
		class LogicalFrame : public UIMainFrame
		{
			private:
			
			protected:
				GoalPlannerDisplayWidget* plannerWidget;
				wxButton* decisionButton;
				wxButton* goalButton;
				wxButton* stopButton;
			
			public:
				
				LogicalFrame( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("HSSH Logical Interface"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 800,600 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
				
				~LogicalFrame();
			
		};
		
		///////////////////////////////////////////////////////////////////////////////
		/// Class DecisionDialog
		///////////////////////////////////////////////////////////////////////////////
		class DecisionDialog : public wxDialog 
		{
			private:
			
			protected:
				wxToggleButton* forwardDecisionButton;
				wxToggleButton* leftDecisionButton;
				wxToggleButton* rightDecisionButton;
				wxToggleButton* backDecisionButton;
				wxStaticLine* goalSeparator;
				wxButton* decisionGoButton;
			
			public:
				
				DecisionDialog( wxWindow* parent, wxWindowID id = ID_DECISION_DIALOG, const wxString& title = wxT("Decision Command"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 306,372 ), long style = wxDEFAULT_DIALOG_STYLE ); 
				~DecisionDialog();
			
		};
		
		///////////////////////////////////////////////////////////////////////////////
		/// Class GoalDialog
		///////////////////////////////////////////////////////////////////////////////
		class GoalDialog : public wxDialog 
		{
			private:
			
			protected:
				wxListBox* goalList;
				wxButton* goalGoButton;
			
			public:
				
				GoalDialog( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Goal Command"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 258,299 ), long style = wxDEFAULT_DIALOG_STYLE ); 
				~GoalDialog();
			
		};
		
		///////////////////////////////////////////////////////////////////////////////
		/// Class TaskDialog
		///////////////////////////////////////////////////////////////////////////////
		class TaskDialog : public wxDialog 
		{
			private:
			
			protected:
				wxStaticText* taskGoalText;
				wxStaticText* goalIdText;
				wxStaticText* interfaceToUseText;
				wxStaticText* interfaceText;
				wxButton* taskOkButton;
			
			public:
				
				TaskDialog( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Task Assignment"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 464,224 ), long style = wxDEFAULT_DIALOG_STYLE ); 
				~TaskDialog();
			
		};
		
		///////////////////////////////////////////////////////////////////////////////
		/// Class TaskCompleteDialog
		///////////////////////////////////////////////////////////////////////////////
		class TaskCompleteDialog : public wxDialog 
		{
			private:
			
			protected:
				wxStaticText* taskCompleteText;
				wxStaticText* completeHiddenText;
				wxButton* taskCompleteOkButton;
			
			public:
				
				TaskCompleteDialog( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Task Complete"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize, long style = wxDEFAULT_DIALOG_STYLE ); 
				~TaskCompleteDialog();
			
		};
		
		///////////////////////////////////////////////////////////////////////////////
		/// Class ExperimentCompleteDialog
		///////////////////////////////////////////////////////////////////////////////
		class ExperimentCompleteDialog : public wxDialog 
		{
			private:
			
			protected:
				wxStaticText* experimentCompleteText;
				wxStaticText* experimentHiddenText;
				wxButton* experimentDoneButton;
			
			public:
				
				ExperimentCompleteDialog( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Experiment Complete"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize, long style = wxDEFAULT_DIALOG_STYLE ); 
				~ExperimentCompleteDialog();
			
		};
		
	} // namespace ui
} // namespace vulcan

#endif //__LOGICAL_INTERFACE_H__

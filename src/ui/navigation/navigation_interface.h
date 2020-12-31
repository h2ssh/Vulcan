///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version 3.9.0 Dec 31 2020)
// http://www.wxformbuilder.org/
//
// PLEASE DO *NOT* EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#pragma once

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
namespace vulcan
{
namespace ui
{
class NavigationInterfaceDisplay;
}
}   // namespace vulcan

#include "ui/common/ui_main_frame.h"
#include <wx/bitmap.h>
#include <wx/button.h>
#include <wx/colour.h>
#include <wx/dialog.h>
#include <wx/font.h>
#include <wx/frame.h>
#include <wx/gdicmn.h>
#include <wx/icon.h>
#include <wx/image.h>
#include <wx/listbox.h>
#include <wx/settings.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/string.h>
#include <wx/textctrl.h>
#include <wx/tglbtn.h>

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
    NavigationInterface(wxWindow* parent,
                        wxWindowID id = wxID_ANY,
                        const wxString& title = wxT("Navigation Interface"),
                        const wxPoint& pos = wxDefaultPosition,
                        const wxSize& size = wxSize(1000, 800),
                        long style = wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);

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

    // Virtual event handlers, override them in your derived class
    virtual void goalNameTextEntered(wxCommandEvent& event) { event.Skip(); }
    virtual void goalNameEnterPressed(wxCommandEvent& event) { event.Skip(); }
    virtual void nameOkayPressed(wxCommandEvent& event) { event.Skip(); }
    virtual void nameCancelPressed(wxCommandEvent& event) { event.Skip(); }


public:
    GoalNameDialogBase(wxWindow* parent,
                       wxWindowID id = wxID_ANY,
                       const wxString& title = wxT("Name the New Goal"),
                       const wxPoint& pos = wxDefaultPosition,
                       const wxSize& size = wxSize(444, 114),
                       long style = wxDEFAULT_DIALOG_STYLE);
    ~GoalNameDialogBase();
};

}   // namespace ui
}   // namespace vulcan

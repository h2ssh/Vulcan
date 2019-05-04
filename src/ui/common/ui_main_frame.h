/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     ui_main_frame.h
* \author   Collin Johnson
*
* Declaration of UIMainFrame abstract base class.
*/

#ifndef UI_COMMON_UI_MAIN_FRAME_H
#define UI_COMMON_UI_MAIN_FRAME_H

#include <system/module_communicator.h>
#include <utils/repeated_task.h>
#include <wx/wx.h>
#include <wx/frame.h>
#include <wx/glcanvas.h>
#include <map>
#include <memory>
#include <thread>
#include <vector>

class wxAuiNotebook;
class wxAuiNotebookEvent;

namespace vulcan
{
namespace ui
{

class UIPanel;
class OpenGLWidget;

const int TIMER_ID = 923;

/**
* UIMainFrame is the base class for the main display frame of any UI program
* in the Vulcan system. The public interface provides methods that allow the
* UIDirector to establish proper data flow within the application.
* 
* The main frame is responsible for starting, stopping, and updating the UIPanels within
* the UI application. These panels are added on construction, updated at the application 
* framerate, and then shutdown when the application closes. The main frame is
* responsible for maintaining the panels after they have been added.
*
* The external interface needed to use a main frame is as follows:
*
*   - void connectConsumersToDataDistributor(UIDataDistributor) : The
*               data flow in the UI is such that all outside data for the
*               UI goes through a central distributor that sends it to all
*               class instances that need the data. This method is the
*               glue to put the two pieces together.
*
*   - void connectProducersToOutputConsumer(UIOutputConsumer) : Components
*               in the UI are going to produce data that are needed by the
*               rest of the system. This method serves to link the producers
*               to a consumer capable of distributing the data.
*
* 
* For subclasses of the main frame, they need to implement two methods to be called
* after construction of the UIMainFrame is complete, but within the constructor of the subclass
* to ensure that everything is correctly setup before connections with the distributor
* and consumer are established.
* 
*   1) Add all panels for the frame via the addPanel() method. All panels need to be added
*      before any other methods are called, otherwise undefined behavior will result.
*   2) Call initialize() to start the timer that will signal the panel updates.
*/
class UIMainFrame : public wxFrame
{
public:
    
    /**
    * Constructor for UIMainFrame.
    */
    UIMainFrame(wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style);

    virtual ~UIMainFrame(void);

protected:
    
    /**
    * addPanel adds a new panel to be managed by the frame.
    * 
    * The UIMainFrame will take ownership of the pointer.
    * 
    * \param[in]    panel               The panel to be managed
    * \param[in]    window              The window associated with panel
    */
    void addPanel(UIPanel* panel, wxWindow* window);

    /**
    * initialize initializes the timers and other events used for a UI frame.
    * 
    * \param[in]    notebook            Notebook instance that contains the tabs with the panels to be displayed (optional)
    * \param[in]    frameRate           Frames per second to update the UI (default = 15)
    * \param[in]    glWidget            Widget to use for initializing the wxGLContext for the frame (optional)
    * \param[in]    statusBar           Status bar to be assigned to the panels (optional)
    */
    void initialize(wxAuiNotebook* notebook = nullptr, int frameRate = 15, OpenGLWidget* glWidget = nullptr, wxStatusBar* statusBar = nullptr);
    
private:

    std::vector<std::unique_ptr<UIPanel>> panels_;
    std::map<wxWindow*, UIPanel*> windowToPanel_;
    wxAuiNotebook* panelNotebook_;
    int numPushedPanels_;
    
    wxGLContext*           glContext;
    
    system::ModuleCommunicator communicator_;
    std::unique_ptr<utils::RepeatedTask> communicatorTask_;

    wxTimer* refreshTimer;
    

    void subscribeConsumersViaCommunicator(void);
    void setCommunicatorForProducers(void);
    void setupRefreshTimer(int frameRate);
    
    // Event handlers
    void paint     (wxPaintEvent& event);
    void timerFired(wxTimerEvent& event);
    void close     (wxCloseEvent& event);
    void pageChanging(wxAuiNotebookEvent& event);
    void pageChanged (wxAuiNotebookEvent& event);
    
    DECLARE_EVENT_TABLE()
};

}
}

#endif // UI_COMMON_UI_MAIN_FRAME_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     ui_main_frame.cpp
* \author   Collin Johnson
*
* Definition of UIMainFrame abstract base class.
*/

#include "ui/common/ui_main_frame.h"
#include "ui/common/ui_panel.h"
#include "ui/components/open_gl_widget.h"
#include <wx/aui/auibook.h>
#include <iostream>

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(UIMainFrame, wxFrame)
    EVT_PAINT(UIMainFrame::paint)
    EVT_TIMER(TIMER_ID, UIMainFrame::timerFired)
    EVT_CLOSE(UIMainFrame::close)
END_EVENT_TABLE()
    

UIMainFrame::UIMainFrame(wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style)
: wxFrame(parent, id, title, pos, size, style)
, panelNotebook_(nullptr)
, numPushedPanels_(0)
, glContext(nullptr)
{
}


UIMainFrame::~UIMainFrame(void)
{
    assert(numPushedPanels_ >= 0);
    // Remove any remaining event handlers
    while(numPushedPanels_ > 0)
    {
        PopEventHandler();
        --numPushedPanels_;
    }
    // Deleting the unique_ptr will stop the task running the communicator
}


void UIMainFrame::subscribeConsumersViaCommunicator(void)
{
    for(auto& panel : panels_)
    {
        panel->subscribe(communicator_);
    }
}


void UIMainFrame::setCommunicatorForProducers(void)
{
    for(auto& panel : panels_)
    {
        panel->setConsumer(&communicator_);
    }
}


void UIMainFrame::initialize(wxAuiNotebook* notebook, int frameRate, OpenGLWidget* glWidget, wxStatusBar* statusBar)
{
    if(glWidget)
    {
        glContext = new wxGLContext(glWidget);
    }
    
    for(auto& panel : panels_)
    {
        panel->setup(glContext, statusBar);
    }
    
    subscribeConsumersViaCommunicator();
    setCommunicatorForProducers();
    
    if(notebook)
    {
        panelNotebook_ = notebook;
        
        Bind(wxEVT_AUINOTEBOOK_PAGE_CHANGING, &UIMainFrame::pageChanging, this, panelNotebook_->GetId());
        Bind(wxEVT_AUINOTEBOOK_PAGE_CHANGED,  &UIMainFrame::pageChanged,  this, panelNotebook_->GetId());
        
        auto panelIt = windowToPanel_.find(panelNotebook_->GetCurrentPage());
        
        if(panelIt != windowToPanel_.end())
        {
            PushEventHandler(panelIt->second);
            ++numPushedPanels_;
        }
    }
    
    setupRefreshTimer(frameRate);
    
    // Everything else is initialized, so launch the thread to start reading data.
    auto communicatorFunc = [this](bool killed) -> bool
    {
        if(!killed)
        {
            communicator_.processIncoming();
        }

        return !killed;
    };
    communicatorTask_.reset(new utils::RepeatedTask(communicatorFunc));
}


void UIMainFrame::addPanel(UIPanel* panel, wxWindow* window)
{
    if(panel)
    {
        panels_.emplace_back(panel);
        
        if(window)
        {
            windowToPanel_.insert(std::make_pair(window, panel));
        }
    }
}


void UIMainFrame::setupRefreshTimer(int frameRate)
{
    int refreshRate = 100;

    if(frameRate != 0)
    {
        refreshRate = 1000 / frameRate;
    }

    refreshTimer = new wxTimer(this, TIMER_ID);
    refreshTimer->Start(refreshRate, wxTIMER_CONTINUOUS);
}


void UIMainFrame::paint(wxPaintEvent& event)
{
    wxPaintDC dc(this);
}


void UIMainFrame::timerFired(wxTimerEvent& event)
{
    for(auto& panel : panels_)
    {
        panel->update();
    }
}


void UIMainFrame::close(wxCloseEvent& event)
{
    // TODO: Handle elegant cleanup of the frame by saving the settings and bailing out.
    refreshTimer->Stop();
    Destroy();
}


void UIMainFrame::pageChanging(wxAuiNotebookEvent& event)
{
    std::cout << "Page changing from " << event.GetOldSelection() << '\n';
    if(numPushedPanels_ > 0)
    {
        PopEventHandler();
        --numPushedPanels_;
    }
}


void UIMainFrame::pageChanged(wxAuiNotebookEvent& event)
{
    std::cout << "Page changed to " << event.GetSelection() << '\n';
    auto panelIt = windowToPanel_.find(panelNotebook_->GetPage(event.GetSelection()));
    
    if(panelIt != windowToPanel_.end())
    {
        PushEventHandler(panelIt->second);
        ++numPushedPanels_;
    }
}

} // namespace ui
} // namespace vulcan

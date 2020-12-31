/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     logplayer_frame.h
 * \author   Collin Johnson
 *
 * Declaration of LogplayerFrame, which handles events for the LogplayerUI.
 */

#ifndef UI_LOGPLAYER_LOGPLAYER_FRAME_H
#define UI_LOGPLAYER_LOGPLAYER_FRAME_H

#include "ui/logplayer/logplayer_ui.h"
#include "utils/mutex.h"
#include <memory>
#include <string>
#include <vector>
#include <wx/wx.h>

namespace vulcan
{
namespace logplayer
{
class LogPlayer;
}

namespace ui
{

/**
 * LogplayerFrame is the main frame that handles events for the Logplayer UI.
 * The frame acts as an intermediary between the UI and the Logplayer. Events
 * are captured by the frame and then the appropriate calls to the Logplayer
 * are made.
 */
class LogplayerFrame : public LogplayerUI
{
public:
    /**
     * Constructor for LogplayerFrame.
     *
     * \param    player          Logplayer instance to use
     */
    LogplayerFrame(std::unique_ptr<logplayer::LogPlayer> player);

    /**
     * Destructor for LogplayerFrame.
     */
    virtual ~LogplayerFrame(void);

private:
    // Event handlers
    void pressedRestart(wxCommandEvent& event);
    void pressedPlay(wxCommandEvent& event);
    void pressedStep(wxCommandEvent& event);
    void pressedSlower(wxCommandEvent& event);
    void pressedFaster(wxCommandEvent& event);
    void enteredSpeed(wxCommandEvent& event);
    void pressedFileOpen(wxCommandEvent& event);
    void pressedFileQuit(wxCommandEvent& event);
    void movedSlider(wxScrollEvent& event);

    void paint(wxPaintEvent& event);
    void redraw(wxTimerEvent& event);

    std::unique_ptr<logplayer::LogPlayer> player;
    std::string activeLogFile;
    float speed;
    std::vector<float> availableSpeeds;
    unsigned int speedIndex;

    utils::Mutex sliderLock;

    bool isPlaying;

    wxTimer* refreshTimer;   // use a refresh timer to update the log position as the file plays

    DECLARE_EVENT_TABLE()
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_LOGPLAYER_LOGPLAYER_FRAME_H

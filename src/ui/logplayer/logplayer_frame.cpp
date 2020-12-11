/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     logplayer_frame.cpp
* \author   Collin Johnson
*
* Definition of LogplayerFrame.
*/

#include <ui/logplayer/logplayer_frame.h>
#include <ui/common/file_dialog_settings.h>
#include <logging/logplayer/log_player.h>
#include <utils/auto_mutex.h>
#include <wx/event.h>
#include <wx/filedlg.h>
#include <wx/string.h>

namespace vulcan
{
namespace ui
{

const int TIMER_ID = 901;

BEGIN_EVENT_TABLE(LogplayerFrame, wxFrame)
    EVT_BUTTON(ID_RESTART_BUTTON,          LogplayerFrame::pressedRestart)
    EVT_BUTTON(ID_PLAY_BUTTON,             LogplayerFrame::pressedPlay)
    EVT_BUTTON(ID_STEP_BUTTON,             LogplayerFrame::pressedStep)
    EVT_BUTTON(ID_SLOWER_BUTTON,           LogplayerFrame::pressedSlower)
    EVT_BUTTON(ID_FASTER_BUTTON,           LogplayerFrame::pressedFaster)
    EVT_TEXT_ENTER(ID_PLAYBACK_SPEED_TEXT, LogplayerFrame::enteredSpeed)
    EVT_MENU(ID_FILE_OPEN_MENU_ITEM,       LogplayerFrame::pressedFileOpen)
    EVT_MENU(ID_FILE_QUIT_MENU_ITEM,       LogplayerFrame::pressedFileQuit)
    EVT_COMMAND_SCROLL_CHANGED(ID_LOG_POSITION_SLIDER, LogplayerFrame::movedSlider)
    EVT_PAINT(LogplayerFrame::paint)
    EVT_TIMER(TIMER_ID, LogplayerFrame::redraw)
END_EVENT_TABLE()


LogplayerFrame::LogplayerFrame(std::unique_ptr<logplayer::LogPlayer> player)
: LogplayerUI(0)
, player(std::move(player))
, speed(1.0)
, speedIndex(5)
, isPlaying(false)
{
    refreshTimer = new wxTimer(this, TIMER_ID);
    refreshTimer->Start(100, wxTIMER_CONTINUOUS);

    for(int n = -4; n <= 4; ++n)
    {
        availableSpeeds.push_back(std::pow(2.0, n));
    }

    this->player->setPlaybackSpeed(availableSpeeds[speedIndex]);
}


LogplayerFrame::~LogplayerFrame(void)
{
    // For std::unique_ptr
}


// Event handlers
void LogplayerFrame::pressedRestart(wxCommandEvent& event)
{
    player->stop();
}


void LogplayerFrame::pressedPlay(wxCommandEvent& event)
{
    if(!isPlaying)
    {
        player->play(false);
        playButton->SetLabel(wxT("Pause"));
    }
    else
    {
        player->pause();
        playButton->SetLabel(wxT("Play"));
    }

    isPlaying = !isPlaying;
}


void LogplayerFrame::pressedStep(wxCommandEvent& event)
{
    player->step();
}


void LogplayerFrame::pressedSlower(wxCommandEvent& event)
{
    if(speedIndex > 0)
    {
        --speedIndex;
    }

    speed = availableSpeeds[speedIndex];
    player->setPlaybackSpeed(speed);
    playbackSpeedText->ChangeValue(wxString::Format(wxT("%f"), speed));
}


void LogplayerFrame::pressedFaster(wxCommandEvent& event)
{
    if(speedIndex < availableSpeeds.size()-1)
    {
        ++speedIndex;
    }

    speed = availableSpeeds[speedIndex];
    player->setPlaybackSpeed(speed);
    playbackSpeedText->ChangeValue(wxString::Format(wxT("%4f"), speed));
}


void LogplayerFrame::enteredSpeed(wxCommandEvent& event)
{
    wxString text = playbackSpeedText->GetLineText(0);

    double newSpeed = speed;

    if(text.ToDouble(&newSpeed) && (newSpeed > 0.0))
    {
        speed = newSpeed;
        player->setPlaybackSpeed(speed);
    }
}


void LogplayerFrame::pressedFileOpen(wxCommandEvent& event)
{
    wxFileDialog open(this,
                      wxT("Select a log file to load..."),
                      wxT("../logs/datasets"),
                      wxEmptyString,
                      wxEmptyString,
                      kFileOpenFlags);

    if(open.ShowModal() == wxID_OK)
    {
        wxString path = open.GetPath();
        activeLogFile = std::string(path.mb_str());
        player->load(activeLogFile);

        std::size_t localPath = path.rfind('/');
        path = path.substr(localPath+1);
        logFileText->SetLabel(path);
        logPositionSlider->SetRange(0, player->numberOfFrames());
    }
}


void LogplayerFrame::pressedFileQuit(wxCommandEvent& event)
{
    Close();
}


void LogplayerFrame::movedSlider(wxScrollEvent& event)
{
    utils::AutoMutex autoLock(sliderLock);

    player->seek(logPositionSlider->GetValue());
}


void LogplayerFrame::paint(wxPaintEvent& event)
{

}


void LogplayerFrame::redraw(wxTimerEvent& event)
{
    utils::AutoMutex autoLock(sliderLock);

    uint32_t frame = player->currentFrame();
    logPositionSlider->SetValue(frame);
}

} // namespace ui
} // namespace vulcan

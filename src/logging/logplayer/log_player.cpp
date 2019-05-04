/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     log_player.cpp
* \author   Collin Johnson
*
* Definition of LogPlayer.
*/

#include <logging/logplayer/log_player.h>
#include <utils/auto_mutex.h>
#include <logging/logplayer/log_reader.h>
#include <unistd.h>
#include <iostream>

// #define DEBUG_PLAYBACK

namespace vulcan
{
namespace logplayer
{

LogPlayer::LogPlayer(const log_player_params_t& params)
: speed(1.0)
, isPlaying(false)
, loader(params.channels)
, playbackTrigger(false)
, playbackThread(true) // make the thread joinable
, shouldRunThread(true)
, exitWhenFinished(false)
{
    playbackThread.attachTask(this);
    playbackThread.start();
}


bool LogPlayer::load(const std::string& filename, log_type_t type)
{
    utils::AutoMutex autoLock(playbackLock);

    playbackTrigger.setPredicate(false);

    logFile    = filename;
    reader     = loader.loadLog(filename, type);
    frame      = 0;
    timestamps = reader->getTimestamps();

    return reader.get() != 0;
}


bool LogPlayer::play(bool exitWhenLogEnds)
{
    exitWhenFinished = exitWhenLogEnds;

    playbackTrigger.setPredicate(true);
    playbackTrigger.broadcast();
    isPlaying = true;

    return true;
}


void LogPlayer::step(void)
{
    if(!isPlaying)
    {
        reader->sendFrame(frame, communicator);
        ++frame;
    }
}


void LogPlayer::pause(void)
{
    playbackTrigger.setPredicate(false);
    isPlaying = false;
}


void LogPlayer::stop(void)
{
    utils::AutoMutex autoLock(playbackLock);

    playbackTrigger.setPredicate(false);
    isPlaying = false;
    frame     = 0;
}


void LogPlayer::seek(std::size_t frame)
{
    utils::AutoMutex autoLock(playbackLock);

    if(frame < timestamps.size())
    {
        this->frame = frame;
    }
}


std::size_t LogPlayer::currentFrame(void) const
{
    utils::AutoMutex autoLock(playbackLock);
    return frame;
}


void LogPlayer::setPlaybackSpeed(float speed)
{
    utils::AutoMutex autoLock(playbackLock);

    if(speed > 0.0f)
    {
        this->speed = speed;
    }
}


void LogPlayer::waitForEnd(void)
{
    playbackThread.join();
}


int LogPlayer::run(void)
{
    while(shouldRunThread)
    {
        playbackTrigger.wait();

        if(frame < timestamps.size())
        {
            reader->sendFrame(frame, communicator);

            waitForNextFrame();

            ++frame;
        }
        else
        {
            std::cout<<"INFO: Finished playing log file:"<<logFile<<'\n';

            if(exitWhenFinished)
            {
                break;
            }

            // Completed, so block until a new log comes in
            playbackTrigger.setPredicate(false);
            isPlaying = false;
        }
    }

    return 0;
}


void LogPlayer::waitForNextFrame(void)
{
    if(frame < timestamps.size()-1)
    {
        int64_t sleepTime = (timestamps[frame+1] - timestamps[frame]) / speed;

#ifdef DEBUG_PLAYBACK
        std::cout << "DEBUG: LogPlayer: Waiting " << sleepTime << " for next frame.\n";
#endif

        assert(timestamps[frame+1] >= timestamps[frame]);
        usleep(sleepTime);
    }
}

} // namespace logplayer
} // namespace vulcan

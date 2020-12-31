/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     log_player.h
* \author   Collin Johnson
*
* Declaration of LogPlayer.
*/

#ifndef LOGPLAYER_LOG_PLAYER_H
#define LOGPLAYER_LOG_PLAYER_H

#include <vector>
#include "utils/condition_variable.h"
#include "utils/mutex.h"
#include "utils/thread.h"
#include "utils/locked_bool.h"
#include "system/module_communicator.h"
#include "logging/logplayer/log_loader.h"

namespace vulcan
{
namespace logplayer
{

class LogReader;

/**
* LogPlayer plays log files and controls playback of logs.
*/
class LogPlayer : public utils::Threadable
{
public:

    /**
    * Constructor for LogPlayer.
    *
    * \param    params          Parameters for the LogPlayer
    */
    LogPlayer(const log_player_params_t& params);

    /**
    * load loads a new log.
    *
    * \param    filename        Name of the log to be loaded
    * \param    type            Type of the log (optional, default = auto-detect)
    * \return   True if the log was loaded. False otherwise.
    */
    bool load(const std::string& filename, log_type_t type = AUTO_DETECT_LOG_TYPE);

    /**
    * play begins playback of the currently loaded log. If no log is loaded, obviously
    * no playback will actually occur. If the load is currently ended, no playback
    * will occur.
    *
    * \param    exitWhenLogEnds         Flag indicating if the logplayer should exit once the log ends
    * \return   True if the log is being played. False otherwise.
    */
    bool play(bool exitWhenLogEnds);

    /**
    * step increments the playback by one frame. Only works when paused.
    */
    void step(void);

    /**
    * pause suspends playback of the currently loaded log file.
    */
    void pause(void);

    /**
    * stop halts playback of the log and resets the position of the next read operation to the
    * first frame in the log.
    */
    void stop(void);

    /**
    * seek seeks to the specified frame.
    *
    * \param    frame           Frame to jump to
    */
    void seek(std::size_t frame);

    /**
    * numberOfFrames retrieves the number of frames in the current log.
    *
    * \return   Number of frames in the log.
    */
    std::size_t numberOfFrames(void) const { return timestamps.size(); }

    /**
    * currentFrame retrieves the current frame being played.
    *
    * \return   Last frame to be played.
    */
    std::size_t currentFrame(void) const;

    /**
    * setPlaybackSpeed adjusts the speed at which playback occurs. 1.0 = realtime, < 1.0 = slower, > 1.0 = faster.
    *
    * \param    speed           Playback speed
    */
    void setPlaybackSpeed(float speed);

    /**
    * waitForEnd is a blocking call that will wait until log playback has finished before returning.
    */
    void waitForEnd(void);

private:

    // utils::Threadable interface
    virtual int run             (void);
    void        waitForNextFrame(void);

    std::string                  logFile;
    std::size_t                  frame;
    float                        speed;
    utils::LockedBool            isPlaying;
    std::vector<int64_t>         timestamps;
    std::shared_ptr<LogReader> reader;

    LogLoader loader;

    system::ModuleCommunicator communicator;

    // Thread primitives for synchronizing the internally running playback thread with the available controls
    mutable utils::Mutex     playbackLock;
    utils::ConditionVariable playbackTrigger;
    utils::Thread            playbackThread;
    bool                     shouldRunThread;
    bool                     exitWhenFinished;
};

}
}

#endif // LOGPLAYER_LOG_PLAYER_H

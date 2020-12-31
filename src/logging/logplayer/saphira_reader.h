/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     saphira_reader.h
* \author   Collin Johnson
* 
* Declaration of SaphiraReader for reading Saphira log files -- .slf
*/

#ifndef LOGGING_LOGPLAYER_SAPHIRA_READER_H
#define LOGGING_LOGPLAYER_SAPHIRA_READER_H

#include "logging/logplayer/log_reader.h"

namespace vulcan 
{
namespace logplayer
{
    
/**
* SaphiraReader 
*/
class SaphiraReader : public LogReader 
{
public:
    
    using LogReader::LogReader;
    
private:
    
    // LogReader interface
    bool convertLogToFrames(const std::string & filename) override;
    
    data_frame_t readNextFrame(std::ifstream& in, const polar_laser_scan_t& scanConfig);
    
    int64_t timestamp_ = 0;
    bool haveLastPose_ = false;
    pose_t lastPose_;
};

} // namespace logplayer
} // namespace vulcan 

#endif // LOGGING_LOGPLAYER_SAPHIRA_READER_H

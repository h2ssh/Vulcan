/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     saphira_reader.cpp
 * \author   Collin Johnson
 *
 * Definition of SaphiraReader for reading Saphira log files -- .slf
 */

#include "logging/logplayer/saphira_reader.h"
#include <fstream>
#include <iostream>
#include <sstream>

namespace vulcan
{
namespace logplayer
{

const std::string kVersionStr("version");
const std::string kLogTypeStr("LaserOdometryLog");
const std::string kLaserPoseStr("sick1pose");
const std::string kLaserConfStr("sick1conf");
const std::string kScanHeaderStr("#Scan");
const std::string kRobotPoseStr("robot:");
const std::string kRobotIdStr("robot_id:");
const std::string kLaserDataStr("sick1:");

struct scan_header_info_t
{
    std::string version;
    polar_laser_scan_t scanConfig;   // use this as the basic config to store data in
};


scan_header_info_t read_header_information(std::ifstream& in);


bool SaphiraReader::convertLogToFrames(const std::string& filename)
{
    std::ifstream in(filename);

    if (!in.is_open()) {
        std::cerr << "ERROR: SaphiraReader: Failed to open " << filename << '\n';
        return false;
    }

    auto header = read_header_information(in);

    std::cout << "Reading Saphira log version " << header.version << " Laser config:\n"
              << "Pose offset: " << header.scanConfig.offset << '\n'
              << "Scan start:  " << header.scanConfig.startAngle << '\n'
              << "Scan res:    " << header.scanConfig.angularResolution << '\n'
              << "Num ranges:  " << header.scanConfig.numRanges << '\n';

    std::string scanStr;
    int scanId = 0;

    while (!in.eof()) {
        auto frame = readNextFrame(in, header.scanConfig);

        // If a valid frame is read, then add it
        if (frame.haveLaser && frame.haveOdometry) {
            // Timestamp and scan increment info isn't available, so it needs to be deduced
            frame.scan.scanId = scanId++;
            frame.timestamp = timestamp_;
            timestamp_ += 50000;   // assume 20Hz just ot make playback fast
            addFrame(frame);
        }
    }

    return true;
}


LogReader::data_frame_t SaphiraReader::readNextFrame(std::ifstream& in, const polar_laser_scan_t& scanConfig)
{
    data_frame_t frame;
    std::string line;
    std::string type;

    while (std::getline(in, line)) {
        // Once the next scan header is found, this frame is completed
        if (line.find(kScanHeaderStr) != std::string::npos) {
            return frame;
        }

        std::istringstream lineIn(line);

        if (line.find(kRobotPoseStr) != std::string::npos) {
            pose_t pose;
            lineIn >> type >> pose.x >> pose.y >> pose.theta;

            frame.haveOdometry = true;
            frame.odometry.timestamp = timestamp_;
            frame.odometry.x = pose.x / 1000.0;
            frame.odometry.y = pose.y / 1000.0;
            frame.odometry.theta = pose.theta * M_PI / 180.0;

            if (haveLastPose_) {
                frame.odometry.translation = distance_between_points(pose.toPoint(), lastPose_.toPoint());
                frame.odometry.rotation = angle_diff(pose.theta, lastPose_.theta);
            } else {
                frame.odometry.translation = 0.0f;
                frame.odometry.rotation = 0.0f;
            }

            lastPose_ = pose;
            haveLastPose_ = true;
        } else if (line.find(kRobotIdStr) != std::string::npos) {
            // Ignore the robot id
        } else if (line.find(kLaserDataStr) != std::string::npos) {
            lineIn >> type;
            frame.haveLaser = true;
            frame.scan = scanConfig;
            frame.scan.timestamp = timestamp_;
            auto rangeIt = frame.scan.ranges.begin();
            int range = 0;
            while (lineIn >> range) {
                *rangeIt = range / 1000.0;
                ++rangeIt;
                std::cout << range << ' ';
            }
            std::cout << '\n';
        }
    }

    // Some error occurred if we make it out here
    return frame;
}


scan_header_info_t read_header_information(std::ifstream& in)
{
    std::cout << "Saphira log raw header:\n";

    scan_header_info_t header;
    header.scanConfig.laserId = 0;
    header.scanConfig.maxRange = 40.0f;
    header.scanConfig.scanPeriod = 0.025;

    std::string line;
    std::string type;
    while (std::getline(in, line)) {
        if (line.find(kScanHeaderStr) != std::string::npos) {
            return header;
        }

        std::cout << line << '\n';

        if (line.find(kVersionStr) != std::string::npos) {
            std::istringstream verIn(line);
            verIn >> type >> header.version;
        } else if (line.find(kLaserPoseStr) != std::string::npos) {
            std::istringstream poseIn(line);
            poseIn >> type >> header.scanConfig.offset.x >> header.scanConfig.offset.y
              >> header.scanConfig.offset.theta;
            // Convert to meters/radians
            header.scanConfig.offset.x /= 1000.0;
            header.scanConfig.offset.y /= 1000.0;
            header.scanConfig.offset.theta *= M_PI / 180.0;
        } else if (line.find(kLaserConfStr) != std::string::npos) {
            float startAngle = 0.0f;
            float endAngle = 0.0f;
            int numRanges = 0;
            std::istringstream confIn(line);
            confIn >> type >> startAngle >> endAngle >> numRanges;
            header.scanConfig.numRanges = numRanges;
            header.scanConfig.startAngle = startAngle * M_PI / 180.0;
            header.scanConfig.angularResolution = ((endAngle - startAngle) / numRanges) * M_PI / 180.0;
            header.scanConfig.ranges.resize(numRanges, 0);
            header.scanConfig.intensities.resize(numRanges, 0);
        }
    }

    return header;
}

}   // namespace logplayer
}   // namespace vulcan

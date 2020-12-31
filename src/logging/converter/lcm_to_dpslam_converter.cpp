/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "logging/converter/converters.h"
#include "core/laser_scan.h"
#include "core/odometry.h"
#include "lcmtypes/laser/laser_t.h"
#include "lcmtypes/sensors/odometry_t.h"
#include "system/serialized_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <cereal/archives/binary.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/stream.hpp>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

namespace vulcan 
{
namespace logging
{
    
struct log_contents_t
{
    std::vector<polar_laser_scan_t> laserScans;
    std::vector<odometry_t>       odometry;
    std::vector<pose_t>             scanPoses;
    
    std::size_t firstLaserIndex;
    std::size_t lastLaserIndex;
};

const std::string kOldLaserChannel("SENSOR_FRONT_LASER_INTENSITY");
const std::string kNewLaserChannel("SENSOR_LASER_FRONT");
const std::string kOdometryChannel("SENSOR_ODOMETRY");

const int kRayIncrement = 4;

log_contents_t             read_lcm_old_log    (::lcm::LogFile& log);
log_contents_t             read_lcm_new_log    (::lcm::LogFile& log);
std::vector<pose_t> calculate_scan_poses(const log_contents_t& log);
bool                       save_dpslam_log     (const std::string&    filename,
                                                const log_contents_t& contents);
void                       print_dpslam_params (const log_contents_t& log);
void                       print_log_stats     (log_contents_t& log);


void prune_scans_without_odometry(log_contents_t& contents);
void find_valid_laser_indices    (log_contents_t& contents);

pose_t pose_at_time(int64_t timestamp, const std::vector<odometry_t>& odometry);
pose_t pose_diff(const pose_t& lhs, const pose_t& rhs);


bool convert_lcm_to_dpslam(const std::string& filename, bool useOldLogFormat)
{
    /*
    * The DP-SLAM algorithm requires two pieces of information. A laser scan and an odometry reading.
    * The odometry is the position at which the laser data was gathered.
    * 
    * The full LCM log is read.
    * 
    * For each laser scan, find the corresponding scan pose.
    * Write the each scan data and pose to the dpslam log.
    * 
    * Write the parameters and files in which they need to be changed to stdout.
    */
    
    ::lcm::LogFile lcmLog(filename, "r");
    
    if(!lcmLog.good())
    {
        std::cerr << "ERROR: convert_lcm_to_dpslam: Failed to open LCM log for conversion: " << filename << '\n';
        return false;
    }
    
    auto logContents = useOldLogFormat ? read_lcm_old_log(lcmLog) : read_lcm_new_log(lcmLog);
    
    std::sort(logContents.laserScans.begin(), 
              logContents.laserScans.end(), 
              [](const polar_laser_scan_t& lhs, const polar_laser_scan_t& rhs)
                {
                    return lhs.timestamp < rhs.timestamp;
                });
    
    std::sort(logContents.odometry.begin(), 
              logContents.odometry.end(), 
              [](const odometry_t& lhs, const odometry_t& rhs)
              {
                  return lhs.timestamp < rhs.timestamp;
              });
    
    std::cout << "INFO: Read " << logContents.laserScans.size() << " scans and " << logContents.odometry.size() << " odometry readings." << std::endl;
    prune_scans_without_odometry(logContents);
    find_valid_laser_indices    (logContents);
    
    logContents.scanPoses = calculate_scan_poses(logContents);
    
    print_dpslam_params(logContents);
    print_log_stats(logContents);
    
    bool savedLog = save_dpslam_log(filename, logContents);
    
    if(!savedLog)
    {
        std:: cerr << "ERROR: convert_lcm_to_dpslam: Failed to save DP-SLAM log after conversion: " << filename << '\n';
        return false;
    }
    
    return true;
}


log_contents_t read_lcm_old_log(::lcm::LogFile& log)
{
    int laserCount = 0;
    log_contents_t   contents;
    const ::lcm::LogEvent* event = 0;
    
    while((event = log.readNextEvent()))
    {
        if((event->channel == kOldLaserChannel) && (laserCount++ % kRayIncrement == 0))
        {
            vulcan_lcm_laser_with_intensity_t msg;
            polar_laser_scan_t         scan;
            vulcan_lcm_laser_with_intensity_t_decode(event->data, 0, event->datalen, &msg);
            lcm::convert_lcm_to_vulcan(msg, scan);
            contents.laserScans.push_back(scan);
            vulcan_lcm_laser_with_intensity_t_decode_cleanup(&msg);
        }
        else if((event->channel == kNewLaserChannel) && (laserCount++ % kRayIncrement == 0))
        {
            vulcan_lcm_laser_t        msg;
            polar_laser_scan_t scan;
            vulcan_lcm_laser_t_decode(event->data, 0, event->datalen, &msg);
            lcm::convert_lcm_to_vulcan(msg, scan);
            contents.laserScans.push_back(scan);
            vulcan_lcm_laser_t_decode_cleanup(&msg);
        }
        else if(event->channel == kOdometryChannel)
        {
            vulcan_lcm_odometry_t msg;
            odometry_t   odom;
            vulcan_lcm_odometry_t_decode(event->data, 0, event->datalen, &msg);
            lcm::convert_lcm_to_vulcan(msg, odom);
            contents.odometry.push_back(odom);
            vulcan_lcm_odometry_t_decode_cleanup(&msg);
        }
    }
    
    return contents;
}


log_contents_t read_lcm_new_log(::lcm::LogFile& log)
{
    log_contents_t         contents;
    const ::lcm::LogEvent* event = 0;
    
    int laserCount = 0;
    polar_laser_scan_t scan;
    odometry_t       odom;
    
    while((event = log.readNextEvent()))
    {
        try
        {
            boost::iostreams::array_source inputBuf(static_cast<char*>(event->data), event->datalen);
            boost::iostreams::stream<decltype(inputBuf)> input(inputBuf);
            cereal::BinaryInputArchive in(input);
            
            if(event->channel == kOldLaserChannel && (laserCount++ % kRayIncrement == 0))
            {
                in >> scan;
                contents.laserScans.push_back(scan);
            }
            else if(event->channel == kOdometryChannel)
            {
                in >> odom;
                contents.odometry.push_back(odom);
            }
        }
        catch(std::exception& e)
        {
            std::cerr << "EXCEPTION: Failed to unserialize message on channel: " << event->channel << " Error : " << e.what() << " Message discarded!\n";
        }
    }
    
    return contents;
}


std::vector<pose_t> calculate_scan_poses(const log_contents_t& log)
{
    std::vector<pose_t> scanPoses(log.laserScans.size());
    
    for(std::size_t n = 0; n < log.laserScans.size(); ++n)
    {
        scanPoses[n] = pose_at_time(log.laserScans[n].timestamp, log.odometry);
    }
    
    return scanPoses;
}


bool save_dpslam_log(const std::string&    filename,
                     const log_contents_t& contents)
{
    assert(contents.laserScans.size() == contents.scanPoses.size());
    
    std::ofstream out(filename + ".dpslam");
    
    if(!out.good())
    {
        std::cerr << "ERROR: Failed to open DP-SLAM log at " << filename << ".dpslam\n";
        return false;
    }
    
    for(std::size_t n = 0; n < contents.laserScans.size(); ++n)
    {
        const auto& scan = contents.laserScans[n];
        const auto& pose = contents.scanPoses[n];
        
        out << "Odometry " << pose.x << ' ' << pose.y << ' ' << pose.theta << '\n'
            << "Lidar " << ((contents.lastLaserIndex - contents.firstLaserIndex) / kRayIncrement);
        
        for(std::size_t i = contents.firstLaserIndex; i < contents.lastLaserIndex-kRayIncrement; i += kRayIncrement)
        {
            out << ' ' << std::abs(scan.ranges[i]) << ' ' << wrap_to_pi(scan.offset.theta + scan.startAngle + scan.angularResolution*i);
        }
        
        out << '\n';
    }
    
    return true;
}


void print_dpslam_params(const log_contents_t& log)
{
    
}


void print_log_stats(log_contents_t& log)
{
    /*
    * Calculate the adjacent difference of the scan poses.
    * Calculate statistics for the differences:
    *   - mean
    *   - min
    *   - max
    *   - variation
    * Print each
    */
    
    using namespace ::boost::accumulators;
    
    std::vector<pose_t> poseDiffs(log.scanPoses.size());
    std::adjacent_difference(log.scanPoses.begin(), log.scanPoses.end(), poseDiffs.begin(), pose_diff);
    
    accumulator_set<double, stats<tag::mean, tag::variance, tag::max, tag::min>> transAcc;
    accumulator_set<double, stats<tag::mean, tag::variance, tag::max, tag::min>> rotAcc;
    
    for(std::size_t n = 1; n < poseDiffs.size(); ++n)
    {
        transAcc(std::sqrt(poseDiffs[n].x*poseDiffs[n].x + poseDiffs[n].y*poseDiffs[n].y));
        rotAcc(poseDiffs[n].theta);
        
        if(poseDiffs[n].theta > M_PI/4)
        {
            std::cout << "Big pose diff found! Erasing value at N = " << n << " Prev: " << log.scanPoses[n-1] << " Curr: " << log.scanPoses[n] << '\n';
            
            log.laserScans.erase(log.laserScans.begin()+n);
            log.scanPoses.erase(log.scanPoses.begin()+n);
        }
    }
    
    std::cout << "INFO: Translation Stats:\n"
              << "Mean: " << mean(transAcc)     << '\n'
              << "Var:  " << variance(transAcc) << '\n'
              << "Max:  " << max(transAcc)      << '\n'
              << "Min:  " << min(transAcc)      << '\n';
              
    std::cout << "INFO: Rotation Stats:\n"
              << "Mean: " << mean(rotAcc)     << '\n'
              << "Var:  " << variance(rotAcc) << '\n'
              << "Max:  " << max(rotAcc)      << '\n'
              << "Min:  " << min(rotAcc)      << '\n';
}


void prune_scans_without_odometry(log_contents_t& contents)
{
    assert(!contents.odometry.empty());
    
    std::size_t firstValidLaser = 0;
    
    bool foundOdometry = true;
    
    // Find the first valid laser scan
    for(std::size_t n = 0; n < contents.laserScans.size(); ++n)
    {
        if(contents.laserScans[n].timestamp > contents.odometry.front().timestamp)
        {
            firstValidLaser = n;
            break;
        }
    }
    
    // Find the last valid laser scan
    std::size_t finalIndex = 0;
    std::size_t odomIndex = 1;
    
    for(finalIndex = firstValidLaser; foundOdometry && finalIndex < contents.laserScans.size(); ++finalIndex)
    {
        foundOdometry = false;
        int64_t laserTime = contents.laserScans[finalIndex].timestamp;
        
        for(; odomIndex < contents.odometry.size(); ++odomIndex)
        {
            if((contents.odometry[odomIndex-1].timestamp <= laserTime) && 
                (contents.odometry[odomIndex].timestamp > laserTime))
            {
                foundOdometry = true;
                break;
            }
        }
    }
    
    if(!foundOdometry)
    {
        --finalIndex;
    }
    
    std::cout << "Initial time: " << contents.laserScans.front().timestamp << " Odom: " << contents.odometry.front().timestamp << '\n';
    std::cout << "Final time: " << contents.laserScans.back().timestamp << " Odom: " << contents.odometry.back().timestamp << '\n';
    
    // Prune away the invalid laser scans
    contents.laserScans = std::vector<polar_laser_scan_t>(contents.laserScans.begin()+firstValidLaser,
                                                                 contents.laserScans.begin()+finalIndex);
    
    std::cout << "INFO: Number of valid scans read: " << contents.laserScans.size() << '\n';
}


void find_valid_laser_indices(log_contents_t& contents)
{
    if(contents.laserScans.empty())
    {
        return;
    }
    
    const polar_laser_scan_t& scan = contents.laserScans.front();
    
    contents.firstLaserIndex = scan.ranges.size();
    contents.lastLaserIndex  = 0;
    
    for(std::size_t n = 0; n < scan.ranges.size(); ++n)
    {
        if(scan.ranges[n] > 0.0f)
        {
            if(n < contents.firstLaserIndex)
            {
                contents.firstLaserIndex = n;
            }
            else if(n > contents.lastLaserIndex)
            {
                contents.lastLaserIndex = n;
            }
        }
    }
    
    if(contents.firstLaserIndex > contents.lastLaserIndex)
    {
        std::cerr << "ERROR: Invalid log. There are no valid rays.\n";
    }
}


pose_t pose_at_time(int64_t timestamp, const std::vector<odometry_t>& odometry)
{
    assert(!odometry.empty());
    
    for(std::size_t n = 1; n < odometry.size(); ++n)
    {
        if(odometry[n-1].timestamp <= timestamp && odometry[n].timestamp >= timestamp)
        {
            return interpolate_pose(pose_t(odometry[n-1].timestamp,
                                                         odometry[n-1].x,
                                                         odometry[n-1].y,
                                                         odometry[n-1].theta),
                                           pose_t(odometry[n].timestamp,
                                                         odometry[n].x,
                                                         odometry[n].y,
                                                         odometry[n].theta),
                                           timestamp);
        }
    }
    
    std::cerr << "ERROR: Unable to find pose at " << timestamp << ". Didn't have odometry with values on both sides."
              << " Final odometry time: " << odometry.back().timestamp << '\n';
    assert(false);
    return pose_t(odometry.back().timestamp,
                         odometry.back().x,
                         odometry.back().y,
                         odometry.back().theta);
}


pose_t pose_diff(const pose_t& lhs, const pose_t& rhs)
{
    return pose_t(std::abs(lhs.x - rhs.x),
                         std::abs(lhs.y - rhs.y),
                         std::abs(angle_diff(lhs.theta, rhs.theta)));
}

} // namespace logging
} // namespace vulcan

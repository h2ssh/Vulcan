/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     hokuyo_urg_laser.cpp
 * \author   Collin Johnson
 *
 * Definition of HokyoURGLaser.
 */

#include "sensors/hokuyo_urg_laser.h"
#include "math/statistics.h"
#include "utils/timestamp.h"
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <urg/ScipHandler.h>

namespace vulcan
{
namespace sensors
{

using qrk::RangeSensorParameter;
using qrk::UrgDevice;


// Helpers for basic output
void print_laser_params(const std::vector<std::string>& deviceInfo, const qrk::RangeSensorParameter& params);

// Helper functions for interacting with the UrgUtil class
void set_capture_parameters(UrgDevice& controller, const RangeSensorParameter& params);
void convert_urg_data_to_laser_scan(polar_laser_scan_t& scan,
                                    const std::vector<long>& ranges,
                                    const std::vector<long>& intensities,
                                    const RangeSensorParameter& scanParams);

HokuyoURGLaser::HokuyoURGLaser(const std::string& port, int id, bool withIntensity, bool verbose)
: id(id)
, lastLaserTimestamp(0)
{
    int numRetries = 10;
    while ((--numRetries >= 0) && !laserController.connect(port.c_str())) {
        sleep(1);
    }

    if (numRetries < 0) {
        std::cerr << "ERROR: Failed to connect to laser on port " << port << '\n';
        assert(laserController.isConnected());
    }

    laserParams = laserController.parameter();

    if (verbose) {
        std::vector<std::string> versionInfo;
        laserController.versionLines(versionInfo);

        print_laser_params(versionInfo, laserParams);
    }

    // Currently gathering the full scan every iterations
    set_capture_parameters(laserController, laserParams);

    if (withIntensity) {
        captureMode = qrk::IntensityCapture;
    } else   // range data only
    {
        captureMode = qrk::AutoCapture;
    }

    scanPeriod = (60.0 / laserParams.scan_rpm) * (laserParams.area_max - laserParams.area_min)
      / static_cast<float>(laserParams.area_total);
}


std::string HokuyoURGLaser::getSerialNumber(void) const
{
    if (!laserController.isConnected()) {
        std::cerr << "ERROR: HokuyoURGLaser: Cannot retrieve serial number of disconnected laser!\n";
        return "ERROR";
    }

    std::vector<std::string> versionInfo;
    laserController.versionLines(versionInfo);

    // From the SCIP2.0 protocol, the serial number will be labelled with SERI:, then be H......;
    for (auto& info : versionInfo) {
        if (info.find("SERI:") != std::string::npos) {
            auto startIndex = info.find('H');
            auto endIndex = info.find(';');

            return info.substr(startIndex, endIndex - startIndex);
        }
    }

    return "ERROR";
}


int64_t HokuyoURGLaser::calculateLatency(void)
{
    if (!laserController.isConnected()) {
        std::cerr << "ERROR: HokuyoURGLaser: Cannot calculate latency of disconnected laser!\n";
        return 0;
    }

    qrk::ScipHandler scip;
    scip.setConnection(laserController.connection());

    scip.setRawTimestampMode(true);

    int timestamp = 0;
    std::vector<int64_t> offsets;

    for (int n = 0; n < 100; ++n) {
        int64_t startTime = utils::system_time_us();
        scip.rawTimestamp(&timestamp);
        int64_t stopTime = utils::system_time_us();
        offsets.push_back((stopTime - startTime) / 2);

        usleep(30000);
    }

    scip.setRawTimestampMode(false);

    laserLatencyUs = math::median(offsets.begin(), offsets.end());

    std::cout << "Estimated latency:" << laserLatencyUs << '\n';
    return laserLatencyUs;
}


HokuyoURGLaser::~HokuyoURGLaser(void)
{
    if (laserController.isConnected()) {
        laserController.disconnect();
    }
}


bool HokuyoURGLaser::startRangefinder(void)
{
    // Setting the capture mode to AutoCapture immediately begins data acquisition
    laserController.setCaptureMode(captureMode);

    return true;
}


void HokuyoURGLaser::stopRangefinder(void)
{
    laserController.stop();
}


bool HokuyoURGLaser::isScanning()
{
    return laserController.captureMode() == captureMode;
}


bool HokuyoURGLaser::getLaserScan(polar_laser_scan_t& scan)
{
    if (!isScanning()) {
        startRangefinder();
        usleep(100000);
        calculateLatency();
    }

    long scanTimestamp = 0;
    int64_t captureTime = 0;

    // Keep trying to read data from the laser until something is actually received
    if (captureMode == qrk::IntensityCapture) {
        while (laserController.captureWithIntensity(rawScanData, rawIntensityData, &scanTimestamp) <= 0) {
            usleep(1000);
        }
    } else   // no intensity data being taken
    {
        while (laserController.capture(rawScanData, &scanTimestamp) <= 0) {
            usleep(1000);
        }
    }

    // Scan timestamp units are milliseconds measured when the first value of the scan was taken
    // Need to convert to microseconds to ensure the offset has the correct units
    captureTime = time_.timestamp(scanTimestamp * 1000);

    convert_urg_data_to_laser_scan(scan, rawScanData, rawIntensityData, laserParams);

    scan.laserId = id;
    scan.scanPeriod = scanPeriod;
    scan.angularResolution =
      2.0 * M_PI / laserParams.area_total;   // area_total = # readings/360degrees. weird name for that
    scan.startAngle = -(laserParams.area_front - laserParams.area_min) * scan.angularResolution;
    scan.maxRange = laserParams.distance_max * 0.001;
    scan.timestamp = captureTime;   // - laserLatencyUs - 60*1000000 / laserParams.scan_rpm;

    return true;
}


// Helpers for basic output
void print_laser_params(const std::vector<std::string>& deviceInfo, const qrk::RangeSensorParameter& params)
{
    std::cout << "INFO: HokuyoURGLaser: Initialized new HokuyoURGLaser: ";

    for (auto& s : deviceInfo) {
        std::cout << '\n' << s;
    }

    std::cout << "\nModel: " << params.model << "\nMininum distance: " << params.distance_min << "mm"
              << "\nMaximum distance: " << params.distance_max << "mm"
              << "\nAngular resolution: " << (2 * M_PI / params.area_total) << " rad"
              << "\nFirst scan index: " << params.area_min << "\nLast scan index: " << params.area_max
              << "\nFront index: " << params.area_front << "\nScan RPM: " << params.scan_rpm << std::endl;
}


// Helper functions for interacting with the UrgUtil class
void set_capture_parameters(UrgDevice& controller, const RangeSensorParameter& params)
{
    controller.setCaptureFrameInterval(0);   // capture all frames
    controller.setCaptureSkipLines(0);       // don't skip lines when doing the capture -- want ALL scan lines
    controller.setCaptureTimes(0);           // acquire data forever!

    controller.setCaptureRange(params.area_min, params.area_max);
}


void convert_urg_data_to_laser_scan(polar_laser_scan_t& scan,
                                    const std::vector<long>& ranges,
                                    const std::vector<long>& intensities,
                                    const RangeSensorParameter& scanParams)
{
    // Data returned by the laser is in millimeters
    int numRanges = scanParams.area_max - scanParams.area_min + 1;

    scan.ranges.resize(numRanges);

    if (!intensities.empty()) {
        scan.intensities.resize(numRanges);
    }

    for (int x = scanParams.area_min, end = ranges.size(); x < end; ++x) {
        // Value below min distance indicates an error, so assign a zero distance so the error is known
        if (ranges[x] >= scanParams.distance_min) {
            scan.ranges[x - scanParams.area_min] = ranges[x] * 0.001;
        } else if (ranges[x] == 1)   // no object in range, so return maximum distance
        {
            scan.ranges[x - scanParams.area_min] = scan.maxRange;
        } else   // all other errors are unrecoverable, so just ignore these readings
        {
            scan.ranges[x - scanParams.area_min] = -ranges[x];
        }

        // Only copy the intensity data if it was captured
        if (!intensities.empty()) {
            scan.intensities[x - scanParams.area_min] = intensities[x];
        }
    }

    scan.numRanges = numRanges;
}

}   // namespace sensors
}   // namespace vulcan

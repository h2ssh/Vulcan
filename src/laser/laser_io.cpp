/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     laser_io.cpp
 * \author   Collin Johnson
 *
 * Definition of laser scan I/O functions.
 */

#include "laser/laser_io.h"
#include "core/laser_scan.h"
#include <fstream>

namespace vulcan
{
namespace laser
{

void allocate_ranges_if_needed(polar_laser_scan_t& scan, uint16_t numRanges, bool haveIntensity);


bool save_laser_scan_to_file(const polar_laser_scan_t& scan, std::ofstream& file)
{
    if (!file.is_open()) {
        return false;
    }

    file << scan.laserId << ' ' << scan.timestamp << ' ' << scan.scanId << ' ' << scan.startAngle << ' '
         << scan.angularResolution << ' ' << scan.maxRange << ' ' << scan.scanPeriod << ' ' << scan.offset.x << ' '
         << scan.offset.y << ' ' << scan.offset.theta << ' ' << (!scan.intensities.empty()) << ' ' << scan.numRanges
         << ' ';

    for (uint16_t n = 0; n < scan.numRanges; ++n) {
        file << scan.ranges[n] << ' ';
    }

    if (!scan.intensities.empty()) {
        for (uint16_t n = 0; n < scan.numRanges; ++n) {
            file << scan.intensities[n] << ' ';
        }
    }

    file << '\n';

    return true;
}


bool load_scan_from_file(std::ifstream& file, polar_laser_scan_t& scan)
{
    if (!file.is_open()) {
        return false;
    }

    bool haveIntensity = false;
    uint16_t numRanges = 0;
    int laserType = 0;

    file >> scan.laserId >> scan.timestamp >> scan.scanId >> laserType >> scan.startAngle >> scan.angularResolution
      >> scan.maxRange >> scan.scanPeriod >> scan.offset.x >> scan.offset.y >> scan.offset.theta >> haveIntensity
      >> numRanges;

    allocate_ranges_if_needed(scan, numRanges, haveIntensity);

    for (uint16_t n = 0; n < scan.numRanges; ++n) {
        file >> scan.ranges[n];
    }

    if (haveIntensity) {
        for (uint16_t n = 0; n < scan.numRanges; ++n) {
            file >> scan.intensities[n];
        }
    }

    return true;
}


void allocate_ranges_if_needed(polar_laser_scan_t& scan, uint16_t numRanges, bool haveIntensity)
{
    scan.ranges.resize(numRanges);

    if (haveIntensity) {
        scan.intensities.resize(numRanges);
    }
}

}   // namespace laser
}   // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     laser_io_test.cpp
* \author   Collin Johnson
*
* Simple test program to make sure the save/load functions for laser scans are working.
*/

#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <laser/laser_io.h>
#include <core/laser_scan.h>
#include <core/float_comparison.h>

using namespace vulcan;
using namespace vulcan::laser;

void create_random_scan               (polar_laser_scan_t& scan);
void create_random_scan_with_intensity(polar_laser_scan_t& scan);

bool compare_scans(const polar_laser_scan_t& random, const polar_laser_scan_t& loaded);

int main(int argc, char** argv)
{
    // Create a regular scan and a scan with intensity. Save them. Load them. Compare them.
    std::ofstream regularOut  ("laser_io_test_regular.scan");
    std::ofstream intensityOut("laser_io_test_intensity.scan");

    polar_laser_scan_t regular;
    polar_laser_scan_t intensity;

    create_random_scan(regular);
    create_random_scan_with_intensity(intensity);

    save_laser_scan_to_file(regular,   regularOut);
    save_laser_scan_to_file(intensity, intensityOut);

    regularOut.close();
    intensityOut.close();

    polar_laser_scan_t loadedRegular;
    polar_laser_scan_t loadedIntensity;

    std::ifstream regularIn  ("laser_io_test_regular.scan");
    std::ifstream intensityIn("laser_io_test_intensity.scan");

    load_scan_from_file(regularIn,   loadedRegular);
    load_scan_from_file(intensityIn, loadedIntensity);

    int success = 0;

    if(!compare_scans(regular, loadedRegular))
    {
        std::cout<<"ERROR: laser_io_test: Loaded regular scan failed to match original regular scan.\n";
        success = 1;
    }
    else
    {
        std::cout<<"SUCCESS: laser_io_test: Loaded regular scan matched original regular scan.\n";
    }

    if(!compare_scans(intensity, loadedIntensity))
    {
        std::cout<<"ERROR: laser_io_test: Loaded intensity scan failed to match original intensity scan.\n";
        success = 1;
    }
    else
    {
        std::cout<<"SUCCESS: laser_io_test: Loaded intensity scan matched original intensity scan.\n";
    }

    return success;
}


void create_random_scan(polar_laser_scan_t& scan)
{
    scan.timestamp         = 123;
    scan.scanId            = 410;
    scan.angularResolution = 0.25f;
    scan.startAngle        = 0.0f;
    scan.maxRange          = 30.0f;
    scan.offset.x          = 1.0f;
    scan.offset.y          = 3.1f;
    scan.offset.theta      = 2.0f;

    scan.numRanges = 1000;

    scan.ranges.resize(scan.numRanges);

    for(uint16_t n = 0; n < scan.numRanges; ++n)
    {
        scan.ranges[n] = drand48() * scan.maxRange;
    }
}


void create_random_scan_with_intensity(polar_laser_scan_t& scan)
{
    create_random_scan(scan);

    scan.intensities.resize(scan.numRanges);

    for(uint16_t n = 0; n < scan.numRanges; ++n)
    {
        scan.intensities[n] = drand48() * 65535;
    }
}


bool compare_scans(const polar_laser_scan_t& random, const polar_laser_scan_t& loaded)
{
    assert(random.timestamp         == loaded.timestamp);
    assert(random.scanId            == loaded.scanId);
    assert(random.angularResolution == loaded.angularResolution);
    assert(random.startAngle        == loaded.startAngle);
    assert(random.maxRange          == loaded.maxRange);
    assert(random.offset.x          == loaded.offset.x);
    assert(random.offset.y          == loaded.offset.y);
    assert(random.offset.theta      == loaded.offset.theta);
    assert(random.numRanges         == loaded.numRanges);

    for(uint16_t n = 0; n < random.numRanges; ++n)
    {
        assert(vulcan::absolute_fuzzy_equal(random.ranges[n], loaded.ranges[n]));
    }

    if(!random.intensities.empty())
    {
        assert(loaded.intensities.empty());
    }

    return true;
}

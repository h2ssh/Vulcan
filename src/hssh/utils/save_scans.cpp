/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     save_scans.cpp
* \author   Paul Foster
*
* Functions to allow saving the scans used in mapping to file
*
*   - accumulate_scan  :  stores a copy of the scan to a list of scans used
*   - save_scans       :  dumps the stored scans to disk
*/

#include "hssh/metrical/mapping/map_builder.h"
#include "laser/moving_laser_scan.h"
#include <string>
#include <vector>

namespace vulcan
{
namespace hssh
{

/**
 * scan_save_data_t stores the part of the scan data to save to file later
 *
 */
struct scan_save_data_t
{
    int64_t timestamp;
    int laserId;
    float x;// approximate x of laser for scan
    float y;// approximate y of laser for scan
    float thetaStart;
    float thetaEnd;
    std::vector<laser::adjusted_ray_t> rays;
};

static std::vector<scan_save_data_t> log;

/**
* accumulate_scan
*
* \param    scan      Maximum number of features to consider
*/
void accumulate_scan(const map_update_data_t& data)
{
    scan_save_data_t scanData;
    scanData.timestamp = data.timestamp;
    scanData.laserId = data.scan.laserId();

    std::vector<laser::adjusted_ray_t> rays(data.scan.begin(), data.scan.end());
    auto& midray = rays[rays.size()/2];

    scanData.x = midray.position.x;
    scanData.y = midray.position.y;
    scanData.thetaStart = angle_to_point(rays.front().position, rays.front().endpoint);
    scanData.thetaEnd = angle_to_point(rays.back().position, rays.back().endpoint);
    scanData.rays = std::move(rays);

    log.push_back(std::move(scanData));

    std::cout << "pushed:" << log.size() << '\n';
}

#define w(r) {const auto& d=r;fwrite(&d,sizeof d,1,f);}
/**
* save_scans
*
* \param    filename File to save to
*/
void save_scans(const std::string& filename) {
    FILE* f;
    f = fopen(filename.c_str(), "wb");
    for(scan_save_data_t& s:log){
        w(s.timestamp);
        w(s.laserId);
        w(s.x);
        w(s.y);
        w(s.thetaStart);
        w(s.thetaEnd);
        w(s.rays.size());
        for(auto& r : s.rays){
            w(r.position.x);
            w(r.position.y);
            w(r.endpoint.x);
            w(r.endpoint.y);
            w(r.range);
        }
    }
    fclose(f);
}

} // namespace hssh
} // namespace vulcan

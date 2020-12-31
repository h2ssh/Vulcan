/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     laser_scan_rasterizer.h
* \author   Collin Johnson
*
* Declaration of LaserScanRasterizer for fast occupancy grid mapping.
*/

#ifndef HSSH_LOCAL_METRIC_MAPPING_LASER_SCAN_RASTERIZER_H
#define HSSH_LOCAL_METRIC_MAPPING_LASER_SCAN_RASTERIZER_H

#include "core/point.h"
#include "utils/tiled_cell_grid.h"
#include "hssh/metrical/mapping/mapping_params.h"

namespace vulcan
{
struct pose_t;

namespace laser { class MovingLaserScan; }
namespace robot { struct velocity_t; }

namespace hssh
{

using RasterGrid = utils::TiledCellGrid<int8_t, 8>;

/**
* LaserScanRasterizer rasterizes a laser scan into a CellGrid in the current LPM's reference frame. The rasterization
* works by ray tracing each laser ray from the laser pose to the endpoint. The beam divergence of the laser ray is
* ignored.
*
* All cells between the laser and the end of the ray are marked as free space. They will have a negative value, which
* indicates a decreased likelihood of occupancy. The endpoints will have a positive value because they indicate occupied
* space. In the case of overlapping rays, an occupied cell always trumps a free cell. For example, if a cell has been
* marked as occupied and another ray would mark it free, the cell retains the value associated with an occupied cell.
*
* Rays longer than the maximum allowed mapping range are treated specially. These rays can happen in a long hallway, at
* high angles of incidence to a surface, or some other error. An easy approach would simply toss them aside, but that
* doesn't work well when traveling down a hallway because the middle of the hallway will remain unmapped. However, mapping
* the full ray can degrade the map far from the laser. The currently implemented solution is as follows:
*
*   1) Find the last valid ray before the too long ray.
*   2) Find the next valid ray after the too long ray.
*   3) For all rays between the valid rays, treat the distance as:
*             (min(last.range, next.range) * scaleFactor
*
* By using this approach, the middle of the hallway will be marked free to about the same distance as the furthest valid
* scan on either side of the hallway.
*
* The scan rasterizer can also incorporate covariance-weighting for the rays. The covariance-weighting is described in more
* detail in hssh/utils/covariance_based_scan_weighter.h. The basic idea is incorporate the robot's pose uncertainty into the
* update values for each cell. A scan from very sure pose will update the map significantly more than from an unsure pose.
* Therefore, updating the map is less likely to break during times of poor localization.
*
* The laser raster has the following properties to determine the visibility/occupancy of the cells:
*
*   - value > 0  : seen and occupied
*   - value == 0 : unseen
*   - value < 0  : seen and unoccupied
*
* The actual values taken correspond to the LPM cost changes. Other builders only need to know the flag, so the above will work
* for that.
*/
class LaserScanRasterizer
{
public:

    /**
    * Constructor for LaserScanRasterizer.
    *
    * \param    cellScale           Discretization level for the rasterization
    * \param    params              Parameters for the rasterizer
    */
    LaserScanRasterizer(float cellScale, const laser_scan_rasterizer_params_t& params);

    // LaserScanRasterizer does not have value semantics
    LaserScanRasterizer(const LaserScanRasterizer& copy) = delete;
    void operator=(const LaserScanRasterizer& rhs)       = delete;

    // Accessors
    const RasterGrid& getRaster(void) const { return raster; }

    /**
    * rasterizeScan creates an update for the current laser scan. The scan area is rasterized into a grid
    * represented by a laser_scan_raster_t. See laser_scan_raster_t for the exact output. The laser_scan_raster_t is
    * in global coordinates.
    *
    * \param    scan                Scan to use for this update of the grid
    * \param    lpmReferenceFrame   The reference frame of the LPM, which is the location of the bottom left corner
    * \param    initialUpdate       Flag indicating if this is the initial update for the given map, and thus use the initial update changes instead of default
    * \return   Rasterized area for the scan.
    */
    void rasterizeScan(const laser::MovingLaserScan& scan,
                       const Point<float>&     lpmReferenceFrame,
                       bool                          initialUpdate = false);

private:

    int8_t occupiedCellCost;
    int8_t freeCellCost;

    RasterGrid raster;

    std::vector<float> adjustedRanges_;

    Point<float> updateBottomLeft;
    Point<float> updateTopRight;

    laser_scan_rasterizer_params_t params;


    void initializeUpdate       (bool initialUpdate);
    void adjustMaxDistanceRanges(const laser::MovingLaserScan& scan);
    void calculateScanBoundary  (const laser::MovingLaserScan& scan,
                                 const Point<float>&     lpmReferenceFrame);
    void updateBufferWithLaserData(const laser::MovingLaserScan& scan);

    // Buffer update methods for laser data
    void scoreRay     (float radius, float angle, const Point<float>& laserCell);
    void scoreEndpoint(const Point<float>& endpoint);
};

}
}

#endif // HSSH_LOCAL_METRIC_MAPPING_LASER_SCAN_RASTERIZER_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     tracking_laser_scan.h
 * \author   Collin Johnson
 *
 * Declaration of ObjectDetector.
 */

#ifndef TRACKER_TRACKING_LASER_SCAN_H
#define TRACKER_TRACKING_LASER_SCAN_H

#include "core/laser_scan.h"
#include "tracker/laser_object_collection.h"
#include <fstream>

namespace vulcan
{
struct pose_distribution_t;
namespace hssh
{
class LocalPerceptualMap;
}
namespace laser
{
class MovingLaserScan;
}
namespace laser
{
struct adjusted_ray_t;
}
namespace utils
{
class ConfigFile;
}
namespace tracker
{

/**
 * object_detector_params_t defines the parameters that control the laser objects being detected.
 *
 * The configuration file parameters for the LaserObjectDetector are:
 *
 *   [ObjectDetectionParameters]
 *   max_adjacent_ray_distance_m = maximum distance between adjacent rays for the rays to be considered in the same
 * cluster (meters) min_cluster_rays            = minimum number of consecutive range measurements within the distance
 * threshold for an object to counted as a cluster max_tracking_distance_m     = maximum distance from the robot to
 * track an object (meters) laser_variance              = variance of the laser points measuring the detected objects
 *   should_filter_glass_rays    = flag indicating if rays intersecting glass cells should be filtered out
 *   should_save_positions       = flag indicating if the detected object positions should be saved to file for analysis
 */
struct object_detector_params_t
{
    float maxAdjacentRayDistance;
    std::size_t minClusterSize;
    float maxTrackingDistance;
    float laserVariance;
    bool filterGlassRays;
    bool savePositions;

    object_detector_params_t(void) = default;
    object_detector_params_t(const utils::ConfigFile& config);
};

/**
 * ObjectDetector represents a single laser scan gathered by the robot. The laser scan is taken while the robot is
 * exploring an environment that contains both static and dynamic objects. The ObjectDetector segregates the static
 * from dynamic objects by first determining which rays from the laser scan fall in free space. These scan points can
 * then be segmented into distinct objects via the detectObjects method.
 *
 * Objects are detected in a laser scan by locating clusters of adjacent range measurements that
 * all fall within some threshold distance of one another. The intuition behind the clustering approach is
 * that connected components in the scan can be identified by two metrics:
 *
 *   1) Two rays in scan is connected if a slope dr/dRay is nearly normal to direction of the rays.
 *   2) The connected component, or a cluster, is valid only when it is larger than a typical leg diameter.
 *
 * These two simple criteria segment a laser scan into relatively a small number of clusters. Two
 * legs are frequently identified as independent clusters, so the clusters that are closer than size of a typical human
 * body are merged.
 */
class ObjectDetector
{
public:
    /**
     * Constructor for ObjectDetector.
     */
    ObjectDetector(const object_detector_params_t& params);

    /**
     * detectObjects segments a laser scan into a set of LaserObjects representing the clusters of similar adjacent
     * measurements found in the scan. The objects found are represented in the global coordinate frame of the robot.
     *
     * \param    scans       Raw laser scans to find dynamic rays and cluster
     * \param    pose        Pose of the robot when the scan was taken
     * \param    lpm         LPM of the current environment
     * \return   A LaserObjectCollection of objects detected in the scan.
     */
    LaserObjectCollection detectObjects(const std::vector<laser::MovingLaserScan>& scans,
                                        const pose_distribution_t& pose,
                                        const hssh::LocalPerceptualMap& lpm);

private:
    const object_detector_params_t params_;

    Position laserPosition_;
    std::vector<bool> isDynamic_;
    std::vector<Point<float>> laserEndpoints_;

    std::ofstream positionOut_;

    void identifyDynamicRays(const laser::MovingLaserScan& scan, const hssh::LocalPerceptualMap& lpm);
    bool doesRayIntersectGlass(const laser::adjusted_ray_t& ray, const hssh::LocalPerceptualMap& lpm);
    std::vector<LaserObject> segmentScan(const laser::MovingLaserScan& scan, const pose_distribution_t& robotPose);
    LaserObjectCollection mergeCollections(const std::vector<LaserObjectCollection>& collections);
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_TRACKING_LASER_SCAN_H

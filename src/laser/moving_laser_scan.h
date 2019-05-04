/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     moving_laser_scan.h
* \author   Collin Johnson
*
* Definition of MovingLaserScan.
*/

#ifndef LASER_MOVING_LASER_SCAN_H
#define LASER_MOVING_LASER_SCAN_H

#include <core/laser_scan.h>
#include <core/pose.h>
#include <core/point.h>
#include <utils/strided_sequence.h>
#include <utils/timestamp.h>
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <cassert>
#include <cmath>

namespace vulcan
{
namespace laser
{

/**
* adjusted_ray_t holds the robot position at the time a ray was taken, along with the theta in global coordinates along
* which the ray was shooting.
*/
struct adjusted_ray_t
{
    Point<float> position;        ///< Global position of the robot when ray was taken
    Point<float> endpoint;        ///< Global position of the endpoint of the ray
    float              range;           ///< Range measured by the ray (< 0 = error)
    float              angle;           ///< Angle in global coordinates (atan2(endpoint.y-position.y), (endpoint.x-position.x))
    uint16_t           intensity = 0;   ///< Measured intensity of the ray (default is 0, not all logs have intensity)
};

template <class Archive>
void serialize(Archive& ar, adjusted_ray_t& ray)
{
    ar( ray.position,
        ray.endpoint,
        ray.range,
        ray.angle,
        ray.intensity
    );
}

/**
* MovingLaserScan calculates the Cartesian endpoints for a laser scan that is on a moving robot. When the robot is
* moving, especially rotating, the pose of the laser is moving as well. Thus, the pose of the laser at the beginning
* of a scan is different than the pose at the end.
*
* MovingLaserScan accounts for the motion of the robot while the scan is happening. It currently only considers the
* angular velocity. The angular velocity is assumed to be constant for the entire scan period. The overall change is
* accumulated as the scan is taken. The first ray corresponds to the theta when the capture started. The last ray
* corresponds to theta + angVel*scanPeriod.
*/
class MovingLaserScan
{
public:

    using RayIter = std::vector<adjusted_ray_t>::const_iterator;

    /**
    * Default constructor for MovingLaserScan.
    */
    MovingLaserScan(void)
    : id_(-1)
    , timestamp_(0)
    {
    }

    /**
    * Constructor for MovingLaserScan.
    *
    * \param    scan                Scan in which to account for motion
    * \param    previousPose        Pose of the robot before the scan started being taken
    * \param    currentPose         Pose of the robot after the scan was finished
    * \param    stride              Stride to use for selecting rays to consider
    */
    MovingLaserScan(const polar_laser_scan_t& scan,
                    const pose_t&      previousPose,
                    const pose_t&      currentPose,
                    int                       stride = 1)
    : id_(scan.laserId)
    , timestamp_(scan.timestamp)
    {
        /*
        * The laser is assumed to run at a constant velocity. The timestamp for the laser measures the time the
        * *last*ray was measured. The previous and current poses give a change in pose assumed to be constant over the
        * timestamp indicated, (dX,dY, dTheta).
        *
        * Given the start and end time of the scan, the starting pose and ending pose of the scan can be calculated.
        * Between these poses, the increment for each ray is constant. This allows all the adjusted rays to be
        * calculated for the MovingScan.
        *
        * Need:
        *   currentPose.timestamp  >= scan.timestamp
        *   previousPose.timestamp <= scanStartTime
        */
        assert(scan.numRanges > 0);
        assert(previousPose.timestamp <= currentPose.timestamp);

        double deltaTime  = utils::usec_to_sec(currentPose.timestamp - previousPose.timestamp);
        double deltaX     = currentPose.x - previousPose.x;
        double deltaY     = currentPose.y - previousPose.y;
        double deltaTheta = angle_diff(currentPose.theta, previousPose.theta);

        double scanDuration  = ((scan.scanPeriod * std::abs(scan.numRanges * scan.angularResolution)) / (2.0 * M_PI));
        double scanStartTime = utils::usec_to_sec(scan.timestamp  - previousPose.timestamp);

        if((deltaTime == 0.0) || (previousPose.timestamp > scan.timestamp) || (scan.timestamp > currentPose.timestamp))
        {
            deltaTime = 1.0;
            deltaX = deltaY = deltaTheta = 0.0;
        }

        pose_t rayPose(previousPose.x     + (deltaX     * (scanStartTime / deltaTime)),
                              previousPose.y     + (deltaY     * (scanStartTime / deltaTime)),
                              previousPose.theta + (deltaTheta * (scanStartTime / deltaTime)));

        stride = std::max(stride, 1); // ensure the stride actually increments through the scan
        int numSteps = utils::strided_sequence_length(scan.numRanges, stride);

        double xStepRay     = deltaX     * (scanDuration / deltaTime) / numSteps;
        double yStepRay     = deltaY     * (scanDuration / deltaTime) / numSteps;
        double thetaStepRay = deltaTheta * (scanDuration / deltaTime) / numSteps;

        if(thetaStepRay * numSteps > M_PI / 4.0)
        {
            std::cout << "(dX,dY,dT) (sX,sY,sT): (" << deltaX << ',' << deltaY << ',' << deltaTheta << ") ("
                    << xStepRay << ',' << yStepRay << ',' << thetaStepRay << ") Time:" << deltaTime
                    << " Pose time: (" << scanStartTime << ',' << scanDuration << ") Start:" << rayPose << "\n"
                    << "\n\n\n\n\n\n\n\n";
        }

        rays_.clear();
        rays_.reserve(numSteps);

        float cosPhi = std::cos(scan.offset.phi); // pitch
        float cosRho = std::cos(scan.offset.rho); // roll

        Point<float> endpoint;

        for(int n = 0; n < scan.numRanges; n += stride)
        {
            adjusted_ray_t adjusted;
            adjusted.position = homogeneous_transform(scan.offset.to2DPosition(),
                                                            rayPose.x,
                                                            rayPose.y,
                                                            rayPose.theta);

            // only do the full calculation for valid ranges
            if(scan.ranges[n] > 0)
            {
                float globalTheta = rayPose.theta + scan.offset.theta;
                float theta = (scan.angularResolution * n) + scan.startAngle;

                endpoint.x = scan.ranges[n] * std::cos(theta) * cosPhi;
                endpoint.y = scan.ranges[n] * std::sin(theta) * cosRho;

                adjusted.endpoint.x = (std::cos(globalTheta) * endpoint.x) - (std::sin(globalTheta) * endpoint.y) +
                    adjusted.position.x;
                adjusted.endpoint.y = (std::sin(globalTheta) * endpoint.x) + (std::cos(globalTheta) * endpoint.y) +
                    adjusted.position.y;

                adjusted.range = distance_between_points(adjusted.position, adjusted.endpoint);
                adjusted.angle = angle_to_point(adjusted.position, adjusted.endpoint);
//                 adjusted.range = scan.ranges[n];
            }
            else
            {
                adjusted.range = scan.ranges[n];    // need the original range because it indicates the error conditions
            }

            if(!scan.intensities.empty())
            {
                adjusted.intensity = scan.intensities[n];
            }

            rays_.push_back(adjusted);

            rayPose.x     += xStepRay;
            rayPose.y     += yStepRay;
            rayPose.theta += thetaStepRay;
        }
    }

    /**
    * laserId retrieves the id of the laser from which this scan was taken.
    */
    int laserId(void) const { return id_; }

    /**
    * timestamp retrieves the start time of the scan.
    */
    int64_t timestamp(void) const { return timestamp_; }

    // Iterate through the scan
    std::size_t size (void) const { return rays_.size();  }
    RayIter     begin(void) const { return rays_.begin(); }
    RayIter     end  (void) const { return rays_.end();   }

    const adjusted_ray_t& operator[](int n) const { return rays_[n]; }

private:

    int id_;
    int64_t timestamp_;
    std::vector<adjusted_ray_t> rays_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( id_,
            timestamp_,
            rays_
        );
    }
};

} // namespace laser
} // namespace vulcan

DEFINE_DEBUG_MESSAGE(laser::MovingLaserScan, ("DEBUG_MOVING_LASER"))

#endif // LASER_MOVING_LASER_SCAN_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     unicycle_lyapunov_steering.cpp
 * \author   Jong Jin Park
 *
 * Definition of numerically stable steering via lyapunov function.
 */

#include "mpepc/rrt/unicycle_lyapunov_steering.h"
#include "mpepc/math/unicycle_lyapunov_distance.h"

namespace vulcan
{
namespace mpepc
{

UnicycleLyapunovSteering::UnicycleLyapunovSteering(const unicycle_lyapunov_steering_params_t& params) : params_(params)
{
}


unicycle_path_segment_t UnicycleLyapunovSteering::extend(const pose_t& robotPose,
                                                         const UnicycleLyapunovDistance& lyap,
                                                         double maxExtension) const
{
    std::vector<pose_t> steps;
    steps.push_back(robotPose);

    std::vector<Point<float>> path;
    path.push_back(robotPose.toPoint());

    // rotate onto the vector field
    bool isAligned = false;
    while (!isAligned) {
        // rotation onto vector field (isAligned is updated here)
        pose_t rotatedPose = incrementalRotation(steps.back(), lyap, params_.incrementalRotation, &isAligned);
        //         std::cout<< "is rotating at: "<< rotatedPose << "    with increments: " <<
        //         params_.incrementalRotation <<'\n';

        steps.push_back(rotatedPose);
        // path doesn't have to be updated since only rotating-in-place
    }

    // once successfully rotated onto the vector field, move along the vector field
    bool isConverged = false;
    while (!isConverged) {
        // translation along vector field (path and isConverged are updated here)
        pose_t translatedPose =
          incrementalTranslation(steps.back(), lyap, params_.incrementalTranslation, &path, &isConverged);
        //         std::cout<< "is translating at: "<< translatedPose << "    with increments: " <<
        //         params_.incrementalTranslation <<'\n';

        steps.push_back(translatedPose);

        if (maxExtension > 0.0)   // when maxExtension is positive
        {
            UnicycleLyapunovDistance traveled(translatedPose, lyap.params_);
            double distanceTraveled = traveled.distanceFromPose(translatedPose);

            if (distanceTraveled > maxExtension)   // terminate if moved sufficiently far away
            {
                break;
            }
        }
    }

    return unicycle_path_segment_t{steps, path};
}


unicycle_path_segment_t UnicycleLyapunovSteering::steer(const pose_t& robotPose,
                                                        const UnicycleLyapunovDistance& lyap) const
{
    return this->extend(robotPose, lyap, -1);   // extend without length constraint
}


std::vector<Point<float>> UnicycleLyapunovSteering::steerAway(const Point<float>& point,
                                                              const UnicycleLyapunovDistance& lyap,
                                                              double maxExtension) const
{
    // TODO: Implement this!
    std::vector<Point<float>> temp;
    return temp;
}


pose_t UnicycleLyapunovSteering::incrementalRotation(const pose_t& robotPose,
                                                     const UnicycleLyapunovDistance& lyap,
                                                     double maxRotation,
                                                     bool* isAligned) const
{
    pose_t nextPose = robotPose;

    // find reference heading induced by the laypunov function
    double referenceHeading = lyap.stabilizingHeading(robotPose.toPoint());
    double headingError = angle_diff(referenceHeading, robotPose.theta);

    if (fabs(headingError) > maxRotation) {
        *isAligned = false;
        nextPose.theta = nextPose.theta + copysign(maxRotation, headingError);
    } else {
        *isAligned = true;
        nextPose.theta = wrap_to_2pi(referenceHeading);
    }

    return nextPose;
}


pose_t UnicycleLyapunovSteering::incrementalTranslation(const pose_t& robotPose,
                                                        const UnicycleLyapunovDistance& lyap,
                                                        double maxTranslation,
                                                        std::vector<Point<float>>* points,
                                                        bool* isConverged) const
{
    // hard-coded parameters
    const double kIntegrationStepSize = 0.01;
    const double kConvergedRadius = 0.05;
    size_t kMaxNumSteps = maxTranslation / kIntegrationStepSize;
    size_t numSteps = 0;

    // map to reduced egocentric polar coords
    reduced_egocentric_polar_coords_t coords = lyap.chart_.point2rp(robotPose.toPoint());

    while (true) {
        if (points) {
            points->push_back(lyap.chart_.rp2point(coords));   // store trajectory in Cartesian coords
        }

        // termination conditions
        numSteps++;
        *isConverged = distance_on_manifold(coords.r, coords.phi, lyap.params_.kPhi) < kConvergedRadius;
        if (*isConverged || numSteps >= kMaxNumSteps) {
            break;
        }

        // align heading along the vector field
        float deltaStar = stabilizing_delta_star(coords.r,
                                                 coords.phi,
                                                 lyap.params_.kPhi,
                                                 lyap.params_.vectorFieldType,
                                                 lyap.params_.smallRadius);

        // step forward along the heading
        // x-axis crossing and passing-the-origin are numerical errors, so modify step sizes to prevent that.
        double ds = std::min(kIntegrationStepSize, coords.r);   // prevents passing-the-origin
        double dr = -ds * cos(deltaStar);
        double dphi = ds / coords.r * sin(deltaStar);   // compute ds/r first for numerical stability

        if (coords.phi * (coords.phi + dphi) < 0)   // detect r-axis crossing
        {
            dr = fabs(coords.phi / dphi) * dr;   // scale change in r
            dphi = -coords.phi;                  // to snap to x-axis
        }

        // update
        coords.r += dr;
        coords.phi += dphi;
    }

    // final pose
    double deltaAtEndPoint = stabilizing_delta_star(coords.r,
                                                    coords.phi,
                                                    lyap.params_.kPhi,
                                                    lyap.params_.vectorFieldType,
                                                    lyap.params_.smallRadius);
    egocentric_polar_coords_t extendedCoords = {coords.r, coords.phi, deltaAtEndPoint, coords.lineOfSightAngle};
    pose_t endPose = lyap.chart_.rpd2pose(extendedCoords);

    return endPose;
}


//
// pose_t steer_away_along_vector_field(const pose_t& robotPose,
//                                             const pose_t& targetPose,
//                                             double maxTranslation,
//                                             double kPhi,
//                                             stabilizing_vector_field_type_t  vectorFieldType,
//                                             std::vector<Point<float>>* points,
//                                             bool*  isConverged,
//                                             double rangeEpsilon = 0.05)
// {
//     const double kIntegrationStepSize = 0.01;
//     const double kConvergedRadius     = 0.05;
//     size_t kMaxNumSteps = maxTranslation / kIntegrationStepSize;
//     size_t numSteps = 0;
//
//     // map to reduced egocentric polar coords
//     reduced_egocentric_polar_coords_t coords(targetPose, robotPose.toPoint(), rangeEpsilon);
//     while(true)
//     {
//         if(points)
//         {
//             points->push_back(coords.toPoint(targetPose)); // store trajectory in Cartesian coords
//         }
//
//         // align heading along the vector field
//         float deltaStar = stabilizing_delta_star(coords.r, coords.phi, kPhi, vectorFieldType, rangeEpsilon);
//
//         // step *backward* along the heading
//         // x-axis crossing and passing-the-origin are numerical errors, so modify step sizes to prevent that.
//         double ds   =  kIntegrationStepSize;
//         double dr   = -ds * cos(deltaStar);
//         double dphi =  ds / coords.r * sin(deltaStar); // compute ds/r first for numerical stability
//
//         // update (note the subtraction)
//         coords.r   -= dr;
//         coords.phi -= dphi;
//
//         // terminate if reached bounds or the length is too long
//         numSteps++;
//         if(fabs(coords.phi) > M_PI - 0.01 || numSteps >= kMaxNumSteps)
//         {
//             break;
//         }
//     }
//
//     // final pose
//     float deltaAtEndPoint = stabilizing_delta_star(coords.r, coords.phi, kPhi, vectorFieldType, rangeEpsilon);
//     egocentric_polar_coords_t extendedCoords(coords.r, coords.phi, deltaAtEndPoint, coords.observationAngle);
//     pose_t endPose = extendedCoords.toRobotPose(targetPose);
//
//     return endPose;
//
// };
//
//
// struct path_along_vector_field_result_t
// {
//     bool isNotTerminatedByCollision;
//     std::vector<pose_t>  steps;
//     std::vector<Point<T>> path;
//
// };
//
// path_along_vector_field_result_t find_obstacle_free_path_along_vector_field(const pose_t& robotPose,
//                                                                             const pose_t& targetPose,
//                                                                             double maxRotation,
//                                                                             double maxTranslation,
//                                                                             double kPhi,
//                                                                             stabilizing_vector_field_type_t
//                                                                             vectorFieldType, const
//                                                                             RobotCollisionModel&  robotShape, const
//                                                                             ObstacleDistanceGrid& grid, double
//                                                                             smallRadius = 0.05)
// {
//     bool haveFoundPath = false;
//
//     std::vector<Point<float>> path;
//     path.push_back(robotPose.toPoint());
//
//     std::vector<pose_t> steps;
//     steps.push_back(robotPose);
//
//     // rotate onto the vector field
//     bool isAligned = false;
//     while(!isAligned)
//     {
//         // rotation onto vector field (isAligned is updated here)
//         pose_t rotatedPose = rotate_onto_vector_field(robotPose, targetPose, maxRotation, kPhi, vectorFieldType,
//         &isAligned, smallRadius);
//
//         // collision check
//         bool isCollisionFree = robotShape.closestObstacle(rotatedPose, grid) > 0;
//
//         if(!isCollisionFree)
//         {
//             // if in collision report failure
//             haveFoundPath = false;
//             break;
//         }
//         else
//         {
//             // otherwise extend steps
//             steps.push_back(rotatedPose);
//             // path doesn't have to be updated since only rotating-in-place
//         }
//     }
//
//     // once successfully rotated onto the vector field, move along the vector field
//     bool isConverged = false;
//     while(!isConverged)
//     {
//         // translation along vector field (path and isConverged are updated here)
//         pose_t translatedPose = steer_along_vector_field(robotPose, targetPose, maxTranslation, kPhi,
//         vectorFieldType, &path, &isConverged, smallRadius);
//
//         // collision check
//         bool isCollisionFree = robotShape.closestObstacle(translatedPose, grid) > 0;
//
//         if(!isCollisionFree)
//         {
//             // if in collision report failiure
//             haveFoundPath = false;
//             break;
//         }
//         else
//         {
//             // otherwise exted steps
//             steps.push_back(translatedPose);
//         }
//     }
//
//     return haveFoundPath;
// };

}   // namespace mpepc
}   // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     unicycle_chart.h
 * \author   Jong Jin Park
 *
 * Declaration of egocentric polar coordinates and related mapping functions from/to Cartesian coordinates.
 */

#ifndef MPEPC_UNICYCLE_CHART_H
#define MPEPC_UNICYCLE_CHART_H

#include "core/pose.h"
#include "mpepc/math/egocentric_polar_coordinates.h"

namespace vulcan
{
namespace mpepc
{

/**
 * UnicycleChart is a collection of bijective mappings between egocentric polar
 * coordinates and Cartesian cooridnates, anchored around a target pose.
 * This mapping is particularly useful for unicycle-type vehicles converging to
 * a target pose on a plane. See [Park-15] for exact definition.
 */
class UnicycleChart
{
public:
    /**
     * Constructor for unicycle chart.
     *
     * \param    targetPose      Target pose, around which a chart is constructed.
     * \param    smallRadius     Small radius threshold, used to avoid numerical instability at r = 0.
     */
    UnicycleChart(const pose_t& targetPose, double smallRadius);

    // simple getters
    pose_t getTargetPose(void) const { return targetPose_; };
    double getSmallRadius(void) const { return smallRadius_; };

    /**
     * lineOfSight finds a line_of_sight_t to a targetPose. If the range is too
     * small the angle of observation aligns with target orientation.
     *
     * \param    observerPosition        Starting point of the line of sight
     * \return   line of sight (range, angle)
     */
    line_of_sight_t lineOfSight(const Point<float>& observerPosition) const;

    /**
     * point2rp is a mapping from a Cartesian point to egocentric polar coords.
     * (1) phi is zero if the line of sight aligns with the target orientation
     * (2) Be careful when r is very small or phi is close to +- pi!
     *
     * \param    point           Cartesian point
     * \return   point in egocentric polar coordinates(r, phi)
     */
    reduced_egocentric_polar_coords_t point2rp(const Point<float>& point) const;

    /**
     * rp2point is a mapping from a point in egocentric polar coordinates (r,phi)
     * to a point in Cartesian coordinates.
     *
     * \param    coords           point in egocentric polar coordinates
     * or (overloaded)
     * \param    r                radial distance to the target
     * \param    phi              orientaion of the target measured from the line of sight
     *
     * \return   point in Cartesian coordinates
     */
    Point<float> rp2point(double r, double phi) const;
    Point<float> rp2point(const reduced_egocentric_polar_coords_t& coords) const;

    /**
     * pose2rpd is a mapping from a pose in egocentric polar coordinates (r,phi)
     * to a pose in Cartesian coordinates.
     *
     * \param    pose            Cartesian pose
     * \return   pose in egocentc polar coordinates
     */
    egocentric_polar_coords_t pose2rpd(const pose_t& robotPose) const;

    /**
     * rpd2pose is a mapping from a pose in egocentric polar coordinates (r,phi,delta)
     * to a robot pose in Cartesian coordinates.
     *
     * \param    coords           pose in egocentric polar coordinates
     * or (overloaded)
     * \param    r                radial distance to the target
     * \param    phi              orientaion of the target measured from the line of sight
     * \param    delta            orientaion of the robot measured from the line of sight
     *
     * \return   robot pose in Cartesian coordinates
     */
    pose_t rpd2pose(double r, double phi, double delta) const;
    pose_t rpd2pose(const egocentric_polar_coords_t& coords) const;

    /**
     * rpd2target recovers the target pose in Cartesian coordinates forom a pose
     * in egocentric polar coordinates (r,phi,delta) and a robot pose in
     * Cartesian coordinates.
     *
     * \param    coords           pose in egocentric polar coordinates
     * \param    robotPose        pose of a robot in Cartesian coordinates
     * or (overloaded)
     * \param    r                radial distance to the target
     * \param    phi              orientaion of the target measured from the line of sight
     * \param    delta            orientaion of the robot measured from the line of sight
     * \param    robotPose        pose of a robot in Cartesian cooridnates
     *
     * \return   target pose in Cartesian coordinates
     */
    pose_t rpd2target(double r, double phi, double delta, const pose_t& robotPose) const;
    pose_t rpd2target(const egocentric_polar_coords_t& coords, const pose_t& robotPose) const;

private:
    pose_t targetPose_;    // target pose
    double smallRadius_;   // small radius
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_UNICYCLE_CHART_H

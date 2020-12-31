/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     reflected_laser_scan.h
 * \author   Collin Johnson
 *
 * Definition of ReflectedLaserScan.
 */

#ifndef LASER_REFLECTED_LASER_SCAN_H
#define LASER_REFLECTED_LASER_SCAN_H

#include "core/point.h"
#include "system/message_traits.h"
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <cstdint>

namespace vulcan
{
namespace laser
{

/**
 * reflected_ray_t defines a laser ray with information about whether or not it was reflected and at which point it was
 * reflected.
 */
struct reflected_ray_t
{
    bool isReflected;         ///< Flag indicating if the ray was reflected
    float range;              ///< Overall range of the ray
    float distToReflection;   ///< Distance at which the ray hit a specular surface and was reflected
    Point<float> origin;      ///< Origin of the ray -- where it was gathered -- in the global frame
    float angle;              ///< Angle the ray is emitted in the global frame
};

template <class Archive>
void serialize(Archive& ar, reflected_ray_t& ray)
{
    ar(ray.isReflected, ray.range, ray.distToReflection, ray.origin, ray.angle);
}

/**
 * ReflectedLaserScan represents a laser scan with information about probable reflections off specular surfaces in the
 * environment. Each reflected_ray_t contains information on whether or not a ray was likely to have been reflected and
 * if it was reflected, how far along the scan the reflection occurred.
 *
 * The reflected laser scan is meant for use in debugging to see how the reflection processing is working and to allow
 * for rendering this internal state of the mapping algorithm.
 */
class ReflectedLaserScan
{
public:
    using RayIter = std::vector<reflected_ray_t>::const_iterator;

    /**
     * Default constructor for ReflectedLaserScan.
     */
    ReflectedLaserScan(void) : id_(-1), timestamp_(0) { }

    /**
     * Constructor for ReflectedLaserScan.
     *
     * \param    laserId             Id of the laser that took the scan
     * \param    timestamp           Timestamp the scan was taken
     * \param    rays                Rays with stored reflection information
     */
    ReflectedLaserScan(int32_t laserId, int64_t timestamp, const std::vector<reflected_ray_t>& rays)
    : id_(laserId)
    , timestamp_(timestamp)
    , rays_(rays)
    {
    }

    /**
     * laserId retrieves the id of the laser that took this scan.
     */
    int32_t laserId(void) const { return id_; }

    /**
     * timestamp retrieves the time at which the scan was taken.
     */
    int64_t timestamp(void) const { return timestamp_; }

    // Iterate through the scan
    std::size_t size(void) const { return rays_.size(); }
    RayIter begin(void) const { return rays_.begin(); }
    RayIter end(void) const { return rays_.end(); }

    const reflected_ray_t& operator[](int n) const { return rays_[n]; }

private:
    int32_t id_;
    int64_t timestamp_;
    std::vector<reflected_ray_t> rays_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(id_, timestamp_, rays_);
    }
};

}   // namespace laser
}   // namespace vulcan

DEFINE_DEBUG_MESSAGE(laser::ReflectedLaserScan, ("DEBUG_REFLECTED_LASER"))

#endif   // LASER_REFLECTED_LASER_SCAN_H

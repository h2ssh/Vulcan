/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     angle_range.h
 * \author   Collin Johnson
 *
 * Definition of angle_range_t for representing a continuous range of angles as a start and extent.
 */

#ifndef MATH_ANGLE_RANGE_H
#define MATH_ANGLE_RANGE_H

#include "core/angle_functions.h"
#include <iosfwd>

namespace vulcan
{
namespace math
{

/**
 * angle_rangle_t represents a range of angles beginning with start and going in the positive
 * direction, i.e. counterclockwise, for extent radians. The represented region is [start, start+extent].
 */
struct angle_range_t
{
    float start;    ///< Starting angle of the range : [-pi, pi]
    float extent;   ///< Extent of contained angles  : [0, 2pi]

    explicit angle_range_t(float start = 0.0f, float extent = 0.0f) : start(wrap_to_pi(start)), extent(extent)
    {
        if (this->extent < 0.0f) {
            this->extent = 0.0f;
        } else if (this->extent > 2.0 * M_PI) {
            this->extent = 2.0 * M_PI;
        }
    }

    /**
     * overlaps checks if this range overlaps with another range.
     *
     * \param    rhs             Rangle to check for overlap
     * \param    uncertainty     Uncertainty of the range estimates (optional, default = 0)
     * \return   True if some portion of the ranges overlap
     */
    bool overlaps(const angle_range_t& rhs, float uncertainty = 0.0f) const
    {
        double rangeToBInA = wrap_to_2pi(rhs.start - start);
        double rangeToAInB = wrap_to_2pi(start - rhs.start);

        bool bIsInA = (rangeToBInA + uncertainty) < extent;
        bool aIsInB = (rangeToAInB + uncertainty) < rhs.extent;

        return bIsInA || aIsInB;
    }

    /**
     * contains checks if the specific angle is contained within the angle range. The check sees if the distance from
     * the start angle is less than the overall extent.
     *
     * \param    angle       Angle to check
     * \return   True if the angle is in the range.
     */
    bool contains(float angle) const { return wrap_to_2pi(angle - start) < extent; }

    /**
     * expand expands a range by incorporating a new angle. The range is expanded the minimum amount necessary to
     * include the angle. This means if the angle is closer to start, then the start is moved. If the angle is closer
     * to start + extent, then the extent is increased. If the angle is already in the range, then the range is
     * unchanged.
     *
     * \param    angle       Angle to add to the range
     */
    void expand(float angle)
    {
        angle = wrap_to_pi(angle);
        double rangeFromStart = wrap_to_2pi(angle_diff(angle, start));

        // If the range is larger than the extent, then the angle doesn't fit in the current range
        if (rangeFromStart > extent) {
            // Move the start if the angle being added is closer to the start than the end of the range and the angle is
            // to the right of the start angle. The range is always positive, i.e. to the left of the start, so that
            // invariant needs to be maintained If there isn't an extent yet, then move such that the range is positive
            double distToStart = angle_diff(start, angle);
            double distFromEnd = angle_diff_abs(angle, wrap_to_pi(start + extent));

            // If there's no extent and the new angle is to the right of the start, then move the start
            if ((extent == 0.0f) && (distToStart > 0.0f)) {
                start = angle;
                extent = distToStart;
            }
            // Otherwise if there's an extent and the distance to start is less than that to the end, then move the
            // start
            else if (extent > 0.0f && std::abs(distToStart) <= distFromEnd) {
                extent += std::abs(distToStart);
                start = angle;
            }
            // The start remains and just increase the extent to include the new angle
            else {
                extent = rangeFromStart;
            }
        }
    }
};


std::ostream& operator<<(std::ostream& out, const angle_range_t& range);

// Serialization support
template <class Archive>
void serialize(Archive& ar, angle_range_t& range)
{
    ar(range.start, range.extent);
}

}   // namespace math
}   // namespace vulcan

#endif   // MATH_ANGLE_RANGE_H

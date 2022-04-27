/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef CORE_ANGLE_FUNCTIONS_H
#define CORE_ANGLE_FUNCTIONS_H

#include <numbers>
#include <cmath>

namespace vulcan
{

constexpr float PI_F = std::numbers::pi_v<float>;
constexpr float TWO_PI_F = 2.0f * PI_F;
constexpr float PI_2_F = PI_F / 2.0f;
constexpr float PI_4_F = PI_F / 4.0f;

/**
 * wrap_to_pi takes an angle of arbitrary size and reduces it to the range [-PI, PI].
 *
 * \param    angle           Angle to wrap
 * \return   Equivalent angle in range [-PI, PI].
 */
template <typename T>
T wrap_to_pi(T angle)
{
    constexpr auto PI = std::numbers::pi_v<T>;
    constexpr auto TWO_PI = T(2) * PI;

    if (angle < -PI) {
        for (; angle < -PI; angle += TWO_PI)
            ;
    } else if (angle > PI) {
        for (; angle > PI; angle -= TWO_PI)
            ;
    }

    return angle;
}

/**
 * wrap_to_2pi takes an angle of arbitrary sizes and reduces it to the range [0, 2PI].
 *
 * \param    angle           Angle to wrap
 * \return   Equivalent angle in range [0, 2PI].
 */
template <typename T>
T wrap_to_2pi(T angle)
{
    constexpr auto PI = std::numbers::pi_v<T>;
    constexpr auto TWO_PI = T(2) * PI;

    if (angle < 0) {
        for (; angle < 0; angle += TWO_PI)
            ;
    } else if (angle > TWO_PI) {
        for (; angle > TWO_PI; angle -= TWO_PI)
            ;
    }

    return angle;
}

/**
 * wrap_to_pi_2 takes an arbitrary angle and wraps it to the range [-pi/2,pi/2]. This function is intended
 * for use with lines where the direction doesn't matter, e.g. where something like 3pi/4 == -pi/4 like the
 * slope of a line.
 *
 * \param    angle           Angle to wrap
 * \return   Angle in the range [-pi/2,pi/2].
 */
template <typename T>
T wrap_to_pi_2(T angle)
{
    constexpr auto PI = std::numbers::pi_v<T>;
    constexpr auto PI_2 = PI / T(2);

    T wrapped = wrap_to_pi(angle);

    if (wrapped < -PI_2) {
        wrapped += PI;
    } else if (wrapped > PI_2) {
        wrapped -= PI;
    }

    return wrapped;
}

/**
 * angle_diff finds the difference in radians between two angles and ensures that the differences
 * falls in the range of [-PI, PI].
 *
 * \param    leftAngle           Angle on the left-handside of the '-'
 * \param    rightAngle          Angle on the right-handside of the '-'
 * \return   The difference between the angles, leftAngle - rightAngle, in the range [-PI, PI].
 */
template <typename T>
T angle_diff(T leftAngle, T rightAngle)
{
    constexpr auto PI = std::numbers::pi_v<T>;
    constexpr auto TWO_PI = T(2) * PI;

    T diff = leftAngle - rightAngle;
    if (std::abs(diff) > PI) {
        diff -= (diff > 0) ? TWO_PI : -TWO_PI;
    }

    return diff;
}

/**
 * angle_diff_abs finds the absolute value of the difference in radians between two angles and ensures that the
 * differences falls in the range of [0, PI].
 *
 * \param    leftAngle           Angle on the left-handside of the '-'
 * \param    rightAngle          Angle on the right-handside of the '-'
 * \return   The absolute value of the difference between the angles, leftAngle - rightAngle, in the range [0, PI].
 */
template <typename T>
T angle_diff_abs(T leftAngle, T rightAngle)
{
    return std::abs(angle_diff(leftAngle, rightAngle));
}


/**
 * angle_diff_abs_pi_2 finds the difference in radians between two angles where the range is [0, PI/2].
 * The calculation is the angle_diff. If the abs(diff) > M_PI/2, then the result is PI - abs(diff). This
 * function is intended for cases where the angles represent line slopes rather than vectors. The slope
 * only can differ by at most PI/2.
 *
 * \param    lhs         Angle on left of '-'
 * \param    rhs         Angle on right of '-'
 * \return   lhs - rhs, in the range [0, PI/2].
 */
template <typename T>
T angle_diff_abs_pi_2(T lhs, T rhs)
{
    constexpr auto PI = std::numbers::pi_v<T>;
    constexpr auto PI_2 = PI / T(2);

    T diff = std::abs(angle_diff(lhs, rhs));

    return (diff < PI_2) ? diff : PI - diff;
}


/**
 * angle_sum finds the sum in radians of two angles and ensures the value is in the range [-PI, PI].
 *
 * \param    angleA              First angle in the sum
 * \param    angleB              Second angle in the sum
 * \return   The sum of the two angles, angleA + angleB, in the range [-PI, PI].
 */
template <typename T>
T angle_sum(T angleA, T angleB)
{
    constexpr auto PI = std::numbers::pi_v<T>;
    constexpr auto TWO_PI = T(2) * PI;

    T sum = angleA + angleB;

    if (std::abs(sum) > PI) {
        sum -= (sum > 0) ? TWO_PI : -TWO_PI;
    }

    return sum;
}

}   // namespace vulcan

#endif   // CORE_ANGLE_FUNCTIONS_H

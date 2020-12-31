/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gl_camera.cpp
 * \author   Collin Johnson
 *
 * Definition of GLCamera.
 */

#include "ui/common/gl_camera.h"
#include "core/angle_functions.h"
#include <GL/glu.h>
#include <cmath>
#include <iostream>

#define DEBUG_CAMERA_FOCAL_POINT
#define DEBUG_CAMERA_POSITION

namespace vulcan
{
namespace ui
{

inline float rendered_height(float fieldOfView, float cameraDistance)
{
    return 2.0 * tan(fieldOfView / 2.0) * cameraDistance;
}

inline float camera_distance_to_view_height(float fieldOfView, float height)
{
    return 0.5 * height / tan(fieldOfView / 2.0);
}

inline float clamp_pi_over_two(float angle)
{
    if (angle > M_PI / 2.0f) {
        return M_PI / 2.0f;
    } else if (angle < -M_PI / 2.0f) {
        return -M_PI / 2.0f;
    }

    return angle;
}


GLCamera::GLCamera(unsigned int defaultAnimationLength)
: focalPoint(0, 0, 0)
, cameraSpherical(10.0, M_PI / 2, 0.0)
, fieldOfViewRadians(M_PI / 4)
, viewWidth(1.0f)
, viewHeight(1.0f)
, aspectRatio(1.0f)
, pixelWidth(1.0f)
, pixelHeight(1.0f)
, animationLength(1)
{
}


void GLCamera::setupCamera(float width, float height)
{
    auto cameraPosition = cameraSpherical.toCartesian() + focalPoint;

    setViewport(width, height);
    viewHeight = rendered_height(fieldOfViewRadians, cameraSpherical.rho);
    viewWidth = aspectRatio * viewHeight;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fieldOfViewRadians * 180.0 / M_PI, aspectRatio, 0.1f, cameraSpherical.rho + 10.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluLookAt(cameraPosition.x,
              cameraPosition.y,
              cameraPosition.z,
              focalPoint.x,
              focalPoint.y,
              focalPoint.z,
              sin(cameraSpherical.theta),
              cos(cameraSpherical.theta),
              0.0);
}


void GLCamera::setAnimationLength(unsigned int length)
{
    animationLength = length;
}


void GLCamera::pan(int deltaX, int deltaY)
{
    focalPoint.x += (deltaX / pixelWidth) * viewWidth;
    focalPoint.y += (deltaY / pixelHeight) * viewHeight;
}


void GLCamera::zoom(float percent)
{
    if (percent < -1.0f) {
        percent = -1.0f;
    } else if (percent > 1.0f) {
        percent = 1.0f;
    }

    cameraSpherical.rho *= 1.0f + percent;
}


void GLCamera::setDistance(float distance)
{
    if (distance > 0) {
        cameraSpherical.rho = distance;
    } else {
        cameraSpherical.rho = 1.0f;
    }
}


void GLCamera::setViewport(float width, float height)
{
    pixelHeight = (height == 0.0f) ? 1.0f : height;
    pixelWidth = (width == 0.0f) ? 1.0f : width;

    aspectRatio = pixelWidth / pixelHeight;
}


void GLCamera::setViewRegion(float width, float height)
{
    // If the width of the view that would be rendered using the height to specify the camera distance would
    // not show the necessary area, then scale the height by the aspect ratio to ensure the full view region
    // will be displayed on the screen
    if ((aspectRatio == 0.0f) || (width < aspectRatio * height)) {
        cameraSpherical.rho = camera_distance_to_view_height(fieldOfViewRadians, height);
    } else {
        cameraSpherical.rho = camera_distance_to_view_height(fieldOfViewRadians, width / aspectRatio);
    }
}


void GLCamera::tilt(float radians)
{
    cameraSpherical.phi = clamp_pi_over_two(cameraSpherical.phi + radians);
}


void GLCamera::setTilt(float radians)
{
    cameraSpherical.phi = clamp_pi_over_two(wrap_to_pi(radians));
}


void GLCamera::rotate(float radians)
{
    cameraSpherical.theta = angle_sum(cameraSpherical.theta, radians);
}


void GLCamera::setRotation(float radians)
{
    cameraSpherical.theta = wrap_to_pi(radians);
}

}   // namespace ui
}   // namespace vulcan

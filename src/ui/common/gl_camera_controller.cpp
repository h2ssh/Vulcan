/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gl_camera_controller.cpp
 * \author   Collin Johnson
 *
 * Definition of GLCameraController.
 */

#include "ui/common/gl_camera_controller.h"
#include "ui/common/gl_camera.h"
#include <wx/event.h>

namespace vulcan
{
namespace ui
{

GLCameraController::GLCameraController(GLCamera& camera)
: camera(camera)
, regionWidth(1)
, regionHeight(1)
, canPan(true)
, canZoom(true)
, canTilt(true)
, canRotate(true)
{
}


void GLCameraController::setRegionSize(int width, int height)
{
    regionWidth = width;
    regionHeight = height;
}


GLEventStatus GLCameraController::handleLeftMouseDown(const GLMouseEvent& event)
{
    lastEvent = event;

    return GLEventStatus::passthrough;
}


GLEventStatus GLCameraController::handleLeftMouseUp(const GLMouseEvent& event)
{
    lastEvent = event;

    return GLEventStatus::passthrough;
}


GLEventStatus GLCameraController::handleRightMouseDown(const GLMouseEvent& event)
{
    lastEvent = event;

    return GLEventStatus::passthrough;
}


GLEventStatus GLCameraController::handleRightMouseUp(const GLMouseEvent& event)
{
    lastEvent = event;

    return GLEventStatus::passthrough;
}


GLEventStatus GLCameraController::handleMouseMoved(const GLMouseEvent& event)
{
    int deltaX = lastEvent.screenCoords.x - event.screenCoords.x;
    int deltaY = lastEvent.screenCoords.y - event.screenCoords.y;

    if (mouseEventIsPan(event) && canPan) {
        doMousePan(deltaX, deltaY);
    }

    if (mouseEventIsTilt(event) && canTilt) {
        doMouseTilt(deltaY);
    }

    if (mouseEventIsRotate(event) && canRotate) {
        doMouseRotate(deltaX);
    }

    lastEvent = event;

    return GLEventStatus::passthrough;
}


GLEventStatus GLCameraController::handleMouseWheel(const GLMouseEvent& event)
{
    if (mouseEventIsZoom(event) && canZoom) {
        doMouseZoom(event.wheelRotationDirection);
    }

    lastEvent = event;

    return GLEventStatus::passthrough;
}


GLEventStatus GLCameraController::keyPressed(wxKeyEvent& event)
{
    const float kPercentPan = 0.05;
    const int kRotateAmount = 10;
    const int kTiltAmount = 5;

    auto status = GLEventStatus::capture;

    if (keyEventIsZoomIn(event) && canZoom) {
        doMouseZoom(1);
    } else if (keyEventIsZoomOut(event) && canZoom) {
        doMouseZoom(-1);
    } else if (keyEventIsPanUp(event) && canPan) {
        doMousePan(0, regionHeight * kPercentPan);
    } else if (keyEventIsPanDown(event) && canPan) {
        doMousePan(0, regionHeight * -kPercentPan);
    } else if (keyEventIsPanLeft(event) && canPan) {
        doMousePan(regionHeight * -kPercentPan, 0);
    } else if (keyEventIsPanRight(event) && canPan) {
        doMousePan(regionHeight * kPercentPan, 0);
    } else if (keyEventIsRotateLeft(event) && canRotate) {
        doMouseRotate(-kRotateAmount);
    } else if (keyEventIsRotateRight(event) && canRotate) {
        doMouseRotate(kRotateAmount);
    } else if (keyEventIsTiltUp(event) && canTilt) {
        doMouseTilt(kTiltAmount);
    } else if (keyEventIsTiltDown(event) && canTilt) {
        doMouseTilt(-kTiltAmount);
    } else {
        status = GLEventStatus::passthrough;
    }

    return status;
}


void GLCameraController::doMouseZoom(int direction)
{
    const float ZOOM_PERCENT = 0.02;

    if (direction > 0) {
        camera.zoom(-ZOOM_PERCENT);
    } else {
        camera.zoom(ZOOM_PERCENT);
    }
}


void GLCameraController::doMousePan(int deltaX, int deltaY)
{
    camera.pan(deltaX, deltaY);
}


void GLCameraController::doMouseTilt(int delta)
{
    const float TILT_PER_HEIGHT = M_PI;
    float tiltPerPixel = TILT_PER_HEIGHT / regionHeight;

    camera.tilt(delta * tiltPerPixel);
}


void GLCameraController::doMouseRotate(int delta)
{
    const float ROTATE_PER_WIDTH = 2.0f * M_PI;
    float rotatePerPixel = ROTATE_PER_WIDTH / regionWidth;

    camera.rotate(delta * rotatePerPixel);
}

}   // namespace ui
}   // namespace vulcan

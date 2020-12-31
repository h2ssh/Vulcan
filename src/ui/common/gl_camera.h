/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gl_camera.h
* \author   Collin Johnson
*
* Declaration of GLCamera.
*/

#ifndef UI_COMMON_GL_CAMERA_H
#define UI_COMMON_GL_CAMERA_H

#include "core/point.h"
#include "math/spherical_point.h"

namespace vulcan
{
namespace ui
{

/**
* GLCamera is an abstraction of an OpenGL camera. The camera supports normal operations
* like zooming, panning, and following. Each camera operation can be optionally animated
* by specifying the number of frames over which to take an action.
*/
class GLCamera
{
public:

    /**
    * Constructor for GLCamera.
    *
    * \param    defaultAnimationLength          Specifies default number of frames to use for animating motions (optional)
    */
    explicit GLCamera(unsigned int defaultAnimationLength = 1);

    /**
    * setupCamera issues the OpenGL commands for the current camera position.
    *
    * \param    width           Width of the area to be rendered in pixels
    * \param    height          Height of the area to be rendered in pixels
    */
    void setupCamera(float width, float height);

    /**
    * setAnimationLength sets the length of animations to use for the camera
    *
    * 1 frame means the change happens immediately.
    *
    * \param    length          Number of frames to use for each camera motion
    */
    void setAnimationLength(unsigned int length);

    /**
    * setFocalPoint changes the focal point of the camera so the camera will point at the specified
    * position.
    *
    * \param    newFocalPoint           New (x,y,z) position on which the camera should focus
    */
    void setFocalPoint(const math::Point3D<float>& newFocalPoint) { focalPoint = newFocalPoint; }

    /**
    * getFocalPoint retrieves the point at which the camera is currently pointing.
    */
    math::Point3D<float> getFocalPoint(void) const { return focalPoint; }

    /**
    * getRelativePosition retrieves the position of the camera relative to the focal point.
    */
    math::SphericalPoint getRelativePosition(void) const { return cameraSpherical; }

    /**
    * getAbsolutePosition retrieves the absolute position of the camera in the rendered coordinate system.
    */
    math::Point3D<float> getAbsolutePosition(void) const { return cameraSpherical.toCartesian() + focalPoint; }

    /**
    * setPosition sets the position of the camera from where it will view its surroundings.
    *
    * \param    newPosition             New (x,y,z) position of the camera itself
    */
    void setPosition(const math::SphericalPoint& newPosition) { cameraSpherical = newPosition; }

    /**
    * pan pans the camera by the specified number of pixels in both the x- and y-direction. Positive
    * is right and up. Negative is left and down. The coordinates are screen pixels, not the current
    * OpenGL coordinates. Panning moves the camera focal point.
    *
    * \param    deltaX          Change in x
    * \param    deltaY          Change in y
    */
    void pan(int deltaX, int deltaY);

    /**
    * zoom adjusts the zoom by the specified percent, moving the zoom that much closer to
    * either the min or max. A negative % zooms out. A positive % zooms in.
    *
    * \param    percent         Percent to adjust the zoom. Range: [-1.0, 1.0]
    */
    void zoom(float percent);

    /**
    * setDistance sets the distance of the camera from the origin or focal point, depending on the follow
    * mode.
    *
    * \param    distance        Distance from focal point for the camera
    */
    void setDistance(float distance);
    
    /**
    * setViewport sets the width and height of the viewport with which the camera is associated.
    * 
    * \param    width           Width of the viewport in pixels
    * \param    height          Height of the viewport in pixels
    */
    void setViewport(float width, float height);

    /**
    * setViewRegion sets the region on the screen that will be viewable. The region should be in the
    * camera coordinates, not pixels.
    *
    * \param    width           Width of the region to display
    * \param    height          Height of the region to display
    */
    void setViewRegion(float width, float height);

    /**
    * tilt changes the tilt by the specified number of radians. The tilt will not go outside
    * the range [-pi/2, pi/2].
    *
    * \param    radians         Number of radians to shift the tilt
    */
    void tilt(float radians);

    /**
    * setTilt sets the radians for the tilt. 0 radians places the camera on the z-axis pointing toward
    * (0, 0, 0).
    *
    * \param    radians         Tilt angle for the camera
    */
    void setTilt(float radians);

    /**
    * rotate rotates the camera the specified number of radians.
    *
    * \param    radians         Number of radians to rotate
    */
    void rotate(float radians);

    /**
    * setRotation sets the amount of rotation for the camera for the camera in radians from the x-axis.
    *
    * \param    radians         Exact position of the camera
    */
    void setRotation(float radians);

private:

    // The camera is maintained in spherical coordinates relative to either the focal point or the
    // origin of the coordinate system depending on the follow mode.
    math::Point3D<float> focalPoint;
    math::SphericalPoint cameraSpherical;

    float fieldOfViewRadians;
    float viewWidth;
    float viewHeight;
    float aspectRatio;
    float pixelWidth;
    float pixelHeight;

    int animationLength;
    int framesToGo;
    math::SphericalPoint sphericalStep;

};

}
}

#endif // UI_COMMON_GL_CAMERA_H

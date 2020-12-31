/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     pose_target_renderer.h
 * \author   Collin Johnson and Jong Jin Park
 *
 * Declaration of PoseTargetRenderer.
 */

#ifndef UI_COMPONENTS_POSE_TARGET_RENDERER_H
#define UI_COMPONENTS_POSE_TARGET_RENDERER_H

#include "core/point.h"
#include "ui/common/ui_color.h"
#include <memory>

namespace vulcan
{
struct pose_t;

namespace ui
{

class RobotRenderer;

/**
 * PoseTargetRenderer is used to render a pose target for the robot. A pose target a position and an orientation. The
 * position is drawn as a filled in circle. The orientation is drawn as an arrow pointing in the direction of the
 * target.
 *
 * Two methods are provided for rendering pose targets and the functionality is slightly different. One method accepts
 * to positions, the waypoint position and the orientation position. The waypoint position is where the circle will be
 * drawn for the indicating the target. The orientation arrow will extend from waypoint position to orientation
 * position, thus the length of the arrow will not be fixed. The other method simply accepts a pose. In this case, the
 * orientation arrow will be a fixed length. The arrow will not scale as the view is zoomed-in/out, so it can disappear
 * if the scene is drawn from a sufficiently far distance.
 */
class PoseTargetRenderer
{
public:
    /**
     * Constructor for PoseTargetRenderer.
     */
    PoseTargetRenderer(void);

    ~PoseTargetRenderer(void);

    // methods to set colors.
    /**
     * setTatgetColor sets the color to render the target.
     */
    void setTargetColor(const GLColor& targetColor) { this->targetColor_ = targetColor; };

    /**
     * setArrowColor sets the color to render the target.
     */
    void setArrowColor(const GLColor& arrowColor) { this->arrowColor_ = arrowColor; };

    /**
     * setRenderColors sets the color to render the target and the arrow.
     */
    void setRenderColors(const GLColor& targetColor, const GLColor& arrowColor);


    // methods to set size properties.
    /**
     * setTargetSize sets the size, which is used as a radius of a circle, a base length of a triangle, and a base
     * length of a rectable. The length is in meters.
     */
    void setTargetSize(float size) { this->targetSize_ = size; };

    /**
     * setArrowLength sets the length of the orientation arrow to be drawn.
     * The length is in meters.
     */
    void setArrowLength(float length) { this->arrowLength_ = length; };


    // methods to render the target with various shapes
    /**
     * renderTargetCircle renders the specified target as a circle.
     *
     * \param    waypointPosition            Location of the waypoint (meters)
     * \param    orientationPosition         Location at which the orientation arrow should end (meters)
     */
    void renderTargetCircle(const Point<float>& waypointPosition, const Point<float>& orientationPosition) const;

    /**
     * renderTargetCircle renders the specified target as a circle.
     *
     * \param    target          Pose to render. The orientation is a fixed length as specified by
     * setOrientationArrowLength
     */
    void renderTargetCircle(const pose_t& target) const;

    /**
     * renderTargetTriangle renders the specified target as a triangle.
     *
     * \param    target          Pose to render.
     * \param    isFilled        Optional parameter to set filled/unfilled triangle to be drwan. Default is unfilled
     * rectangle.
     */
    void renderTargetTriangle(const pose_t& target, bool isFilled = false) const;

    /**
     * renderTargetRectangle renders the specified target as an unfilled rectangle and an orienation arrow.
     *
     * \param    target          Pose to render.
     */
    void renderTargetRectangle(const pose_t& target) const;

private:
    // shape and color properties
    float targetSize_;
    float arrowLength_;
    GLColor targetColor_;
    GLColor arrowColor_;

    std::unique_ptr<RobotRenderer> robotRenderer_;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_POSE_TARGET_RENDERER_H

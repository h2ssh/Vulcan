/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     laser_scan_renderer.h
 * \author   Collin Johnson and Jong Jin Park
 *
 * Declaration of LaserScanRenderer, which draws a laser scan as either points or lines.
 */

#ifndef UI_COMPONENTS_LASER_SCAN_RENDERER_H
#define UI_COMPONENTS_LASER_SCAN_RENDERER_H

#include "core/point.h"
#include "ui/common/color_interpolator.h"
#include "ui/common/ui_color.h"

namespace vulcan
{

struct pose_t;
struct pose_6dof_t;
struct polar_laser_scan_t;

namespace laser
{
class MovingLaserScan;
}
namespace laser
{
class ReflectedLaserScan;
}
namespace robot
{
struct proximity_warning_indices_t;
}
namespace robot
{
struct velocity_t;
}

namespace ui
{

/**
 * LaserScanRenderer renders a polar_laser_scan_t.
 */
class LaserScanRenderer
{
public:
    LaserScanRenderer(bool renderLines = false);

    void setRenderColors(const GLColor& normal, const GLColor& maxIntensity);
    void setRenderColors(const GLColor& normal,
                         const GLColor& warning,
                         const GLColor& critical,
                         const GLColor& maxIntensity,
                         const GLColor& minScore,
                         const GLColor& mediumScore,
                         const GLColor& maxScore);

    /**
     * renderScan where colors are all set to normal.
     */
    void renderScan(const polar_laser_scan_t& scan, const pose_t& globalRobotPose);

    /**
     * renderScan renders a MovingLaserScan. The scan is assumed to be in the global coordinate frame.
     */
    void renderScan(const laser::MovingLaserScan& scan);

    /**
     * renderScan renders a ReflectedLaserScan. The scan is assumed to be in the global coordinate frame.
     */
    void renderScan(const laser::ReflectedLaserScan& scan);

    /**
     * renderScan where colors are set based on the proximity indices.
     */
    void renderScan(const polar_laser_scan_t& scan,
                    const robot::proximity_warning_indices_t& proximityIndices,
                    const pose_t& globalRobotPose);

    /**
     * renderScan where colors are based on scores, interpolating between min, medium, and max.
     */
    void renderScan(const polar_laser_scan_t& scan, const std::vector<double>& scores, const pose_t& scanPose);

    /**
     * renderScan where laser pose and robot frame and robot pose in reference frame are specified manually.
     */
    void renderScan(const polar_laser_scan_t& scan,
                    const pose_6dof_t& laserPoseInRobotFrame,
                    const pose_t& robotPoseInReferenceFrame);


    void doRenderLines(void);
    void doNotRenderLines(void);

    bool isRenderingLines(void) const { return shouldRenderLines; }

private:
    std::vector<float> scanPoints;
    std::vector<float> scanLines;
    std::vector<float> pointColors;
    std::vector<float> lineColors;

    GLColor normalColor;
    GLColor warningColor;
    GLColor criticalColor;
    GLColor intensityColor;

    std::vector<GLColor> scoreColors;
    bool shouldRenderLines;

    LinearColorInterpolator interpolator;

    void setProximityColors(const robot::proximity_warning_indices_t& proximityIndices);
    void setScoreColors(const std::vector<double>& scores);
    void createScanLinesFromScanPoints(int numScanPoints, const Point<float>& laserPosition);

    void drawScan(const polar_laser_scan_t& scan, const pose_t& pose);
    void drawScan(const polar_laser_scan_t& scan, const pose_6dof_t& laserPose, const pose_t& robotPose);
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_LASER_SCAN_RENDERER_H

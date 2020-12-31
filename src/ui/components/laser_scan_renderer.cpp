/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     laser_scan_renderer.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of LaserScanRenderer.
*/

#include "ui/components/laser_scan_renderer.h"
#include "ui/common/gl_arrays_helpers.h"
#include "core/laser_scan.h"
#include "laser/moving_laser_scan.h"
#include "laser/reflected_laser_scan.h"
#include "math/coordinates.h"
#include "robot/proximity_warning_indices.h"
#include <algorithm>
#include <cmath>

namespace vulcan
{
namespace ui
{

void set_scan_colors(const GLColor& color, std::vector<GLfloat>& scanColors, size_t numColors);
void copy_cartesian_scan_to_scan_points(std::vector<GLfloat>& scanPoints, const cartesian_laser_scan_t& scan);


LaserScanRenderer::LaserScanRenderer(bool renderLines)
: scanPoints(0)
, scanLines(0)
, pointColors(0)
, lineColors(0)
, scoreColors(3)
, shouldRenderLines(renderLines)
{
}


void LaserScanRenderer::setRenderColors(const GLColor& normal, const GLColor& maxIntensity)
{
    normalColor    = normal;
    intensityColor = maxIntensity;

    interpolator.setColors(normalColor, intensityColor);
}


void LaserScanRenderer::setRenderColors(const GLColor& normal,
                                        const GLColor& warning,
                                        const GLColor& critical,
                                        const GLColor& maxIntensity,
                                        const GLColor& minScore,
                                        const GLColor& mediumScore,
                                        const GLColor& maxScore)
{
    normalColor    = normal;
    warningColor   = warning;
    criticalColor  = critical;
    intensityColor = maxIntensity;

    scoreColors[0] = minScore;
    scoreColors[1] = mediumScore;
    scoreColors[2] = maxScore;

    interpolator.setColors(normalColor, intensityColor);
}


void LaserScanRenderer::renderScan(const polar_laser_scan_t& scan, const pose_t& globalRobotPose)
{
    set_scan_colors(normalColor, pointColors, scan.ranges.size());
    drawScan(scan, globalRobotPose);
}


void LaserScanRenderer::renderScan(const laser::MovingLaserScan& scan)
{
    scanPoints.clear();
    scanLines.clear();
    pointColors.clear();
    lineColors.clear();

    for(auto& ray : scan)
    {
        scanPoints.push_back(ray.endpoint.x);
        scanPoints.push_back(ray.endpoint.y);
        scanPoints.push_back(0.0f);

        pointColors.push_back(normalColor.red());
        pointColors.push_back(normalColor.green());
        pointColors.push_back(normalColor.blue());
        pointColors.push_back(normalColor.alpha());

        scanLines.push_back(ray.position.x);
        scanLines.push_back(ray.position.y);
        scanLines.push_back(0.0f); // not using z right now
        scanLines.push_back(ray.endpoint.x);
        scanLines.push_back(ray.endpoint.y);
        scanLines.push_back(0.0f); // not using z right now

        lineColors.push_back(normalColor.red());
        lineColors.push_back(normalColor.green());
        lineColors.push_back(normalColor.blue());
        lineColors.push_back(normalColor.alpha());

        lineColors.push_back(normalColor.red());
        lineColors.push_back(normalColor.green());
        lineColors.push_back(normalColor.blue());
        lineColors.push_back(normalColor.alpha());
    }

    set_scan_colors(normalColor, pointColors, scan.size());

    glPointSize(2.0f);
    draw_gl_array(scanPoints.data(), 3, pointColors.data(), scan.size(), GL_POINTS);

    if(shouldRenderLines)
    {
        glLineWidth(1.0f);
        draw_gl_array(scanLines.data(), 3, lineColors.data(), scan.size()*2, GL_LINES);
    }
}


void LaserScanRenderer::renderScan(const laser::ReflectedLaserScan& scan)
{
    scanPoints.clear();
    scanLines.clear();

    pointColors.clear();
    lineColors.clear();     // each ray will need a different setup for the free vs. reflected part of the ray
    GLColor reflectedColor(0.9, 0.0, 0.0, 0.25);

    for(auto& ray : scan)
    {
        // Endpoint of the part of the scan that goes through free space
        double freeEndX = ray.origin.x + ray.distToReflection * std::cos(ray.angle);
        double freeEndY = ray.origin.y + ray.distToReflection * std::sin(ray.angle);

        scanPoints.push_back(freeEndX);
        scanPoints.push_back(freeEndY);
        scanPoints.push_back(0.0f);

        pointColors.push_back(normalColor.red());
        pointColors.push_back(normalColor.green());
        pointColors.push_back(normalColor.blue());
        pointColors.push_back(normalColor.alpha());

        // The part of the scan line that goes through free space
        scanLines.push_back(ray.origin.x);
        scanLines.push_back(ray.origin.y);
        scanLines.push_back(0.0f); // not using z right now
        scanLines.push_back(freeEndX);
        scanLines.push_back(freeEndY);
        scanLines.push_back(0.0f); // not using z right now

        lineColors.push_back(normalColor.red());
        lineColors.push_back(normalColor.green());
        lineColors.push_back(normalColor.blue());
        lineColors.push_back(normalColor.alpha());

        lineColors.push_back(normalColor.red());
        lineColors.push_back(normalColor.green());
        lineColors.push_back(normalColor.blue());
        lineColors.push_back(normalColor.alpha());

        // The part of the scan line that occurs after the reflection
        scanLines.push_back(freeEndX);
        scanLines.push_back(freeEndY);
        scanLines.push_back(0.0f); // not using z right now
        scanLines.push_back(ray.origin.x + ray.range * std::cos(ray.angle));
        scanLines.push_back(ray.origin.y + ray.range * std::sin(ray.angle));
        scanLines.push_back(0.0f);

        lineColors.push_back(reflectedColor.red());
        lineColors.push_back(reflectedColor.green());
        lineColors.push_back(reflectedColor.blue());
        lineColors.push_back(reflectedColor.alpha());

        lineColors.push_back(reflectedColor.red());
        lineColors.push_back(reflectedColor.green());
        lineColors.push_back(reflectedColor.blue());
        lineColors.push_back(reflectedColor.alpha());
    }

    glPointSize(2.0f);
    draw_gl_array(scanPoints.data(), 3, pointColors.data(), scan.size(), GL_POINTS);

    if(shouldRenderLines)
    {
        glLineWidth(1.0f);
        draw_gl_array(scanLines.data(), 3, lineColors.data(), scan.size()*4, GL_LINES);
    }
}


void LaserScanRenderer::renderScan(const polar_laser_scan_t&          scan,
                                   const robot::proximity_warning_indices_t& proximityIndices,
                                   const pose_t&                      globalRobotPose)
{
    set_scan_colors(normalColor, pointColors, scan.ranges.size());
    setProximityColors(proximityIndices);  // set the proximity colors AFTER the intensity colors so they aren't overwritten
    drawScan(scan, globalRobotPose);
}


void LaserScanRenderer::renderScan(const polar_laser_scan_t& scan,
                                   const std::vector<double>&       scores,
                                   const pose_t&             scanPose)
{
    set_scan_colors(normalColor, pointColors, scan.ranges.size());
    setScoreColors(scores);
    drawScan(scan, scanPose);
}


void LaserScanRenderer::renderScan(const polar_laser_scan_t& scan,
                                   const pose_6dof_t& laserPoseInRobotFrame,
                                   const pose_t& robotPoseInReferenceFrame)
{
    set_scan_colors(normalColor, pointColors, scan.ranges.size());
    drawScan(scan, laserPoseInRobotFrame, robotPoseInReferenceFrame);
}


void LaserScanRenderer::doRenderLines(void)
{
    shouldRenderLines = true;
}


void LaserScanRenderer::doNotRenderLines(void)
{
    shouldRenderLines = false;
}


void LaserScanRenderer::setProximityColors(const robot::proximity_warning_indices_t& proximityIndices)
{
    for(auto warningIt = proximityIndices.warningIndices.begin(); warningIt != proximityIndices.warningIndices.end(); ++warningIt)
    {
        int index = *warningIt;

        pointColors[4*index]     = warningColor.red();
        pointColors[4*index + 1] = warningColor.green();
        pointColors[4*index + 2] = warningColor.blue();
        pointColors[4*index + 3] = warningColor.alpha();
    }

    for(auto criticalIt = proximityIndices.criticalIndices.begin(); criticalIt != proximityIndices.criticalIndices.end(); ++criticalIt)
    {
        int index = *criticalIt;

        pointColors[4*index]     = criticalColor.red();
        pointColors[4*index + 1] = criticalColor.green();
        pointColors[4*index + 2] = criticalColor.blue();
        pointColors[4*index + 3] = criticalColor.alpha();
    }
}


void LaserScanRenderer::setScoreColors(const std::vector<double>& scores)
{
    interpolator.setColors(scoreColors);

    GLColor scoreColor;

    for(std::size_t n = 0; n < scores.size(); ++n)
    {
        scoreColor = interpolator.calculateColor(scores[n]);

        pointColors[4*n]     = scoreColor.red();
        pointColors[4*n + 1] = scoreColor.green();
        pointColors[4*n + 2] = scoreColor.blue();
        pointColors[4*n + 3] = scoreColor.alpha();
    }
}


void LaserScanRenderer::createScanLinesFromScanPoints(int numScanPoints, const Point<float>& laserPosition)
{
    scanLines.resize(numScanPoints * 6);
    lineColors.resize(numScanPoints * 8);
    for(int n = 0; n < numScanPoints; ++n)
    {
        scanLines[6*n]     = laserPosition.x;
        scanLines[6*n + 1] = laserPosition.y;
        scanLines[6*n + 2] = 0;

        scanLines[6*n + 3] = scanPoints[3*n];
        scanLines[6*n + 4] = scanPoints[3*n+1];
        scanLines[6*n + 5] = scanPoints[3*n+2];

        lineColors[8*n]     = pointColors[4*n];
        lineColors[8*n + 1] = pointColors[4*n + 1];
        lineColors[8*n + 2] = pointColors[4*n + 2];
        lineColors[8*n + 3] = pointColors[4*n + 3];

        lineColors[8*n + 4] = pointColors[4*n];
        lineColors[8*n + 5] = pointColors[4*n + 1];
        lineColors[8*n + 6] = pointColors[4*n + 2];
        lineColors[8*n + 7] = pointColors[4*n + 3];
    }
}


void LaserScanRenderer::drawScan(const polar_laser_scan_t& scan, const pose_t& pose)
{
    // Some scan points might have bad values and they are filtered out here so that they
    // do not get drawn
    cartesian_laser_scan_t cartesianScan;
    polar_scan_to_cartesian_scan_in_robot_frame(scan, cartesianScan, true);
    copy_cartesian_scan_to_scan_points(scanPoints, cartesianScan);

    glPushMatrix();

    glTranslatef(pose.x, pose.y, 0.0);
    glRotatef(pose.theta*180.0f/M_PI, 0.0, 0.0, 1.0);

    glPointSize(2.0f);
    draw_gl_array(scanPoints.data(), 3, pointColors.data(), scan.ranges.size(), GL_POINTS);

    if(shouldRenderLines)
    {
        glLineWidth(1.0f);
        createScanLinesFromScanPoints(scan.ranges.size(), scan.offset.to2DPosition());
        draw_gl_array(scanLines.data(), 3, lineColors.data(), scan.ranges.size()*2, GL_LINES);
    }

    glPopMatrix();
}


void LaserScanRenderer::drawScan(const polar_laser_scan_t& scan,
                                 const pose_6dof_t& laserPoseInRobotFrame,
                                 const pose_t& robotPoseInReferenceFrame)
{
    // Some scan points might have bad values and they are filtered out here so that they
    // do not get drawn
    cartesian_laser_scan_t cartesianScan;
    polar_scan_to_cartesian_scan_in_robot_frame(scan, laserPoseInRobotFrame, cartesianScan, true);
    copy_cartesian_scan_to_scan_points(scanPoints, cartesianScan);

    glPushMatrix();

    glTranslatef(robotPoseInReferenceFrame.x, robotPoseInReferenceFrame.y, 0.0);
    glRotatef(robotPoseInReferenceFrame.theta*180.0f/M_PI, 0.0, 0.0, 1.0);

    glPointSize(2.0f);
    draw_gl_array(scanPoints.data(), 3, pointColors.data(), scan.ranges.size(), GL_POINTS);

    if(shouldRenderLines)
    {
        glLineWidth(1.0f);
        createScanLinesFromScanPoints(scan.ranges.size(), laserPoseInRobotFrame.to2DPosition());
        draw_gl_array(scanLines.data(), 3, lineColors.data(), scan.ranges.size()*2, GL_LINES);
    }

    glPopMatrix();
}


void set_scan_colors(const GLColor& color, std::vector<GLfloat>& scanColors, size_t numColors)
{
    scanColors.resize(numColors * 4);
    for(size_t i = 0; i < numColors*4; i += 4)
    {
        scanColors[i]   = color.red();
        scanColors[i+1] = color.green();
        scanColors[i+2] = color.blue();
        scanColors[i+3] = color.alpha();
    }
}


void copy_cartesian_scan_to_scan_points(std::vector<GLfloat>& scanPoints, const cartesian_laser_scan_t& scan)
{
    scanPoints.resize(scan.scanPoints.size() * 3);
    for(std::size_t i = 0, j = 0; i < scan.scanPoints.size(); ++i, j += 3)
    {
        scanPoints[j]   = scan.scanPoints[i].x;
        scanPoints[j+1] = scan.scanPoints[i].y;
        scanPoints[j+2] = 0;  // not using z at the moment;
    }
}

} // namespace ui
} // namespace vulcan

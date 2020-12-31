/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_metric_display_widget.cpp
* \author   Collin Johnson
*
* Definition of LocalMetricDisplayWidget.
*/

#include "ui/debug/local_metric_display_widget.h"
#include "utils/auto_mutex.h"
#include "ui/common/gl_utilities.h"
#include "ui/common/gl_shapes.h"
#include "ui/components/occupancy_grid_renderer.h"
#include "ui/components/glass_map_renderer.h"
#include "ui/components/laser_scan_renderer.h"
#include "ui/components/lines_renderer.h"
#include "ui/components/particles_renderer.h"
#include "ui/components/pose_trace_renderer.h"
#include "ui/components/robot_renderer.h"
#include "core/odometry.h"
#include <boost/range/iterator_range.hpp>
#include <GL/gl.h>
#include <GL/glu.h>
#include <cassert>

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(LocalMetricDisplayWidget, GridBasedDisplayWidget)
    EVT_MOTION(LocalMetricDisplayWidget::handleMouseMoved)
    EVT_LEFT_DOWN(LocalMetricDisplayWidget::handleMouseDown)
    EVT_LEFT_UP(LocalMetricDisplayWidget::handleMouseUp)
    EVT_RIGHT_DOWN(LocalMetricDisplayWidget::handleMouseDown)
    EVT_RIGHT_UP(LocalMetricDisplayWidget::handleMouseUp)
END_EVENT_TABLE()


LocalMetricDisplayWidget::LocalMetricDisplayWidget(wxWindow* parent,
                                                   wxWindowID id,
                                                   const wxPoint& pos,
                                                   const wxSize& size,
                                                   long style,
                                                   const wxString& name,
                                                   const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, gridRenderer(new OccupancyGridRenderer)
, glassRenderer(new GlassMapRenderer)
, laserRenderer(new LaserScanRenderer)
, extractedLinesRenderer(new LinesRenderer)
, poseTraceRenderer(new PoseTraceRenderer)
, motionTraceRenderer(new PoseTraceRenderer)
, particlesRenderer(new ParticlesRenderer)
, robotRenderer(new RobotRenderer)
, laserData(kNumLasers)
, mostRecentLaserTime(0)
, pose(0, 0, 0)
, maxTraceLength(0)
, mapToShow(LocalMetricMapType::LPM)
, laserToShow_(LaserToShowType::raw)
, shouldCenterOnRobot(true)
, shouldShowExtractedLines(true)
, shouldShowIntensityPlots(false)
, shouldShowPoseTrace(true)
, shouldShowMotionTrace(false)
, shouldShowUncertaintyEllipse(true)
, shouldShowParticles(true)
, shouldShowGlassWalls(false)
, shouldShowGlassAngles(false)
, haveScan(false)
, havePose(false)
, haveGlass(false)
, haveNewGrid(false)
, haveNewGlass(false)
, particleMode(NOT_IN_VIEW_MODE)
{
    // Can't create the trace renderer until the params are specified
    disableScrolling();
    setCameraFocalPoint(Point<float>(0.0f, 0.0f));
    setGridDimensions(20.0f, 20.0f);
}


LocalMetricDisplayWidget::~LocalMetricDisplayWidget(void)
{
}


void LocalMetricDisplayWidget::setWidgetParams(const lpm_display_params_t& params)
{
    maxTraceLength = params.maxTraceLength;

    laserRenderer->setRenderColors(params.frontLaserColor,
                                   params.warningLaserColor,
                                   params.criticalLaserColor,
                                   params.intensityLaserColor,
                                   params.frontLaserColor,
                                   params.warningLaserColor,
                                   params.criticalLaserColor);
    extractedLinesRenderer->setRenderColor(params.extractedLinesColor);
    robotRenderer->setRobotColor(params.robotColor);
    poseTraceRenderer->setRenderColor(params.traceColor);
    motionTraceRenderer->setRenderColor(params.odometryTraceColor);
    particlesRenderer->setRenderColor(params.particlesColor);

    laserData[kFrontLaserId].color = params.frontLaserColor;
    laserData[kBackLaserId].color  = params.backLaserColor;

    this->params = params;
}


void LocalMetricDisplayWidget::centerOnRobot(bool center)
{
    shouldCenterOnRobot = center;

    if(shouldCenterOnRobot)
    {
        disableScrolling();
    }
    else
    {
        enableScrolling();
    }
}


void LocalMetricDisplayWidget::showLaserLines(bool show)
{
    if(show)
    {
        laserRenderer->doRenderLines();
    }
    else
    {
        laserRenderer->doNotRenderLines();
    }
}


void LocalMetricDisplayWidget::showExtractedLines(bool show)
{
    shouldShowExtractedLines = show;
}


void LocalMetricDisplayWidget::showPoseTrace(bool show)
{
    shouldShowPoseTrace = show;
}


void LocalMetricDisplayWidget::showMotionTrace(bool show)
{
    shouldShowMotionTrace = show;
}


void LocalMetricDisplayWidget::showUncertaintyEllipse(bool show)
{
    shouldShowUncertaintyEllipse = show;
}


void LocalMetricDisplayWidget::showParticles(bool show)
{
    shouldShowParticles = show;
}


void LocalMetricDisplayWidget::clearPoseTrace(void)
{
    utils::AutoMutex lock(dataLock);

    poseTrace.clear();
    poseTraceRenderer->setPoseTrace(poseTrace);
}


void LocalMetricDisplayWidget::clearMotionTrace(void)
{
    utils::AutoMutex lock(dataLock);

    motionStateTrace.clear();
    motionTraceRenderer->setPoseTrace(motionStateTrace);
}


void LocalMetricDisplayWidget::setGlassMap(const hssh::GlassMap& glass)
{
    utils::AutoMutex lock(dataLock);

    this->glass = glass;
    haveNewGlass = true;
    haveGlass = true;
}


void LocalMetricDisplayWidget::showGlassIntensity(bool show)
{
    utils::AutoMutex lock(dataLock);
    glassRenderer->showIntensity(show);
}


void LocalMetricDisplayWidget::showGlassWalls(bool show)
{
    utils::AutoMutex lock(dataLock);

    shouldShowGlassWalls = show;

    if(haveGlass)
    {
        glassWalls = hssh::predict_glass_walls(glass);
    }
}


void LocalMetricDisplayWidget::showGlassAngles(bool show)
{
    glassRenderer->showAngles(show);

    // If wasn't showing the angles and now am, then reload the glass map.
    if(!shouldShowGlassAngles && show)
    {
        haveNewGlass = true;
    }

    shouldShowGlassAngles = show;
}


void LocalMetricDisplayWidget::setAnglesToShow(GlassAnglesToDraw angles)
{
    glassRenderer->setAnglesToDraw(angles);
}


void LocalMetricDisplayWidget::flattenGlassMap(int flattenThreshold)
{
    if(haveGlass)
    {
        glass.flattenFullMap(flattenThreshold);
        haveNewGlass = true;
    }
}


void LocalMetricDisplayWidget::filterDynamicObjectsFromGlass(int highlyVisibleThreshold)
{
    if(haveGlass)
    {
        glass.filterDynamicObjectsFromFullMap(highlyVisibleThreshold);
        haveNewGlass = true;
    }
}


void LocalMetricDisplayWidget::rotateLPM(float radians)
{
    utils::AutoMutex lock(dataLock);

    grid.changeReferenceFrame(pose_t(0.0f, 0.0f, radians));
    haveNewGrid = true;
    setGridDimensions(grid.getWidthInMeters(), grid.getHeightInMeters());
}


Point<int> LocalMetricDisplayWidget::convertWorldToGrid(const Point<float>& world) const
{
    if(mapToShow == LocalMetricMapType::GLASS)
    {
        return utils::global_point_to_grid_cell(world, glass);
    }
    else
    {
        return utils::global_point_to_grid_cell(world, grid);
    }
}


void LocalMetricDisplayWidget::handleData(const polar_laser_scan_t& scan, const std::string& channel)
{
    utils::AutoMutex lock(dataLock);

    haveScan            = true;
    mostRecentLaserTime = scan.timestamp;

    if(scan.laserId >= 0 && scan.laserId < kNumLasers)
    {
        laserData[scan.laserId].rawScan = scan;
    }

    // Only have intensity if something is greater than 0.
    bool hasIntensity = !scan.intensities.empty()
        && (*std::max_element(scan.intensities.begin(), scan.intensities.end()) > 0);

    if(hasIntensity && shouldShowIntensityPlots)
    {
        auto& plot = laserData[scan.laserId].plot;

        plot << "set yrange [0:16384]\n";
        plot << "plot '-' with lines title 'Laser " << scan.laserId << "'\n";
        plot.send1d(scan.intensities);
    }
}


void LocalMetricDisplayWidget::handleData(const laser::MovingLaserScan& scan, const std::string& channel)
{
    utils::AutoMutex lock(dataLock);

    if(scan.laserId() >= 0 && scan.laserId() < kNumLasers)
    {
        laserData[scan.laserId()].mappingScan = scan;
    }
}


void LocalMetricDisplayWidget::handleData(const laser::ReflectedLaserScan& scan, const std::string& channel)
{
    utils::AutoMutex lock(dataLock);

    if(scan.laserId() >= 0 && scan.laserId() < kNumLasers)
    {
        laserData[scan.laserId()].reflectedScan = scan;
    }
}


void LocalMetricDisplayWidget::handleData(const laser::laser_scan_lines_t& scanLines, const std::string& channel)
{
    utils::AutoMutex lock(dataLock);

    if(scanLines.scan.laserId >= 0 && scanLines.scan.laserId < kNumLasers)
    {
        laserData[scanLines.scan.laserId].scanLines = scanLines;
    }
}


void LocalMetricDisplayWidget::handleData(const robot::proximity_warning_indices_t& proximityIndices, const std::string& channel)
{
    utils::AutoMutex lock(dataLock);

    if((proximityIndices.laserId >= 0) && (proximityIndices.laserId < kNumLasers))
    {
        laserData[proximityIndices.laserId].proximity = proximityIndices;
    }
}


void LocalMetricDisplayWidget::handleData(const hssh::LocalPerceptualMap& grid, const std::string& channel)
{
    utils::AutoMutex lock(dataLock);

    this->grid  = grid;
    haveNewGrid = true;

    setGridDimensions(grid.getWidthInMeters(), grid.getHeightInMeters());
}


void LocalMetricDisplayWidget::handleData(const hssh::GlassMap& glass, const std::string& channel)
{
    utils::AutoMutex lock(dataLock);

    this->glass = glass;
    haveNewGlass = true;
    haveGlass    = true;
}


void LocalMetricDisplayWidget::handleData(const pose_t& pose, const std::string& channel)
{
    utils::AutoMutex lock(dataLock);

    if(updateTrace(pose, poseTrace))
    {
        poseTraceRenderer->setPoseTrace(poseTrace);
    }

    this->pose = pose;
    havePose   = true;
}


void LocalMetricDisplayWidget::handleData(const pose_distribution_t& distribution, const std::string& channel)
{
    utils::AutoMutex lock(dataLock);

    poseDistribution = distribution;
}


void LocalMetricDisplayWidget::handleData(const motion_state_t& motion, const std::string& channel)
{
    utils::AutoMutex lock(dataLock);

    motionState                = motion;
    stateEstimatorDistribution = motion.poseDistribution;
    motionStateTrace.push_back(motion.pose);

    if(updateTrace(pose_t(motionState.pose.x, motionState.pose.y, motionState.pose.theta), motionStateTrace))
    {
        motionTraceRenderer->setPoseTrace(motionStateTrace);
    }

    if(!havePose)
    {
        pose.x     = motion.pose.x;
        pose.y     = motion.pose.y;
        pose.theta = motion.pose.theta;

        if(shouldCenterOnRobot)
        {
            setCameraFocalPoint(Point<float>(pose.x, pose.y));
        }
    }
}


void LocalMetricDisplayWidget::handleData(const hssh::local_metric_localization_debug_info_t& particles, const std::string& channel)
{
    utils::AutoMutex lock(dataLock);

    this->particles = particles.particleFilterInfo;
}


void LocalMetricDisplayWidget::renderWidget(void)
{
    utils::AutoMutex lock(dataLock);

    if(shouldCenterOnRobot)
    {
        setCameraFocalPoint(Point<float>(pose.x, pose.y));
    }

    // Order matters here! It was chosen based on visual appeal.
    if(haveNewGrid)
    {
        gridRenderer->setGrid(grid);
        haveNewGrid = false;
    }

    if(haveNewGlass)
    {
        glassRenderer->setGlassMap(glass);
        haveNewGlass = false;
    }

    switch(mapToShow)
    {
    case LocalMetricMapType::LPM:
        gridRenderer->renderGrid();
        break;

    case LocalMetricMapType::GLASS:
        {
            glassRenderer->renderGrid();
            glassRenderer->renderAngleBins(glassCell_.x, glassCell_.y, glass);

            if(shouldShowGlassWalls)
            {
                glassRenderer->renderWalls(glassWalls);
            }
        }
        break;

    case LocalMetricMapType::NONE:
        break;
    }

    robotRenderer->renderRobot(pose);

    if(shouldShowUncertaintyEllipse)
    {
        gl_draw_gaussian_distribution(poseDistribution.uncertainty, 3.0f, params.traceColor, 2.0f);
    }

    if(haveScan && (laserToShow_ != LaserToShowType::none))
    {
        for(auto& data : laserData)
        {
            // Only draw the laser scan if it was received recently. Otherwise, it is stale and should just disappear.
            if(std::abs(mostRecentLaserTime - data.rawScan.timestamp) > 500000)
            {
                continue;
            }

            laserRenderer->setRenderColors(data.color, params.intensityLaserColor);

            switch(laserToShow_)
            {
                case LaserToShowType::raw:
                    laserRenderer->renderScan(data.rawScan, data.proximity, pose);
                    break;
                case LaserToShowType::mapping:
                    laserRenderer->renderScan(data.mappingScan);
                    break;
                case LaserToShowType::reflected:
                    laserRenderer->renderScan(data.reflectedScan);
                    break;
                case LaserToShowType::none:
                    break;
                default:
                    assert(false);
            }
        }
    }

    if(shouldShowExtractedLines)
    {
        for(auto& data : laserData)
        {
            extractedLinesRenderer->setRenderColor(data.color);
            extractedLinesRenderer->renderLines(data.scanLines.lines, pose, 4.0f);
        }
    }

    if(shouldShowPoseTrace)
    {
        poseTraceRenderer->renderTrace();
    }

    if(shouldShowMotionTrace)
    {
        motionTraceRenderer->renderTrace();

        if(shouldShowUncertaintyEllipse)
        {
            gl_draw_gaussian_distribution(stateEstimatorDistribution.uncertainty, 3.0f, params.odometryTraceColor, 2.0f);
        }
    }

    if(shouldShowParticles)
    {
        if((particleMode != NOT_IN_VIEW_MODE) && (selectedParticleId < particles.particleScores.size()))
        {
            double maxScore = 1e-5;
            for(auto& score : particles.particleScores)
            {
                maxScore = std::max(maxScore, *std::max_element(score.scores.begin(), score.scores.end()));
            }

            particlesRenderer->renderParticleScores(particles.particles[selectedParticleId].pose,
                                                    particles.particleScores[selectedParticleId],
                                                    maxScore);
            gl_draw_gaussian_distribution(particles.proposalDistribution, 3.0f, GLColor(0.0f, 0.9f, 0.25f, 0.75f), 2.0f);
        }

        particlesRenderer->renderParticles(particles.particles);
    }
}


std::string LocalMetricDisplayWidget::printCellInformation(Point<int> cell)
{
    std::ostringstream infoOut;

    // In Glass mode, the cell is in GlassMap coordinates
    if(haveGlass && (mapToShow == GLASS))
    {
        if(glass.isCellInGrid(cell))
        {
            int hitCount = 0;
            int missCount = 0;
            for(int val : boost::make_iterator_range(glass.beginBin(cell.x, cell.y),
                                                     glass.endBin(cell.x, cell.y)))
            {
                if(val > 0)
                {
                    ++hitCount;
                }
                else if(val < 0)
                {
                    ++missCount;
                }
            }

            infoOut << " Glass (H,M): (" << hitCount << ',' << missCount << ')';
        }

        const auto& intensity = glass.intensityMap();
        auto intensityCell = cell + glass.glassToFlatMapOffset();

        if(intensity.isCellInGrid(intensityCell))
        {
            infoOut << " Intensity: " << intensity(intensityCell.x, intensityCell.y);
        }
    }

    return infoOut.str();
}



void LocalMetricDisplayWidget::handleMouseDown(wxMouseEvent& event)
{
    particleMode = event.ControlDown() ? particleMode : NOT_IN_VIEW_MODE;

    if(event.ControlDown())
    {
        handleParticleMouseEvent(event);
    }

    event.Skip();
}


void LocalMetricDisplayWidget::handleMouseUp(wxMouseEvent& event)
{
    particleMode = event.ControlDown() ? particleMode : NOT_IN_VIEW_MODE;

    if(event.ControlDown())
    {
        handleParticleMouseEvent(event);
    }

    event.Skip();
}


void LocalMetricDisplayWidget::handleMouseMoved(wxMouseEvent& event)
{
    particleMode = event.ControlDown() ? particleMode : NOT_IN_VIEW_MODE;

    if(event.ControlDown())
    {
        handleParticleMouseEvent(event);

        if(haveGlass)
        {
            glassCell_ = utils::global_point_to_grid_cell(mousePosition(), glass);
        }
    }

    event.Skip();
}


void LocalMetricDisplayWidget::handleParticleMouseEvent(wxMouseEvent& event)
{
    wxSize size = GetSize();

    Point<int>   screenPosition(event.GetX(), size.GetHeight()-event.GetY());
    Point<float> worldPosition = convert_screen_to_world_coordinates(screenPosition, getCameraPosition());

    int closestParticle = findClosestParticle(worldPosition);

    if(event.LeftDown())
    {
        selectedParticleId = closestParticle;
        particleMode       = SELECTED_PARTICLE;
    }
    else if(event.LeftUp() || (particleMode != SELECTED_PARTICLE))
    {
        selectedParticleId = closestParticle;
        particleMode       = HOVERING;
    }
}


int LocalMetricDisplayWidget::findClosestParticle(const Point<float>& worldPosition)
{
    if(particles.particles.empty())
    {
        return 0;
    }

    float closest      = distance_between_points(worldPosition, particles.particles[0].pose.toPoint());
    int   closestIndex = 0;

    for(size_t n = 1; n < particles.particles.size(); ++n)
    {
        float distance = distance_between_points(worldPosition, particles.particles[n].pose.toPoint());

        if(distance < closest)
        {
            closest      = distance;
            closestIndex = n;
        }
    }

    return closestIndex;
}


bool LocalMetricDisplayWidget::updateTrace(const pose_t& pose, std::deque<pose_t>& trace)
{
    bool didUpdateTrace = false;
    pose_t lastTracePose(0, 0, 0);

    if(trace.size() > 0)
    {
        lastTracePose = trace[0];
    }

    if(fabs(lastTracePose.x - pose.x) > 0.05 || fabs(lastTracePose.y - pose.y) > 0.05)
    {
        trace.push_front(pose);

        if(trace.size() > maxTraceLength)
        {
            trace.pop_back();
        }

        didUpdateTrace = true;
    }

    return didUpdateTrace;
}

} // namespace ui
} // namespace vulcan

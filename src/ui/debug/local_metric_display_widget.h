/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_metric_display_widget.h
* \author   Collin Johnson
*
* Definition of LocalMetricDisplayWidget for rendering LPM state.
*/

#ifndef UI_DEBUG_LPM_DISPLAY_WIDGET_H
#define UI_DEBUG_LPM_DISPLAY_WIDGET_H

#include <core/point.h>
#include <core/pose_distribution.h>
#include <core/motion_state.h>
#include <utils/mutex.h>
#include <robot/proximity_warning_indices.h>
#include <hssh/local_metric/lpm.h>
#include <hssh/local_metric/debug_info.h>
#include <hssh/metrical/glass_map.h>
#include <hssh/metrical/mapping/glass_walls.h>
#include <mpepc/motion_controller/task/path.h>
#include <laser/laser_scan_lines.h>
#include <laser/moving_laser_scan.h>
#include <laser/reflected_laser_scan.h>
#include <ui/common/ui_forward_declarations.h>
#include <ui/common/ui_params.h>
#include <ui/components/grid_based_display_widget.h>
#include <gnuplot-iostream.h>
#include <atomic>
#include <deque>
#include <memory>

namespace vulcan
{
namespace ui
{

class OccupancyGridRenderer;
class GlassMapRenderer;
class LaserScanRenderer;
class LinesRenderer;
class ParticlesRenderer;
class PoseTraceRenderer;
class RobotRenderer;
class PoseTargetRenderer;

enum class GlassAnglesToDraw;

enum LocalMetricMapType
{
    LPM,
    GLASS,
    NONE
};

enum class LaserToShowType
{
    raw,
    mapping,
    reflected,
    none,
};

/**
* LocalMetricDisplayWidget is a widget that displays the state information generated
* by the LPM using OpenGL.
*
* The various data to display can be turned on/off using show/hideXXXX methods.
*/
class LocalMetricDisplayWidget : public GridBasedDisplayWidget
{
public:

    LocalMetricDisplayWidget(wxWindow* parent,
                             wxWindowID id = wxID_ANY,
                             const wxPoint& pos = wxDefaultPosition,
                             const wxSize& size = wxDefaultSize,
                             long style = 0,
                             const wxString& name = wxString((const wxChar*)("GLCanvas")),
                             const wxPalette& palette = wxNullPalette);

    virtual ~LocalMetricDisplayWidget(void);

    void setWidgetParams(const lpm_display_params_t& params);

    // Flags for toggling various displays
    void centerOnRobot(bool center);
    void setMapToShow(LocalMetricMapType type) { mapToShow = type; }
    void setLaserToShow(LaserToShowType type) { laserToShow_ = type; }
    void showLaserLines(bool show);
    void showExtractedLines(bool show);
    void showIntensityPlots(bool show) { shouldShowIntensityPlots = show; }
    void showPoseTrace(bool show);
    void showMotionTrace(bool show);
    void showUncertaintyEllipse(bool show);
    void showParticles(bool show);

    // Various commands for controlling the data
    void clearPoseTrace(void);  // clear the trace of poses -- useful when starting a new mapping operation
    void clearMotionTrace(void);

    void setGlassMap(const hssh::GlassMap& glass);
    void showGlassIntensity(bool show);
    void showGlassWalls(bool show);
    void showGlassAngles(bool show);
    void setAnglesToShow(GlassAnglesToDraw angles);
    void flattenGlassMap(int flattenThreshold);
    void filterDynamicObjectsFromGlass(int highlyVisibleThreshold);

    void rotateLPM(float radians);

    // GridBasedDisplayWidget interface
    virtual Point<int> convertWorldToGrid(const Point<float>& world) const;

    const hssh::LocalPerceptualMap& getLPM(void) const { return grid; }

    // Data consumer methods
    void handleData(const polar_laser_scan_t& scan, const std::string& channel);
    void handleData(const laser::MovingLaserScan& scan, const std::string& channel);
    void handleData(const laser::ReflectedLaserScan& scan, const std::string& channel);
    void handleData(const laser::laser_scan_lines_t& scanLines, const std::string& channel);
    void handleData(const robot::proximity_warning_indices_t& proximityIndices, const std::string& channel);

    void handleData(const hssh::LocalPerceptualMap& lpm, const std::string& channel);
    void handleData(const hssh::GlassMap& glass, const std::string& channel);
    void handleData(const pose_t& pose, const std::string& channel);
    void handleData(const pose_distribution_t& distribution, const std::string& channel);
    void handleData(const motion_state_t& motion, const std::string& channel);
    void handleData(const hssh::local_metric_localization_debug_info_t& particles, const std::string& channel);

private:

    enum particle_view_mode_t
    {
        NOT_IN_VIEW_MODE,
        HOVERING,
        SELECTED_PARTICLE
    };

    struct laser_data_t
    {
        polar_laser_scan_t          rawScan;
        laser::MovingLaserScan             mappingScan;
        laser::ReflectedLaserScan          reflectedScan;
        robot::proximity_warning_indices_t proximity;
        laser::laser_scan_lines_t          scanLines;
        GLColor                            color;
        Gnuplot plot;
    };

    std::unique_ptr<OccupancyGridRenderer> gridRenderer;
    std::unique_ptr<GlassMapRenderer>   glassRenderer;
    std::unique_ptr<LaserScanRenderer>  laserRenderer;
    std::unique_ptr<LinesRenderer>      extractedLinesRenderer;
    std::unique_ptr<PoseTraceRenderer>  poseTraceRenderer;
    std::unique_ptr<PoseTraceRenderer>  motionTraceRenderer;
    std::unique_ptr<ParticlesRenderer>  particlesRenderer;
    std::unique_ptr<RobotRenderer>      robotRenderer;

    std::vector<laser_data_t> laserData;
    int64_t                   mostRecentLaserTime;

    pose_t              pose;
    pose_distribution_t poseDistribution;
    motion_state_t      motionState;
    pose_distribution_t stateEstimatorDistribution;
    hssh::LocalPerceptualMap   grid;
    hssh::GlassMap             glass;
    std::vector<hssh::GlassWall> glassWalls;

    Point<int> glassCell_;
    hssh::particle_filter_debug_info_t particles;

    std::deque<pose_t> poseTrace;
    std::deque<pose_t> motionStateTrace;
    std::size_t               maxTraceLength;

    LocalMetricMapType mapToShow;
    LaserToShowType laserToShow_;

    bool shouldCenterOnRobot;
    bool shouldShowExtractedLines;
    std::atomic<bool> shouldShowIntensityPlots;
    bool shouldShowPoseTrace;
    bool shouldShowMotionTrace;
    bool shouldShowUncertaintyEllipse;
    bool shouldShowParticles;
    bool shouldShowGlassWalls;
    bool shouldShowGlassAngles;

    bool haveScan;
    bool havePose;
    bool haveGlass;
    bool haveNewGrid;
    bool haveNewGlass;

    particle_view_mode_t particleMode;
    std::size_t          selectedParticleId;

    lpm_display_params_t params;

    utils::Mutex dataLock;

    // GridBasedDisplayWidget interface
    void renderWidget(void) override;
    std::string printCellInformation(Point<int> cell) override;

    void handleMouseDown (wxMouseEvent& event);
    void handleMouseUp   (wxMouseEvent& event);
    void handleMouseMoved(wxMouseEvent& event);
    void handleParticleMouseEvent(wxMouseEvent& event);
    int  findClosestParticle(const Point<float>& worldPosition);
    bool updateTrace(const pose_t& pose, std::deque<pose_t>& trace);

    DECLARE_EVENT_TABLE()
};

}
}

#endif // UI_DEBUG_LPM_DISPLAY_WIDGET_H

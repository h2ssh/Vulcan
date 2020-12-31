/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     relocalization_display_widget.h
* \author   Collin Johnson
*
* Declaration of RelocalizationDisplayWidget.
*/

#ifndef UI_DEBUG_RELOCALIZATION_DISPLAY_WIDGET_H
#define UI_DEBUG_RELOCALIZATION_DISPLAY_WIDGET_H

#include "ui/components/grid_based_display_widget.h"
#include "ui/common/gl_camera.h"
#include "ui/common/ui_color.h"
#include "hssh/local_metric/lpm.h"
#include "hssh/local_metric/debug_info.h"
#include "hssh/metrical/relocalization/debug_info.h"
#include "core/pose_distribution.h"
#include <mutex>

namespace vulcan
{
namespace ui
{

class  LaserScanRenderer;
class  OccupancyGridRenderer;
class  ParticlesRenderer;
class  RobotRenderer;
struct ui_params_t;

/**
* RelocalizationDisplayWidget
*/
class RelocalizationDisplayWidget : public GridBasedDisplayWidget
{
public:

    /**
    * Constructor for RelocalizationWidget.
    */
    RelocalizationDisplayWidget(wxWindow*        parent,
                                wxWindowID       id      = wxID_ANY,
                                const wxPoint&   pos     = wxDefaultPosition,
                                const wxSize&    size    = wxDefaultSize,
                                long             style   = 0,
                                const wxString&  name    = wxString((const wxChar*)("GLCanvas")),
                                const wxPalette& palette = wxNullPalette);

    void setWidgetParams(const ui_params_t& params);

    // Display options
    void showLaser        (bool show) { shouldShowLaser_         = show; }
    void showErrorEllipse (bool show) { shouldShowError_         = show; }
    void showParticles    (bool show) { shouldShowParticles_     = show; }
    void showInitialRegion(bool show) { shouldShowInitialRegion_ = show; }

    // Set the data for rendering
    void setMap               (const std::shared_ptr<hssh::OccupancyGrid>&     map);
    void setRelocalizationInfo(const hssh::metric_relocalization_debug_info_t& info);
    void setInitializerRegion (const math::Rectangle<float>&                   rectangle);

private:
    
    OccupancyGridRenderer* mapRenderer_;
    ParticlesRenderer*     particlesRenderer_;
    RobotRenderer*         robotRenderer_;
    LaserScanRenderer*     scanRenderer_;
    
    std::shared_ptr<hssh::OccupancyGrid> map_;
    pose_distribution_t           meanPose_;
    hssh::particle_filter_debug_info_t   particleInfo_;
    std::vector<hssh::particle_t>        initialParticles_;
    math::Rectangle<float>               regionInitializerRectangle_;
    
    bool haveNewMap_;
    bool shouldShowLaser_;
    bool shouldShowError_;
    bool shouldShowParticles_;
    bool shouldShowInitialRegion_;
    
    GLColor initialRegionColor;
    GLColor initialParticlesColor;
    
    mutable std::mutex dataLock_;

    // OpenGLWidget interface
    virtual void renderWidget(void);

    // GridBasedDisplayWidget interface
    virtual Point<int> convertWorldToGrid(const Point<float>& world) const;

    void renderMap(void);
    void renderInitialization(void);
};

}
}

#endif // UI_DEBUG_RELOCALIZATION_DISPLAY_WIDGET_H

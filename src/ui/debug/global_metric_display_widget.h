/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_metric_display_widget.h
* \author   Collin Johnson
* 
* Declaration of GlobalMetricDisplayWidget.
*/

#ifndef UI_DEBUG_GLOBAL_METRIC_DISPLAY_WIDGET_H
#define UI_DEBUG_GLOBAL_METRIC_DISPLAY_WIDGET_H

#include "ui/components/grid_based_display_widget.h"
#include "hssh/global_metric/pose.h"
#include "hssh/metrical/localization/debug_info.h"
#include "hssh/metrical/relocalization/debug_info.h"
#include "utils/pose_trace.h"
#include <mutex>

namespace vulcan
{
namespace hssh { class GlobalMetricMap; }
namespace ui
{
    
class OccupancyGridRenderer;
class ParticlesRenderer;
class PoseTraceRenderer;
class RobotRenderer;

struct ui_params_t;

/**
* GlobalMetricDisplayWidget displays the state of the global_metric_hssh module. The displayed information is:
* 
*   1) The current GlobalMetricMap being used for global metric navigation.
*   2) The best estimate of the robot pose, with uncertainty.
*   3) The particle filter information -- either from the relocalizer or from MCL.
*/
class GlobalMetricDisplayWidget : public GridBasedDisplayWidget
{
public:
    
    GlobalMetricDisplayWidget(wxWindow* parent,
                              wxWindowID id = wxID_ANY,
                              const wxPoint& pos = wxDefaultPosition,
                              const wxSize& size = wxDefaultSize,
                              long style = 0,
                              const wxString& name = wxString((const wxChar*)("GLCanvas")),
                              const wxPalette& palette = wxNullPalette);
    
    /**
    * setParams supplies the parameters for the rendering.
    */
    void setParams(const ui_params_t& params);
    
    // Methods for setting the data to be shown
    void setMap               (const std::shared_ptr<const hssh::GlobalMetricMap>& map);
    void setPose              (const hssh::GlobalPose&                             pose);
    void setLocalizationInfo  (const hssh::particle_filter_debug_info_t&           info);
    void setRelocalizationInfo(const hssh::metric_relocalization_debug_info_t&     info);
    
private:
    
    // A flag based on the most recent information received from global_metric_hssh
    // The flag is changed whenever new data arrives
    enum class ModuleState
    {
        Waiting,
        Relocalizing,
        Localized
    };
    
    std::shared_ptr<const hssh::GlobalMetricMap> map_;
    hssh::GlobalPose                             pose_;
    hssh::particle_filter_debug_info_t           localizationInfo_;
    hssh::metric_relocalization_debug_info_t     relocalizationInfo_;
    
    utils::PoseTrace trace_;
    
    ModuleState moduleState_;
    bool        haveNewMap_;
    int64_t     relocalizationStartTime_;
    
    OccupancyGridRenderer* mapRenderer_;
    RobotRenderer*         robotRenderer_;
    PoseTraceRenderer*     traceRenderer_;
    ParticlesRenderer*     particlesRenderer_;
    
    std::mutex dataLock_;
    
    // OpenGLWidget interface
    virtual void renderWidget(void) override;

    // GridBasedDisplayWidget interface
    virtual Point<int> convertWorldToGrid(const Point<float>& world) const override;    
};
    
}
}

#endif // UI_DEBUG_GLOBAL_METRIC_DISPLAY_WIDGET_H

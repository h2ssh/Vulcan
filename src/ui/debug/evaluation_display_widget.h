/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     evaluation_widget.h
* \author   Collin Johnson
*
* Declaration of EvaluationDisplayWidget.
*/

#ifndef UI_DEBUG_EVALUATION_WIDGET_H
#define UI_DEBUG_EVALUATION_WIDGET_H

#include <ui/components/grid_based_display_widget.h>
#include <hssh/local_topological/evaluation/stability_analyzer.h>
#include <utils/pose_trace.h>
#include <memory>

namespace vulcan
{
namespace mpepc { class PathSummary; }
namespace ui
{

class GLColor;
class PoseTraceRenderer;
class StableAreaRenderer;
class VoronoiSkeletonGridRenderer;

/**
* EvaluationDisplayWidget draws information about the evaluation of the topological abstraction for an environment.
*/
class EvaluationDisplayWidget : public GridBasedDisplayWidget
{
public:

    /**
    * Constructor for EvaluationDisplayWidget.
    */
    EvaluationDisplayWidget(wxWindow* parent,
                            wxWindowID id = wxID_ANY,
                            const wxPoint& pos = wxDefaultPosition,
                            const wxSize& size = wxDefaultSize,
                            long int style = 0,
                            const wxString& name = wxString((const wxChar*)("GLCanvas")),
                            const wxPalette& palette = wxNullPalette);

    /**
    * Destructor for EvaluationDisplayWidget.
    */
    virtual ~EvaluationDisplayWidget(void);

    // Change the content being rendered
    void setMap(const std::shared_ptr<hssh::LocalTopoMap>& map);
    void setAreas(const std::shared_ptr<hssh::AreaStabilityAnalyzer>& areas);

    void setPaths(const mpepc::PathSummary& paths);

    // Options for rendering the areas
    void shouldDrawBoundaries(bool draw);
    void shouldDrawStars(bool draw);

private:

    std::unique_ptr<VoronoiSkeletonGridRenderer> mapRenderer_;
    std::unique_ptr<StableAreaRenderer> areaRenderer_;
    std::unique_ptr<PoseTraceRenderer> traceRenderer_;

    std::shared_ptr<hssh::LocalTopoMap> map_;
    std::shared_ptr<hssh::AreaStabilityAnalyzer> areas_;

    std::vector<utils::PoseTrace> regularTraces_;
    std::vector<utils::PoseTrace> socialTraces_;

    bool haveNewMap_ = false;
    uint8_t areaMask_;


    // GridBasedDisplayWidget interface
    Point<int> convertWorldToGrid(const Point<float>& world) const override;
    void renderWidget(void) override;

    // Helpers
    void renderTrace(const utils::PoseTrace& trace, const GLColor& color);
};

} // namespace ui
} // namespace vulcan

#endif // UI_DEBUG_EVALUATION_WIDGET_H

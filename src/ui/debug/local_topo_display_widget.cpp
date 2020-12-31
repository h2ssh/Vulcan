/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_display_widget.cpp
* \author   Collin Johnson
*
* Definition of LocalTopoDisplayWidget.
*/

#include "ui/debug/local_topo_display_widget.h"
#include "ui/components/labeling_csp_player.h"
#include "ui/components/place_grid_renderer.h"
#include "ui/components/gateways_renderer.h"
#include "ui/components/isovist_renderer.h"
#include "ui/components/voronoi_isovist_gradients_renderer.h"
#include "ui/components/area_graph_renderer.h"
#include "ui/components/visibility_graph_renderer.h"
#include "ui/components/area_subgraph_renderer.h"
#include "ui/components/local_area_renderer.h"
#include "ui/components/local_area_event_renderer.h"
#include "ui/common/grid_cell_selector.h"
#include "ui/common/color_generator.h"
#include "hssh/local_metric/pose.h"
#include "utils/auto_mutex.h"
#include <iterator>
#include <sstream>
#include <cassert>

namespace vulcan
{
namespace ui
{

enum
{
    kShowLocalTopoMap,
    kShowLocalTopoGraph,
    kShowHSSHGraph,
    kShowNoAreas,
    kNumShowAreas
};

enum
{
    kDisplayLabels,
    kDisplayDistribution,
    kDisplayFeatures,
    kNumDisplays
};


LocalTopoDisplayWidget::LocalTopoDisplayWidget(wxWindow* parent,
                                               wxWindowID id,
                                               const wxPoint& pos,
                                               const wxSize& size,
                                               long style,
                                               const wxString& name,
                                               const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, gridRenderer(new VoronoiSkeletonGridRenderer)
, gatewaysRenderer(new GatewaysRenderer)
, graphRenderer(new AreaGraphRenderer)
, visibilityGraphRenderer_(new VisibilityGraphRenderer)
, hypothesisRenderer(new AreaHypothesisRenderer)
, areaRenderer_(new LocalAreaRenderer)
, eventRenderer_(new LocalAreaEventRenderer)
, isovistRenderer(new IsovistRenderer)
, gradientsRenderer_(new VoronoiIsovistGradientsRenderer)
, cspPlayer_(nullptr)
, gridWidth(0)
, gridHeight(0)
, selectedHypothesis_(nullptr)
, showTopoMapAsHeatMap_(false)
, havePath(false)
, gridIsDirty(false)
, gradientsAreDirty_(false)
, probabilitiesAreDirty_(false)
, visibilityGraphIsDirty_(false)
, doFollowRobot(true)
, shouldShowGateways(true)
, shouldShowNormals(false)
, shouldShowAreaGraph(true)
, shouldShowFrontiers(true)
, areasToShow(kShowLocalTopoMap)
, shouldShowIsovist(false)
, shouldShowIsovistField(false)
, shouldShowIsovistDerivField_(false)
, shouldShowGradients_(false)
, shouldShowGradientMaxima_(false)
, shouldShowProbabilities_(false)
{
    enableScrolling();
    isovistSelector_.reset(new GridCellSelector(this));
}


LocalTopoDisplayWidget::~LocalTopoDisplayWidget(void)
{
}


void LocalTopoDisplayWidget::setWidgetParams(const local_topo_display_params_t& params)
{
    this->params = params;

    gridRenderer->setRenderColors(params.frontierColor,
                                  params.skeletonCellColor,
                                  params.reducedCellColor);

    gatewaysRenderer->setRenderColors(params.exploredColor, params.frontierColor, params.gatewayEndpointColor);
    isovistRenderer->setRenderColors(params.isovistRayColor, params.isovistFieldColors);
    gradientsRenderer_->setRenderColors(params.isovistRayColor, params.isovistFieldColors);
    graphRenderer->setRenderColors(params.gatewayColor, params.junctionColor, params.frontierColor, params.deadEndColor);
    hypothesisRenderer->setRenderColors(params.unknownColor,
                                        params.decisionColor,
                                        params.destinationColor,
                                        params.pathColor,
                                        params.isovistFieldColors);
}


void LocalTopoDisplayWidget::showVoronoiGrid(bool show)
{
    showVoronoiGrid_ = show;
}


void LocalTopoDisplayWidget::showDistanceGradient(bool show)
{
    gridRenderer->showDistances(show);
    gridIsDirty = true;
}


void LocalTopoDisplayWidget::showFullSkeleton(bool show)
{
    gridRenderer->showFullSkeleton(show);
    gridIsDirty = true;
}


void LocalTopoDisplayWidget::showReducedSkeleton(bool show)
{
    gridRenderer->showReducedSkeleton(show);
    gridIsDirty = true;
}


void LocalTopoDisplayWidget::shouldFollowRobot(bool follow)
{
    doFollowRobot = follow;
}


void LocalTopoDisplayWidget::showFrontiers(bool show)
{
    shouldShowFrontiers = show;

    gridRenderer->showFrontiers(show);
    gridIsDirty = true;
}


void LocalTopoDisplayWidget::showSmallScaleStar(bool show)
{
    areaRenderer_->showStar(show);
}


void LocalTopoDisplayWidget::showAreaGateways(bool show)
{
    areaRenderer_->showGateways(show);
}


void LocalTopoDisplayWidget::showIsovist(bool show)
{
    if(show && !shouldShowIsovist)
    {
        pushMouseHandler(isovistSelector_.get());
    }
    else if(!show && shouldShowIsovist)
    {
        removeMouseHandler(isovistSelector_.get());
        shouldSelectIsovists = false;
    }

    shouldShowIsovist = show;
    selectedIsovists_.clear();
    isovistSelector_->clearSelectedCells();
}


void LocalTopoDisplayWidget::selectIsovists(bool select)
{
    shouldSelectIsovists = select;
    isovistSelector_->clearSelectedCells();
}


void LocalTopoDisplayWidget::setHypothesisValueToDisplay(int toDisplay)
{
    if(toDisplay >= 0 && toDisplay < kNumDisplays)
    {
        areaHypothesisValue_ = toDisplay;
    }
}


void LocalTopoDisplayWidget::setSkeleton(std::shared_ptr<hssh::VoronoiSkeletonGrid> skeleton)
{
    if(!skeleton)
    {
        return;
    }

    utils::AutoMutex autoLock(dataLock);

    grid        = skeleton;
    gridIsDirty = true;

    if(gridWidth != grid->getWidthInCells() || gridHeight != grid->getHeightInCells())
    {
        gridWidth  = grid->getWidthInCells();
        gridHeight = grid->getHeightInCells();
    }

    selectedSources.clear();  // each grid erases the previously selected sources

    // When the map changes, auto-zoom
    setViewRegion(grid->getWidthInMeters(), grid->getHeightInMeters());
    if(!doFollowRobot)
    {
        setCameraFocalPoint(grid->getGlobalCenter());
    }
//     setGridDimensions(grid->getWidthInMeters(), grid->getHeightInMeters());
}


void LocalTopoDisplayWidget::setGateways(const std::vector<hssh::Gateway>& gateways)
{
    utils::AutoMutex autoLock(dataLock);

    this->gateways = gateways;
}


void LocalTopoDisplayWidget::setLocalTopoMap(std::shared_ptr<hssh::LocalTopoMap> map)
{
    utils::AutoMutex autoLock(dataLock);
    topoMap_ = map;
}


void LocalTopoDisplayWidget::setLocation(const hssh::LocalLocation& location)
{
    utils::AutoMutex autoLock(dataLock);
    location_ = location;
}


void LocalTopoDisplayWidget::setVisibleHypotheses(const std::vector<hssh::DebugHypothesis>& hypotheses)
{
    utils::AutoMutex autoLock(dataLock);
    visibleHypotheses_ = hypotheses;
}


void LocalTopoDisplayWidget::setSelectedHypothesis(const hssh::DebugHypothesis* hypothesis)
{
    utils::AutoMutex autoLock(dataLock);
    selectedHypothesis_ = hypothesis;
}


void LocalTopoDisplayWidget::setHypothesisFeatureToShow(int featureIndex)
{
    utils::AutoMutex autoLock(dataLock);
    hypFeatureIndex_ = std::max(0, featureIndex);
}


void LocalTopoDisplayWidget::setEvent(std::shared_ptr<hssh::LocalAreaEvent> event)
{
    utils::AutoMutex autoLock(dataLock);
    event_ = event;
}


void LocalTopoDisplayWidget::setIsovistField(std::shared_ptr<hssh::VoronoiIsovistField> field, utils::Isovist::Scalar scalar)
{
    utils::AutoMutex autoLock(dataLock);
    field_  = field;
    scalar_ = scalar;
    selectedIsovists_.clear();
}


void LocalTopoDisplayWidget::setIsovistGradients(std::shared_ptr<hssh::VoronoiIsovistGradients> gradients, std::shared_ptr<hssh::VoronoiIsovistMaxima> maxima)
{
    utils::AutoMutex autoLock(dataLock);

    gradients_         = gradients;
    maxima_            = maxima;
    gradientsAreDirty_ = true;
}


void LocalTopoDisplayWidget::setGatewayProbabilities(const hssh::CellToTypeMap<double>& probabilities)
{
    utils::AutoMutex autoLock(dataLock);

    probabilities_ = probabilities;
    probabilitiesAreDirty_ = true;
}


void LocalTopoDisplayWidget::setHeatMap(const hssh::HeatMapStatistics& heatMap)
{
    utils::AutoMutex autoLock(dataLock);
    heatMap_ = heatMap;
}


void LocalTopoDisplayWidget::setCSPPlayer(LabelingCSPPlayer* player)
{
    cspPlayer_ = player;
}


void LocalTopoDisplayWidget::setVisibilityGraphFeature(utils::VisibilityGraphFeatureType featureType)
{
    utils::AutoMutex autoLock(dataLock);

    assert(featureType != utils::VisibilityGraphFeatureType::num_features);

    if(visGraphFeatureType_ != featureType)
    {
        visGraphFeatureType_ = featureType;

        // If the feature has changed, then recompute the visiblity graph
        if(visGraphFeatureType_ != utils::VisibilityGraphFeatureType::none)
        {
            visibilityGraphIsDirty_ = true;
        }
    }
}


void LocalTopoDisplayWidget::handleData(const hssh::LocalPose& pose, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock);

    robotPose = pose.pose();
}


void LocalTopoDisplayWidget::handleData(const hssh::local_area_debug_info_t& info, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock);
    areaInfo = info;
    visibilityGraphIsDirty_ = true;
}


GLEventStatus LocalTopoDisplayWidget::handleRightMouseUp(const GLMouseEvent& event)
{
    // Nothing to do if there isn't a grid yet.
    if(!grid)
    {
        return GridBasedDisplayWidget::handleRightMouseUp(event);
    }

    Point<int> cell = convertWorldToGrid(event.glCoords);

    const auto& skeletonSources = grid->getSkeletonSources();
    auto skeletonCellIt = skeletonSources.find(cell);

    if(skeletonCellIt != skeletonSources.end())
    {
        bool shouldReset = selectedSources.find(cell) != selectedSources.end();

        if(shouldReset)
        {
            gridRenderer->resetCellColor(cell, grid->getClassification(cell.x, cell.y));
            selectedSources.erase(cell);
        }
        else
        {
            gridRenderer->setCellColor(cell, params.sourceColor);
            selectedSources.insert(cell);
        }

        for(auto& source : skeletonCellIt->second)
        {
            if(shouldReset)
            {
                gridRenderer->resetCellColor(source, grid->getClassification(source));
            }
            else
            {
                gridRenderer->setCellColor(source, params.sourceColor);
            }
        }
    }

    return GridBasedDisplayWidget::handleRightMouseUp(event);
}


void LocalTopoDisplayWidget::renderWidget(void)
{
    utils::AutoMutex lock(dataLock);

    // Nothing to do if there isn't a grid yet -- that forms the backbone of everything.
    if(!grid)
    {
        return;
    }

    if(doFollowRobot)
    {
        setCameraFocalPoint(robotPose.toPoint());
    }

    renderGrid();
    renderIsovists();
    renderGateways();
    renderAreas();
    renderPose();
}


Point<int> LocalTopoDisplayWidget::convertWorldToGrid(const Point<float>& world) const
{
    if(grid)
    {
        return utils::global_point_to_grid_cell(world, *grid);
    }
    return Point<int>(0, 0);
}


std::string LocalTopoDisplayWidget::printCellInformation(Point<int> cell)
{
    std::string information = printVoronoiDistance(cell);

    if(selectedHypothesis_)
    {
        information += printHypothesisFeatures();
    }

    if(field_)
    {
        information += printIsovistInformation(cell);
    }

    return information;
}


void LocalTopoDisplayWidget::renderGrid(void)
{
    if(gridIsDirty)
    {
        gridRenderer->setGrid(*grid);
        gridIsDirty = false;
    }

    if(showVoronoiGrid_)
    {
        gridRenderer->renderGrid();
    }
}


void LocalTopoDisplayWidget::renderIsovists(void)
{
    if(gradientsAreDirty_ && gradients_ && maxima_ && grid)
    {
        gradientsRenderer_->setGradients(*gradients_, *maxima_, *grid);
        gradientsAreDirty_ = false;
    }

    if(probabilitiesAreDirty_ && grid)
    {
        gradientsRenderer_->setProbabilities(probabilities_, *grid);
        probabilitiesAreDirty_ = false;
    }

    if(shouldShowIsovist && field_ && field_->contains(isovistSelector_->hoverCell()))
    {
        isovistRenderer->renderIsovist(field_->at(isovistSelector_->hoverCell()));

        for(std::size_t n = 0; n < selectedIsovists_.size(); ++n)
        {
            isovistRenderer->renderIsovist(field_->at(selectedIsovists_[n]), isovistColors_[n]);
        }
    }

    if(shouldSelectIsovists && field_)
    {
        if(!isovistSelector_->isActivelySelecting())
        {
            for(auto selected : isovistSelector_->selectedCells())
            {
                auto indexIt = std::find(selectedIsovists_.begin(), selectedIsovists_.end(), selected);
                if(indexIt != selectedIsovists_.end())
                {
                    selectedIsovists_.erase(indexIt);
                }
                else if(field_->contains(selected))
                {
                    selectedIsovists_.push_back(selected);
                }
            }
            isovistSelector_->clearSelectedCells();

            if(isovistColors_.size() != selectedIsovists_.size())
            {
                isovistColors_ = generate_colors(selectedIsovists_.size(), 0.75);
            }
        }
    }

    if(shouldShowIsovistField && field_)
    {
        isovistRenderer->renderIsovistField(field_->begin(), field_->end(), scalar_, grid->metersPerCell());
    }

    if(shouldShowIsovistDerivField_ && field_)
    {
        isovistRenderer->renderIsovistDerivField(field_->begin(), field_->end(), scalar_, grid->metersPerCell());
    }

    if(shouldShowGradients_)
    {
        gradientsRenderer_->renderCellGradients();
    }

    if(shouldShowGradientMaxima_)
    {
        gradientsRenderer_->renderLocalMaxima();
    }

    if(shouldShowProbabilities_)
    {
        gradientsRenderer_->renderProbabilities();
    }
}


void LocalTopoDisplayWidget::renderGateways(void)
{
    if(shouldShowGateways)
    {
        gatewaysRenderer->renderGateways(gateways, shouldShowNormals);
    }
}


void LocalTopoDisplayWidget::renderAreas(void)
{
    if(shouldShowAreaGraph)
    {
        graphRenderer->renderGraph(areaInfo.graph);
    }

    if(shouldShowVisibilityGraph_ && grid)
    {
        if(visibilityGraphIsDirty_)
        {
            visGraph_ = areaInfo.graph.toVisibilityGraph(1.0, *grid);

            if(visGraphFeatureType_ != utils::VisibilityGraphFeatureType::none)
            {
                visGraphFeature_ = visGraph_.calculateFeature(visGraphFeatureType_);
            }

            visibilityGraphIsDirty_ = false;
        }

        // If not showing a visibility graph feature, then just show the graph regularly
        if(visGraphFeatureType_ == utils::VisibilityGraphFeatureType::none)
        {
            visibilityGraphRenderer_->renderVisibilityGraph(visGraph_, grid->getBottomLeft(), grid->metersPerCell());
        }
        // Otherwise provide the feature being used as well
        else
        {
            visibilityGraphRenderer_->renderVisibilityGraph(visGraph_,
                                                            visGraphFeature_,
                                                            grid->getBottomLeft(),
                                                            grid->metersPerCell());
        }
    }

    // If the feature index is valid, then calculate a new normalizer
    double maxValue = 1.0;  // if nothing has a value greater than 1, then there's no need to normalize!
    for(auto& hyp : visibleHypotheses_)
    {
        maxValue = std::max(maxValue, hyp.featureValue(hypFeatureIndex_));
    }
    hypFeatureNormalizer_ = 1.0 / maxValue;


    for(auto& hyp : visibleHypotheses_)
    {
        renderHypothesis(hyp);
    }

    if(selectedHypothesis_)
    {
        renderHypothesis(*selectedHypothesis_);
    }

    areaRenderer_->setMetersPerCell(grid->metersPerCell());

    switch(areasToShow)
    {
    case kShowLocalTopoMap:
        if(topoMap_)
        {
            if(showTopoMapAsHeatMap_)
            {
                areaRenderer_->renderLocalTopoMapAsHeatMap(*topoMap_, heatMap_);
            }
            else
            {
                areaRenderer_->renderLocalTopoMap(*topoMap_);
            }
        }
        break;

    case kShowLocalTopoGraph:
        if(topoMap_)
        {
            areaRenderer_->renderLocalTopoMapAsGraph(*topoMap_);
        }
        break;

    case kShowHSSHGraph:
        if(topoMap_)
        {
            areaRenderer_->renderLocalTopoMapAsHSSHGraph(*topoMap_);
        }
        break;

    case kShowNoAreas:
    default:
        break;
    }

    if(cspPlayer_)
    {
        cspPlayer_->update();
    }

    if(shouldShowEvents_ && event_)
    {
        eventRenderer_->renderEvent(*event_);
    }
}


void LocalTopoDisplayWidget::renderPose(void)
{
    if(showVoronoiGrid_)
    {
        glPointSize(10.0f);
        glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
        glBegin(GL_POINTS);
        glVertex2f(robotPose.x, robotPose.y);
        glEnd();
    }
}


void LocalTopoDisplayWidget::renderProposals(const std::vector<hssh::AreaProposal>& proposals)
{
    for(auto& proposal : proposals)
    {
        areaRenderer_->renderAreaProposal(proposal, *grid);
    }
}


void LocalTopoDisplayWidget::renderHypothesis(const hssh::DebugHypothesis& hypothesis)
{
    switch(areaHypothesisValue_)
    {
    case kDisplayDistribution:
        hypothesisRenderer->renderHypothesisDistribution(hypothesis, grid->metersPerCell());
        break;

    case kDisplayFeatures:
        hypothesisRenderer->renderHypothesis(hypothesis,
                                             hypothesis.featureValue(hypFeatureIndex_) * hypFeatureNormalizer_,
                                             grid->metersPerCell());
        break;

    case kDisplayLabels:
    default:
        hypothesisRenderer->renderHypothesis(hypothesis, grid->metersPerCell());
        break;
    }
}


std::string LocalTopoDisplayWidget::printVoronoiDistance(hssh::cell_t cell) const
{
    std::ostringstream dist;

    if(grid)
    {
        dist << "Dist: " << grid->getMetricDistance(cell.x, cell.y) << ' ';
    }

    return dist.str();
}


std::string LocalTopoDisplayWidget::printHypothesisFeatures(void) const
{
    std::ostringstream hypFeatures;

    if(selectedHypothesis_)
    {
        hypFeatures << "Hypothesis: Features: ";
        std::copy(selectedHypothesis_->featureBegin(),
                  selectedHypothesis_->featureEnd(),
                  std::ostream_iterator<double>(hypFeatures, " "));
    }

    return hypFeatures.str();
}


std::string LocalTopoDisplayWidget::printIsovistInformation(hssh::cell_t cell) const
{
    std::ostringstream isovistInfo;

    if(field_ && field_->contains(cell))
    {
        const auto& isovist = field_->at(cell);
        isovistInfo << "Isovist: Value: " << isovist.scalar(scalar_) << " Deriv: "  << isovist.scalarDeriv(scalar_);

        if(gradients_)
        {
            isovistInfo << " Gradient: " << gradients_->gradientAt(cell).value;
        }
    }

    return isovistInfo.str();
}

} // namespace ui
} // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_topo_display_widget.cpp
* \author   Collin Johnson
*
* Definition of GlobalTopoDisplayWidget.
*/

#include "ui/debug/global_topo_display_widget.h"
#include "ui/components/topological_map_renderer.h"
#include "ui/components/place_view_topological_map_renderer.h"
#include "ui/components/graph_view_topological_map_renderer.h"
#include "ui/components/hypothesis_tree_renderer.h"
#include "ui/components/global_topo_map_previewer.h"
#include "ui/common/gl_utilities.h"
#include "hssh/global_topological/topological_map.h"
#include "hssh/global_topological/mapping/tree_of_maps.h"
#include "hssh/local_topological/events/area_transition.h"
#include "utils/auto_mutex.h"
#include <GL/gl.h>
#include <iostream>

namespace vulcan
{
namespace ui
{

const double HYP_TREE_RATIO = 0.66;

BEGIN_EVENT_TABLE(GlobalTopoDisplayWidget, OpenGLWidget)
    EVT_MOTION   (GlobalTopoDisplayWidget::mouseMotion)
    EVT_LEFT_DOWN(GlobalTopoDisplayWidget::mouseLeftDown)
    EVT_LEFT_UP  (GlobalTopoDisplayWidget::mouseLeftUp)
    EVT_RIGHT_UP (GlobalTopoDisplayWidget::mouseRightUp)
END_EVENT_TABLE()

GlobalTopoDisplayWidget::GlobalTopoDisplayWidget(wxWindow* parent,
                                                 wxWindowID id,
                                                 const wxPoint& pos,
                                                 const wxSize& size,
                                                 long style,
                                                 const wxString& name,
                                                 const wxPalette& palette)
: OpenGLWidget(parent, id, pos, size, style|wxFULL_REPAINT_ON_RESIZE, name, palette)
, placeViewRenderer(new PlaceViewTopologicalMapRenderer())
, graphViewRenderer(new GraphViewTopologicalMapRenderer())
, activeGraphRenderer(graphViewRenderer.get())
, treeRenderer(new HypothesisTreeRenderer())
, mapPreviewer(new GlobalTopoMapPreviewer(3, 1, graphViewRenderer.get()))
, shouldShowBestMap(true)
, viewToShow(PLACE_VIEW)
, mapToShowId(100000000)
, hoverId(1000000)
, haveMap(false)
, showingNewMap(true)
, showingNewTree(true)
{
    setCameraFocalPoint(Point<float>(0, 0));
}


GlobalTopoDisplayWidget::~GlobalTopoDisplayWidget(void) = default;


void GlobalTopoDisplayWidget::setWidgetParams(const global_topo_display_params_t& params, const lpm_display_params_t& localMetricParams)
{
    placeViewRenderer->setRenderColors(params.placeColor, params.pathColor, params.locationColor, params.frontierColor);
    graphViewRenderer->setRenderColors(params.placeColor, params.pathColor, params.locationColor, params.frontierColor);
    treeRenderer->setRenderColors(params.nodeColor, params.edgeColor);

    this->params = params;
}


hssh::hypothesis_tree_node_t GlobalTopoDisplayWidget::getDisplayedMap(void) const
{
    const hssh::hypothesis_tree_node_t* node = nullptr;

    if(shouldShowBestMap)
    {
        node = hypTree.getNode(bestTopoMap.id);
    }
    else
    {
        node = hypTree.getNode(mapToShowId);
    }

    return node ? *node : hssh::hypothesis_tree_node_t();
}


hssh::hypothesis_tree_node_t GlobalTopoDisplayWidget::getHoverMap(void) const
{
    auto node = hypTree.getNode(hoverId);
    return node ? *node : hssh::hypothesis_tree_node_t();
}


void GlobalTopoDisplayWidget::setActiveView(active_view_t viewToShow)
{
    utils::AutoMutex autoLock(dataLock);

    switch(viewToShow)
    {
    case PLACE_VIEW:
        activeGraphRenderer = placeViewRenderer.get();
        showingNewMap       = this->viewToShow != PLACE_VIEW;
        break;

    case GRAPH_VIEW:
        activeGraphRenderer = graphViewRenderer.get();
        showingNewMap       = this->viewToShow != GRAPH_VIEW;
        break;

    case HYPOTHESIS_TREE:
        showingNewTree = this->viewToShow != HYPOTHESIS_TREE;
        break;

    default:
        break;
    }

    this->viewToShow = viewToShow;
}


void GlobalTopoDisplayWidget::setIdToShow(hssh::Id id)
{
    utils::AutoMutex autoLock(dataLock);

    // Ensure a valid index is set
    if((id != mapToShowId) && treeOfMaps && treeOfMaps->stateWithId(id))
    {
        mapToShowId   = id;
        showingNewMap = true;
    }
}


void GlobalTopoDisplayWidget::showBestMap(bool show)
{
    shouldShowBestMap = show;
    showingNewMap     = true;
}


void GlobalTopoDisplayWidget::setTreeOfMaps(const std::shared_ptr<hssh::TreeOfMaps>& maps)
{
    utils::AutoMutex autoLock(dataLock);
    treeOfMaps = maps;
}


void GlobalTopoDisplayWidget::setMapCache(const hssh::MetricMapCache& cache)
{
    utils::AutoMutex autoLock(dataLock);
    placeManager = cache;
}


void GlobalTopoDisplayWidget::clearMaps(void)
{
    utils::AutoMutex autoLock(dataLock);
    treeOfMaps.reset();
}

// GlobalTopologyDataConsumer interface
void GlobalTopoDisplayWidget::handleData(const hssh::TopologicalState& map, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock);

    treeRenderer->cancelHighlight(bestTopoMap.id);
    treeRenderer->highlightHypothesis(map.id, params.bestNodeColor);

    bestTopoMap = map;
    showingNewMap |= shouldShowBestMap || treeOfMaps->empty();
}


void GlobalTopoDisplayWidget::handleData(const hssh::HypothesisTree& tree, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock);
    treeRenderer->setHypothesisTree(tree);
    hypTree = tree;
    showingNewTree = true;
}


void GlobalTopoDisplayWidget::handleData(const hssh::LocalAreaEventVec& events, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock);

    hssh::CacheEventVisitor visitor(placeManager);
    for(auto& e : events)
    {
        e->accept(visitor);
    }
}


void GlobalTopoDisplayWidget::getViewportBoundary(Point<int>& bottomLeft, int& width, int& height)
{
    // In HypothesisTree mode,
    if(viewToShow != HYPOTHESIS_TREE)
    {
        return OpenGLWidget::getViewportBoundary(bottomLeft, width, height);
    }

    wxSize size = GetSize();

    bottomLeft.x = 0;
    bottomLeft.y = 0;
    height       = size.GetHeight();
    width        = size.GetWidth() * HYP_TREE_RATIO;

    mapPreviewer->setBoundary(Point<int>(bottomLeft.x+width, bottomLeft.y), size.GetWidth()*(1.0 - HYP_TREE_RATIO), height);
}


void GlobalTopoDisplayWidget::renderWidget(void)
{
    utils::AutoMutex autoLock(dataLock);

    math::Rectangle<float> boundary;
    Point<float>     boundaryCenter;

    bool shouldAdjustCamera = false;

    auto mapToRender = (shouldShowBestMap || !treeOfMaps || treeOfMaps->empty()) ?
        &bestTopoMap : treeOfMaps->stateWithId(mapToShowId);

    switch(viewToShow)
    {
    case PLACE_VIEW:
    case GRAPH_VIEW:
        if(mapToRender && mapToRender->map)
        {
            activeGraphRenderer->renderTopoMap(*mapToRender, placeManager);

            shouldAdjustCamera = showingNewMap;
            showingNewMap      = false;
            boundary           = activeGraphRenderer->calculateRenderedBoundary(*mapToRender, placeManager);
        }
        break;

    case HYPOTHESIS_TREE:
        drawHypothesisTree();

        shouldAdjustCamera = showingNewTree;
        showingNewTree     = false;
        boundary           = treeRenderer->getRenderedBoundary();
        break;
    }

    boundaryCenter.x = (boundary.bottomLeft.x + boundary.topRight.x) / 2;
    boundaryCenter.y = (boundary.bottomLeft.y + boundary.topRight.y) / 2;

    if(shouldAdjustCamera)
    {
        setCameraFocalPoint(boundaryCenter);
        setViewRegion(boundary.width()*1.25f, boundary.height()*1.25f);
    }
}


void GlobalTopoDisplayWidget::drawHypothesisTree(void)
{
    std::vector<preview_cell_contents_t> previewContents = mapPreviewer->getActiveCellsContents();

    for(auto previewIt = previewContents.begin(), previewEnd = previewContents.end(); previewIt != previewEnd; ++previewIt)
    {
        treeRenderer->highlightHypothesis(previewIt->mapId, previewIt->borderColor);
    }

    treeRenderer->highlightHypothesis(bestTopoMap.id, params.bestNodeColor);
    treeRenderer->renderTree();
    if(treeOfMaps)
    {
        mapPreviewer->render(*treeOfMaps, placeManager);
    }
}


void GlobalTopoDisplayWidget::mouseMotion(wxMouseEvent& event)
{
    wxSize size = GetSize();

    // Only change the highlights if the mouse is inside the hypothesis tree part of the widget
    if((viewToShow == HYPOTHESIS_TREE) && (event.GetX() < size.GetWidth()*HYP_TREE_RATIO))
    {
        // Remember -- Y is the Top here. Silly left-handed coordinate systems
        highlightHoverNode(nearestTreeNode(Point<uint16_t>(event.GetX(), size.GetHeight()-event.GetY())));
    }

    event.Skip();
}


void GlobalTopoDisplayWidget::mouseLeftDown(wxMouseEvent& event)
{
    event.Skip();
}


void GlobalTopoDisplayWidget::mouseLeftUp(wxMouseEvent& event)
{
    event.Skip();
}


void GlobalTopoDisplayWidget::mouseRightUp(wxMouseEvent& event)
{
    wxSize size = GetSize();

    // If the right button goes up in the preview area, determine the cell in which
    // the clicked occurred and clear it out
    if((viewToShow == HYPOTHESIS_TREE) && (event.GetX() > size.GetWidth()*HYP_TREE_RATIO))
    {
        grid_cell_t             clickCell = mapPreviewer->getCellUnderCursor(Point<int>(event.GetX(), size.GetHeight()-event.GetY()));
        preview_cell_contents_t contents  = mapPreviewer->getContents(clickCell);

        if(contents.mapId >= 0)
        {
            treeRenderer->cancelHighlight(contents.mapId);
        }

        mapPreviewer->clearCell(clickCell);
    }
    else if((viewToShow == HYPOTHESIS_TREE) && (event.GetX() < size.GetWidth()*HYP_TREE_RATIO))
    {
        previewNode(nearestTreeNode(Point<uint16_t>(event.GetX(), size.GetHeight()-event.GetY())));
    }

    event.Skip();
}


hssh::Id GlobalTopoDisplayWidget::nearestTreeNode(const Point<uint16_t>& mousePoint)
{
    auto pixel = convert_screen_to_world_coordinates(mousePoint, getCameraPosition());
    return treeRenderer->nodeClosestToPoint(pixel);
}


void GlobalTopoDisplayWidget::previewNode(hssh::Id nodeId)
{
    grid_cell_t previewCell = mapPreviewer->addMap(nodeId);

    if((previewCell.column >= 0) && (previewCell.row >= 0))
    {
        preview_cell_contents_t contents = mapPreviewer->getContents(previewCell);
        treeRenderer->highlightHypothesis(nodeId, contents.borderColor);
    }
}


void GlobalTopoDisplayWidget::highlightHoverNode(hssh::Id nodeId)
{
    if(nodeId != hoverId)
    {
        treeRenderer->cancelHighlight(hoverId);
        treeRenderer->highlightHypothesis(nodeId, params.hoverNodeColor);

        grid_cell_t mapCell(0, 0);
        mapPreviewer->setMap(nodeId, mapCell);

        hoverId = nodeId;
    }
}

} // namespace ui
} // namespace vulcan

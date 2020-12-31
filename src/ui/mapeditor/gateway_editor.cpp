/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gateway_editor.cpp
 * \author   Collin Johnson
 *
 * Definition of GatewayEditor.
 */

#include "ui/mapeditor/gateway_editor.h"
#include "hssh/local_topological/area_detection/gateways/gateway_utils.h"
#include "utils/algorithm_ext.h"
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/as_array.hpp>
#include <boost/range/iterator_range.hpp>

namespace vulcan
{
namespace ui
{

GatewayEditor::GatewayEditor(const GridBasedDisplayWidget* gridWidget)
: skeletonSelector_(gridWidget)
, gatewaySelector_(gridWidget)
{
}


void GatewayEditor::setSkeleton(hssh::VoronoiSkeletonGrid skeleton)
{
    skeleton_ = std::move(skeleton);
    setObjectSelectorCells();
}


void GatewayEditor::addGateways(const std::vector<hssh::Gateway>& gateways)
{
    boost::push_back(constructed_, boost::as_array(gateways));
}


void GatewayEditor::clearGateways(void)
{
    constructed_.clear();
    hover_.reset();
    selected_.reset();
}


GLEventStatus GatewayEditor::handleMouseMoved(const GLMouseEvent& event)
{
    // In the deleting step, gateways are selected
    if (state_ == State::deleting) {
        gatewaySelector_.handleMouseMoved(event);
    }
    // Otherwise grid cells are selected
    else {
        skeletonSelector_.handleMouseMoved(event);
    }

    setState(event);
    processEvent(event, EventType::moved);

    // Events need to be captured if selecting, otherwise the background will move around while the mouse is
    // being dragged
    return (state_ == State::selecting) ? GLEventStatus::capture : GLEventStatus::passthrough;
}


GLEventStatus GatewayEditor::handleRightMouseDown(const GLMouseEvent& event)
{
    setState(event);
    processEvent(event, EventType::down);

    // Let clicks propagate always
    return GLEventStatus::passthrough;
}


GLEventStatus GatewayEditor::handleRightMouseUp(const GLMouseEvent& event)
{
    setState(event);
    processEvent(event, EventType::up);

    // Let clicks propagate always
    return GLEventStatus::passthrough;
}


void GatewayEditor::setObjectSelectorCells(void)
{
    std::map<Point<int>, hssh::cell_t> selectorCells;

    // Go through the grid and map every cell to itself
    for (auto cell : boost::make_iterator_range(skeleton_.beginSkeletonCells(), skeleton_.endSkeletonCells())) {
        if (skeleton_.getClassification(cell.x, cell.y) & hssh::SKELETON_CELL_REDUCED_SKELETON) {
            selectorCells[cell] = cell;
        }
    }

    skeletonSelector_.setObjects(std::move(selectorCells));
}


void GatewayEditor::setGatewaySelectorCells(void)
{
    std::map<Point<int>, hssh::Gateway> selectorGateways;

    for (auto& gwy : constructed_) {
        selectorGateways[gwy.skeletonCell()] = gwy;
    }

    gatewaySelector_.setObjects(std::move(selectorGateways));
}


void GatewayEditor::setState(const GLMouseEvent& event)
{
    // Selecting takes precedence over deleting
    if (event.ctrlIsDown) {
        selected_ = boost::none;
        state_ = State::selecting;
    } else if (event.shiftIsDown) {
        hover_ = boost::none;

        // When switching into delete mode, set the gateways that will be selected amongst for deleting
        if (state_ != State::deleting) {
            setGatewaySelectorCells();
        }

        state_ = State::deleting;
    } else {
        selected_ = boost::none;
        state_ = State::hovering;
    }
}


void GatewayEditor::processEvent(const GLMouseEvent& event, EventType type)
{
    switch (state_) {
    case State::hovering:
        processHoverEvent(event, type);
        break;

    case State::selecting:
        processSelectingEvent(event, type);
        break;

    case State::deleting:
        processDeletingEvent(event, type);
        break;
    }
}


void GatewayEditor::processHoverEvent(const GLMouseEvent& event, EventType type)
{
    // The hover gateway changes when the mouse moves or the click button is released (for adding)
    if (type == EventType::down) {
        return;
    }

    switch (type) {
    case EventType::moved: {
        auto cell = skeletonSelector_.hoverObject();

        // If there's a valid hover cell, then we can create a gateway for that cell
        if (cell) {
            hoverNormal_ = gateway_normal_from_source_cells(*cell, skeleton_);
            hover_ = createGatewayAtCell(*cell, hoverNormal_);
        }
        // Otherwise, there isn't a hover state at the moment
        else {
            hover_ = boost::none;
        }
    } break;

    case EventType::up:
        if (hover_) {
            constructed_.push_back(*hover_);
            hover_.reset();   // clear the hover gateway so a double click doesn't add multiple gateways
        }
        break;

    case EventType::down:
        // Do nothing
        break;
    }
}


void GatewayEditor::processSelectingEvent(const GLMouseEvent& event, EventType type)
{
    // Can only select if a hover gateway was found first
    if (!hover_) {
        std::cerr << "ERROR::GatewayEditor: Can't select a gateway if a hover wasn't specified first.\n";
        return;
    }

    // If the mouse was clicked down here, then need to specify the initial click location for
    // determining where the selected gateway goes
    if (type == EventType::down) {
        selectedClickPoint_ = event.glCoords;
    }

    selected_ =
      createGatewayAtCell(hover_->skeletonCell(), angle_to_point(selectedClickPoint_, event.glCoords) + hoverNormal_);

    if ((type == EventType::up) && selected_) {
        constructed_.push_back(*selected_);
        hover_.reset();
        selected_.reset();   // clear out the selected so a double-click doesn't create a new gateway
    }
}


void GatewayEditor::processDeletingEvent(const GLMouseEvent& event, EventType type)
{
    // Only care about moved or up event
    if (type == EventType::down) {
        return;
    }

    auto gwy = gatewaySelector_.hoverObject();

    if (type == EventType::moved) {
        selected_ = gwy;
        hover_ = boost::none;
    } else if ((type == EventType::up) && selected_) {
        utils::erase_remove(constructed_, *selected_);
    }
}


boost::optional<hssh::Gateway> GatewayEditor::createGatewayAtCell(hssh::cell_t cell, double normal)
{
    auto gateway = create_gateway_at_cell(cell, normal, ++nextId_, skeleton_, 10.0);

    if (gateway) {
        return gateway;
    }

    // If the gateway couldn't be created, then search a range of angles to try and find an alternative
    for (float rotation = 0; rotation <= M_PI_4; rotation += M_PI / 180.0f) {
        auto posGateway = create_gateway_at_cell(cell, normal + rotation, ++nextId_, skeleton_);
        if (posGateway) {
            return posGateway;
        }

        auto negGateway = create_gateway_at_cell(cell, normal - rotation, ++nextId_, skeleton_);
        if (negGateway) {
            return negGateway;
        }
    }

    return boost::none;
}

}   // namespace ui
}   // namespace vulcan

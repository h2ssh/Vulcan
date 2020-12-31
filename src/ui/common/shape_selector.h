/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     shape_selector.h
 * \author   Collin Johnson
 *
 * Definition of ShapeSelector.
 */

#ifndef UI_COMMON_SHAPE_SELECTOR_H
#define UI_COMMON_SHAPE_SELECTOR_H

#include "ui/common/gl_event.h"
#include "ui/common/object_selector_event_handler.h"
#include <vector>

namespace vulcan
{
namespace ui
{

enum class ShapeSelectionButton
{
    kLeft,
    kRight
};

/**
 * ShapeSelector is a GLMouseHandler that determines which shape the mouse has selected/is hovering
 * over at the moment. The collection of Shapes is provided to the handler_ and it returns the index
 * of the currently selected shape.
 *
 * To use the ShapeSelector, you create an instance using the type of shape it is supposed to support.
 * The Shape is required to have a contains(Point<float>) method. The button used to select the
 * shapes is provided in the constructor. Once creates, the collection of shapes from which it is selecting
 * can be changed using the setShapes() method. The event handler_ is registered via the setHandler() method.
 *
 * The events triggered by the shape selector are:
 *
 *   - shapeSelected(int)  : the selection button was clicked on the shape with the provided index
 *   - shapeEntered(int)   : the mouse entered the shape with the provided index
 *   - shapeExited(int)    : the mouse left the shape with the provided index
 */
template <class Shape>
class ShapeSelector : public GLMouseHandler
{
public:
    /**
     * Constructor for ShapeSelector.
     *
     * \param    selectionButton         Button to trigger selections
     */
    ShapeSelector(ShapeSelectionButton selectionButton)
    : selectWithLeft_(selectionButton == ShapeSelectionButton::kLeft)
    , lastSelection_(0)
    , handler_(nullptr)
    {
    }

    /**
     * setShapes sets the shapes to be selected from.
     *
     * A copy of shapes is stored by the selector to help avoid weird crashes if the collection of shapes is updated
     * outside the event loop of the UI.
     */
    void setShapes(const std::vector<Shape>& shapes)
    {
        shapes_ = shapes;
        lastSelection_ = shapes_.size();
    }

    /**
     * setHandler sets the event handler for the events being generated. Currently, only a single event handler is
     * supported. The shapes and the event handler are separate, so the shapes can be changed while the event handler
     * remains the same.
     *
     * To stop generating events, set the handler to nullptr.
     */
    void setHandler(ObjectSelectorEventHandler<Shape>* handler) { handler_ = handler; }

    // GLMouseHandler interface
    virtual GLEventStatus handleLeftMouseUp(const GLMouseEvent& event)
    {
        // Bypass any checks if there isn't a handler_ for the event.
        if (!handler_) {
            return GLEventStatus::passthrough;
        }

        if (selectWithLeft_) {
            std::size_t selection = findSelectedShape(event, true);
            if (selection < shapes_.size()) {
                handler_->objectSelected(shapes_[selection]);
            }
            return GLEventStatus::capture;
        }

        return GLEventStatus::passthrough;
    }

    virtual GLEventStatus handleRightMouseUp(const GLMouseEvent& event)
    {
        // Bypass any checks if there isn't a handler_ for the event.
        if (!handler_) {
            return GLEventStatus::passthrough;
        }

        if (!selectWithLeft_) {
            std::size_t selection = findSelectedShape(event, true);
            if (selection < shapes_.size()) {
                handler_->objectSelected(shapes_[selection]);
            }
            return GLEventStatus::capture;
        }

        return GLEventStatus::passthrough;
    }

    virtual GLEventStatus handleMouseMoved(const GLMouseEvent& event)
    {
        // Bypass any checks if there isn't a handler_ for the event.
        if (!handler_) {
            return GLEventStatus::passthrough;
        }

        std::size_t selection = findSelectedShape(event, false);

        // If the selected shape has changed, then some sort of event must have occurred
        if (selection != lastSelection_) {
            // Was a shape previous selected?
            if (lastSelection_ < shapes_.size()) {
                handler_->objectExited(shapes_[selection]);
            }

            // Has the mouse moved onto a new shape?
            if (selection < shapes_.size()) {
                handler_->objectEntered(shapes_[selection]);
            }

            lastSelection_ = selection;
        }

        return GLEventStatus::passthrough;
    }

private:
    bool selectWithLeft_;
    std::vector<Shape> shapes_;
    std::size_t lastSelection_;
    ObjectSelectorEventHandler<Shape>* handler_;

    std::size_t findSelectedShape(const GLMouseEvent& event, bool mouseUp)
    {
        // The area of a shape is usually much larger than a pixel, which means that most of the time, the mouse will
        // remain inside the same shape as the mouse is dragged across the screen. Short-circuit the full search by
        // opportunistically checking that the mouse hasn't changed shapes
        if (lastSelection_ < shapes_.size() && shapes_[lastSelection_].contains(event.glCoords)) {
            return lastSelection_;
        }

        for (std::size_t n = 0; n < shapes_.size(); ++n) {
            if (shapes_[n].contains(event.glCoords)) {
                return n;
            }
        }

        return shapes_.size();
    }
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMMON_SHAPE_SELECTOR_H

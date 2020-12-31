/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gl_event.h
 * \author   Collin Johnson
 *
 * Declaration of event handlers for OpenGL-based widgets:
 *
 *   - GLMouseEvent type and GLMouseHandler interface to be used to handle mouse events
 *   - GLKeyboardHandler interface to be used to handle keyboard events
 *
 * The wxKeyEvent is not interpreted because there are obviously a vast number of key codes, so reinterpreting that
 * would be a waste of time -- leaky abstraction!
 */

#ifndef UI_COMMON_GL_MOUSE_H
#define UI_COMMON_GL_MOUSE_H

#include "core/point.h"
#include <wx/event.h>

namespace vulcan
{
namespace ui
{

/**
 * GLEventStatus defines the possible return types for an handler method. An event handler has
 * two options: Capture or Passthrough. For Captured events, any handlers below it on the stack won't see the
 * event. For Passthroughed events, the next handler on the stack will also receive the event.
 */
enum class GLEventStatus
{
    capture,
    passthrough
};

/**
 * GLMouseEvent defines the properties of a mouse event. Both the screen and converted OpenGL
 * coordinates are provided, along with flags indicating the state of the buttons. If the mouse wheel
 * has moved, the wheelRotationDirection will indicate which direction it moved.
 *
 * NOTE: wxWidgets has the top left corner as y = 0. These screen coordinates are properly converted
 *       into a normal right-handed coordinate system with the bottom left corner as (0,0)*
 */
struct GLMouseEvent
{
    Point<int> screenCoords;   ///< Coordinate of the event in pixels on the screen
    Point<float> glCoords;     ///< Coordinate of the event in the OpenGL camera coordinates

    int wheelRotationDirection;   ///< Direction the mouse wheel rotated: 1 = up, -1 = down, 0 = no rotation

    // Which mouse buttons are down?
    bool leftIsDown;
    bool rightIsDown;

    // Which keys are down?
    bool ctrlIsDown;
    bool shiftIsDown;
    bool altIsDown;
};

/**
 * GLMouseHandler is an interface for receiving mouse events from an OpenGLWidget. Mouse handlers for an OpenGLWidget
 * use a stack, so handlers can be pushed or popped. A handler has the option of capturing a mouse event or passing
 * it on. If captured, no other handlers further down the stack will see it. Passing the event on will allow a number
 * of mouse handlers to all function together.
 *
 * By default, all mouse events simply passthrough, so only desired mouse events need to be overridden.
 */
class GLMouseHandler
{
public:
    virtual ~GLMouseHandler(void) { }

    /**
     * handleLeftMouseDown is called when the left mouse button is pressed.
     *
     * \param    event           Event generated
     * \return   Status indicating whether the event has been captured or should be passed on.
     */
    virtual GLEventStatus handleLeftMouseDown(const GLMouseEvent& event) { return GLEventStatus::passthrough; }

    /**
     * handleLeftMouseUp is called when the left mouse button is released.
     *
     * \param    event           Event generated
     * \return   Status indicating whether the event has been captured or should be passed on.
     */
    virtual GLEventStatus handleLeftMouseUp(const GLMouseEvent& event) { return GLEventStatus::passthrough; }

    /**
     * handleRightMouseDown is called when the right mouse button is pressed.
     *
     * \param    event           Event generated
     * \return   Status indicating whether the event has been captured or should be passed on.
     */
    virtual GLEventStatus handleRightMouseDown(const GLMouseEvent& event) { return GLEventStatus::passthrough; }

    /**
     * handleRightMouseUp is called when the left mouse button is released.
     *
     * \param    event           Event generated
     * \return   Status indicating whether the event has been captured or should be passed on.
     */
    virtual GLEventStatus handleRightMouseUp(const GLMouseEvent& event) { return GLEventStatus::passthrough; }

    /**
     * handleMouseMoved is called when the mouse is moved.
     *
     * \param    event           Event generated
     * \return   Status indicating whether the event has been captured or should be passed on.
     */
    virtual GLEventStatus handleMouseMoved(const GLMouseEvent& event) { return GLEventStatus::passthrough; }

    /**
     * handleMouseWheel is called when the mouse wheel is rotated.
     *
     * \param    event           Event generated
     * \return   Status indicating whether the event has been captured or should be passed on.
     */
    virtual GLEventStatus handleMouseWheel(const GLMouseEvent& event) { return GLEventStatus::passthrough; }
};


/**
 * GLKeyboardHandler is an interface for classes that wish to receive keyboard events from an OpenGLWidget.
 *
 * By default, all keyboard events are ignored.
 */
class GLKeyboardHandler
{
public:
    /**
     * keyPressed is triggered when a key is pressed.
     *
     * \param    event           Key that was pressed
     */
    virtual GLEventStatus keyPressed(wxKeyEvent& event) { return GLEventStatus::passthrough; }

    /**
     * keyReleased is triggered when a key is released.
     *
     * \param    event           Key that was released
     */
    virtual GLEventStatus keyReleased(wxKeyEvent& event) { return GLEventStatus::passthrough; }
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMMON_GL_MOUSE_H

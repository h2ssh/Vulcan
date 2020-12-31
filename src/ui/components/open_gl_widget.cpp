/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     open_gl_widget.cpp
* \author   Collin Johnson
*
* Implmentation of OpenGLWidget.
*/

#include "ui/components/open_gl_widget.h"
#include "ui/common/gl_utilities.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <cassert>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(OpenGLWidget, wxGLCanvas)
    EVT_MOUSEWHEEL(OpenGLWidget::mouseWheel)
    EVT_MOTION(OpenGLWidget::mouseMotion)
    EVT_LEFT_DOWN(OpenGLWidget::mouseLeftDown)
    EVT_LEFT_UP(OpenGLWidget::mouseLeftUp)
    EVT_RIGHT_DOWN(OpenGLWidget::mouseRightDown)
    EVT_RIGHT_UP(OpenGLWidget::mouseRightUp)
    EVT_KEY_DOWN(OpenGLWidget::keyDown)
    EVT_KEY_UP(OpenGLWidget::keyUp)
    EVT_PAINT(OpenGLWidget::paint)
    EVT_SIZE(OpenGLWidget::resize)
    EVT_ERASE_BACKGROUND(OpenGLWidget::erase)
END_EVENT_TABLE()

static int glAttrib[] = { WX_GL_RGBA, WX_GL_DEPTH_SIZE, 16, WX_GL_DOUBLEBUFFER, 0};

OpenGLWidget::OpenGLWidget(wxWindow* parent,
                           wxWindowID id,
                           const wxPoint& pos,
                           const wxSize& size,
                           long style,
                           const wxString& name,
                           const wxPalette& palette)
: wxGLCanvas(parent, id, glAttrib, pos, size, style, name, palette)
, minWidth(0.0f)
, maxWidth(10.0f)
, minHeight(0.0f)
, maxHeight(10.0f)
, cameraController(new GLCameraController(camera))
{
    setCameraPosition(math::SphericalPoint(5.0f, M_PI/2.0f, 0.0f));
    cameraController->setRegionSize(size.GetWidth(), size.GetHeight());

    mouseHandlers.push_back(cameraController.get());
    keyboardHandlers.push_back(cameraController.get());
}


OpenGLWidget::~OpenGLWidget(void)
{
}


void OpenGLWidget::setMinViewDimensions(float minWidth, float minHeight)
{
    this->minWidth  = std::max(0.0f, minWidth);
    this->minHeight = std::max(0.0f, minHeight);
}


void OpenGLWidget::setMaxViewDimensions(float maxWidth, float maxHeight)
{
    this->maxWidth  = std::max(0.0f, maxWidth);
    this->maxHeight = std::max(0.0f, maxHeight);
}


void OpenGLWidget::setViewRegion(float width, float height)
{
    camera.setViewRegion(width, height);
}


void OpenGLWidget::setCameraFocalPoint(const math::Point3D<float>& focalPoint)
{
    this->focalPoint = focalPoint;
    camera.setFocalPoint(focalPoint);
}


void OpenGLWidget::setCameraFocalPoint(const Point<float>& focalPoint)
{
    this->focalPoint.x = focalPoint.x;
    this->focalPoint.y = focalPoint.y;
    this->focalPoint.z = 0.0f;
    camera.setFocalPoint(focalPoint);
}


void OpenGLWidget::setCameraPosition(const math::SphericalPoint& cameraPosition)
{
    camera.setPosition(cameraPosition);
}



void OpenGLWidget::getViewportBoundary(Point<int>& bottomLeft, int& width, int& height)
{
    wxSize size = GetSize();

    bottomLeft.x = 0;
    bottomLeft.y = 0;
    width        = size.GetWidth();
    height       = size.GetHeight();
}


void OpenGLWidget::pushMouseHandler(GLMouseHandler* handler)
{
    if(handler)
    {
        mouseHandlers.push_front(handler);
    }
}


void OpenGLWidget::removeMouseHandler(GLMouseHandler* handler)
{
    mouseHandlers.erase(std::remove(mouseHandlers.begin(), mouseHandlers.end(), handler), mouseHandlers.end());
}


void OpenGLWidget::pushKeyboardHandler(GLKeyboardHandler* handler)
{
    if(handler)
    {
        keyboardHandlers.push_front(handler);
    }
}


void OpenGLWidget::removeKeyboardHandler(GLKeyboardHandler* handler)
{
    keyboardHandlers.erase(std::remove(keyboardHandlers.begin(), keyboardHandlers.end(), handler),
        keyboardHandlers.end());
}


void OpenGLWidget::mouseWheel(wxMouseEvent& event)
{
    GLMouseEvent glEvent = convertWxEventToGlEvent(event);

    for(auto handler : mouseHandlers)
    {
        // As soon as the event is captured, then exit the method because the event shouldn't be propagated
        // and it shouldn't be skipped
        if(handler->handleMouseWheel(glEvent) == GLEventStatus::capture)
        {
            return;
        }
    }

    // If we make it to here, then no handler captured the event, so pass it on in case anything else needs to use it
    event.Skip();
}


void OpenGLWidget::mouseMotion(wxMouseEvent& event)
{
    GLMouseEvent glEvent = convertWxEventToGlEvent(event);

    for(auto handler : mouseHandlers)
    {
        // As soon as the event is captured, then exit the method because the event shouldn't be propagated
        // and it shouldn't be skipped
        if(handler->handleMouseMoved(glEvent) == GLEventStatus::capture)
        {
            return;
        }
    }

    // If we make it to here, then no handler captured the event, so pass it on in case anything else needs to use it
    event.Skip();
}


void OpenGLWidget::mouseLeftDown(wxMouseEvent& event)
{
    GLMouseEvent glEvent = convertWxEventToGlEvent(event);

    for(auto handler : mouseHandlers)
    {
        // As soon as the event is captured, then exit the method because the event shouldn't be propagated
        // and it shouldn't be skipped
        if(handler->handleLeftMouseDown(glEvent) == GLEventStatus::capture)
        {
            return;
        }
    }

    // If we make it to here, then no handler captured the event, so pass it on in case anything else needs to use it
    event.Skip();
}


void OpenGLWidget::mouseLeftUp(wxMouseEvent& event)
{
    GLMouseEvent glEvent = convertWxEventToGlEvent(event);

    for(auto handler : mouseHandlers)
    {
        // As soon as the event is captured, then exit the method because the event shouldn't be propagated
        // and it shouldn't be skipped
        if(handler->handleLeftMouseUp(glEvent) == GLEventStatus::capture)
        {
            return;
        }
    }

    // If we make it to here, then no handler captured the event, so pass it on in case anything else needs to use it
    event.Skip();
}


void OpenGLWidget::mouseRightDown(wxMouseEvent& event)
{
    GLMouseEvent glEvent = convertWxEventToGlEvent(event);

    for(auto handler : mouseHandlers)
    {
        // As soon as the event is captured, then exit the method because the event shouldn't be propagated
        // and it shouldn't be skipped
        if(handler->handleRightMouseDown(glEvent) == GLEventStatus::capture)
        {
            return;
        }
    }

    // If we make it to here, then no handler captured the event, so pass it on in case anything else needs to use it
    event.Skip();
}


void OpenGLWidget::mouseRightUp(wxMouseEvent& event)
{
    GLMouseEvent glEvent = convertWxEventToGlEvent(event);

    for(auto handler : mouseHandlers)
    {
        // As soon as the event is captured, then exit the method because the event shouldn't be propagated
        // and it shouldn't be skipped
        if(handler->handleRightMouseUp(glEvent) == GLEventStatus::capture)
        {
            return;
        }
    }

    // If we make it to here, then no handler captured the event, so pass it on in case anything else needs to use it
    event.Skip();
}


void OpenGLWidget::keyDown(wxKeyEvent& event)
{
    for(auto handler : keyboardHandlers)
    {
        if(handler->keyPressed(event) == GLEventStatus::capture)
        {
            return;
        }
    }

    // No handler consumed the keyboard event
    event.Skip();
}


void OpenGLWidget::keyUp(wxKeyEvent& event)
{
    for(auto handler : keyboardHandlers)
    {
        if(handler->keyReleased(event) == GLEventStatus::capture)
        {
            return;
        }
    }

    // No handler consumed the keyboard event
    event.Skip();
}


void OpenGLWidget::resize(wxSizeEvent& event)
{
    wxSize size = GetSize();

    cameraController->setRegionSize(size.GetWidth(), size.GetHeight());
}


void OpenGLWidget::paint(wxPaintEvent& event)
{
    // If not shown or context not set yet, then don't attempt to draw!
    if(!IsShownOnScreen() || !context)
    {
        return;
    }

    // Set our GLContext for the drawing operation
    wxGLCanvas::SetCurrent(*context);
    wxPaintDC dc(this);

    setup_opengl_context();

    Point<int> viewStart;
    int              width  = 0;
    int              height = 0;
    getViewportBoundary(viewStart, width, height);

    glViewport(viewStart.x, viewStart.y, width, height);

    glClearColor(1.0, 1.0, 1.0, 1.0);  // clear to white screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    camera.setupCamera(width, height);
    renderWidget();

    glFlush();
    SwapBuffers();
}


void OpenGLWidget::erase(wxEraseEvent& event)
{
}


GLMouseEvent OpenGLWidget::convertWxEventToGlEvent(const wxMouseEvent& wx)
{
    GLMouseEvent gl;

    wxSize size = GetSize();

    gl.screenCoords = Point<int>(wx.GetX(),  size.GetHeight() - wx.GetY());
    gl.glCoords     = convert_screen_to_world_coordinates(gl.screenCoords, getCameraPosition());

    gl.wheelRotationDirection = wx.GetWheelRotation();

    gl.leftIsDown  = wx.LeftIsDown();
    gl.rightIsDown = wx.RightIsDown();
    gl.ctrlIsDown  = wx.ControlDown();
    gl.altIsDown   = wx.AltDown();
    gl.shiftIsDown = wx.ShiftDown();

    return gl;
}

} // namespace ui
} // namespace vulcan

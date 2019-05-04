/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gl_camera_controller.h
* \author   Collin Johnson
* 
* Declaration of GLCameraController.
*/

#ifndef UI_COMMON_GL_CAMERA_CONTROLLER_H
#define UI_COMMON_GL_CAMERA_CONTROLLER_H

#include <ui/common/gl_event.h>
#include <wx/event.h>

class wxKeyEvent;

namespace vulcan
{
namespace ui
{
    
class GLCamera;

/**
* GLCameraController is a mouse and keyboard event handler for allowing user control of the GLCamera
* associated with an OpenGLWidget.
* 
*   - left button + mouse motion  = pan the camera
*   - right button + mouse motion = tilt and rotate the camera
*   - mouse wheel                 = zoom in/out
*
* The following keyboard shortcuts are used:
*
*   - arrow keys        = pan the camera
*   - ctrl + arrow keys = zoom in/out
*   - alt  + arrow keys = rotate/tilt the camera
*/
class GLCameraController : public GLMouseHandler, 
                           public GLKeyboardHandler
{
public:
    
    /**
    * Constructor for GLCameraController.
    * 
    * \param    camera          Camera to be controlled
    */
    GLCameraController(GLCamera& camera);
    
    /**
    * setRegionSize sets the size of the viewing region in which the camera is displaying.
    * 
    * \param    width       Width of the region in screen pixels
    * \param    height      Height of the region in screen pixels
    */
    void setRegionSize(int width, int height);
    
    // Turn on/off various types of control for the camera
    void setPanning(bool enable)  { canPan = enable; }
    void setZooming(bool enable)  { canZoom = enable; }
    void setTilting(bool enable)  { canTilt = enable; }
    void setRotating(bool enable) { canRotate = enable; }
    
    // GLMouseHandler interface
    GLEventStatus handleLeftMouseDown(const GLMouseEvent& event) override;
    GLEventStatus handleLeftMouseUp(const GLMouseEvent& event) override;
    GLEventStatus handleRightMouseDown(const GLMouseEvent& event) override;
    GLEventStatus handleRightMouseUp(const GLMouseEvent& event) override;
    GLEventStatus handleMouseMoved(const GLMouseEvent& event) override;
    GLEventStatus handleMouseWheel(const GLMouseEvent& event) override;
    
    // GLKeyboardHandler interface
    GLEventStatus keyPressed(wxKeyEvent& event) override;
    
private:
    
    // Identify different camera control modes
    bool mouseEventIsZoom  (const GLMouseEvent& event) { return event.wheelRotationDirection != 0; }
    bool mouseEventIsPan   (const GLMouseEvent& event) { return event.leftIsDown  && !event.rightIsDown; }
    bool mouseEventIsTilt  (const GLMouseEvent& event) { return event.rightIsDown && event.leftIsDown; }
    bool mouseEventIsRotate(const GLMouseEvent& event) { return event.rightIsDown && !event.leftIsDown; }
    
    // Identify camera control modes from keyboard
    bool keyEventIsZoomIn     (const wxKeyEvent& event) { return event.ControlDown() && event.GetKeyCode() == WXK_UP;   }
    bool keyEventIsZoomOut    (const wxKeyEvent& event) { return event.ControlDown() && event.GetKeyCode() == WXK_DOWN; }
    bool keyEventIsPanUp      (const wxKeyEvent& event) { return event.GetKeyCode() == WXK_UP;    }
    bool keyEventIsPanDown    (const wxKeyEvent& event) { return event.GetKeyCode() == WXK_DOWN;  }
    bool keyEventIsPanLeft    (const wxKeyEvent& event) { return event.GetKeyCode() == WXK_LEFT;  }
    bool keyEventIsPanRight   (const wxKeyEvent& event) { return event.GetKeyCode() == WXK_RIGHT; }
    bool keyEventIsTiltUp     (const wxKeyEvent& event) { return event.AltDown() && event.GetKeyCode() == WXK_UP;    }
    bool keyEventIsTiltDown   (const wxKeyEvent& event) { return event.AltDown() && event.GetKeyCode() == WXK_DOWN;  }
    bool keyEventIsRotateLeft (const wxKeyEvent& event) { return event.AltDown() && event.GetKeyCode() == WXK_LEFT;  }
    bool keyEventIsRotateRight(const wxKeyEvent& event) { return event.AltDown() && event.GetKeyCode() == WXK_RIGHT; }
    
    // Perform the actual action
    void doMouseZoom  (int direction);
    void doMousePan   (int deltaX, int deltaY);
    void doMouseTilt  (int delta);
    void doMouseRotate(int delta);
    
    GLCamera&        camera;           // Camera instance being controlled
    GLMouseEvent lastEvent;
    
    int regionWidth;
    int regionHeight;
    
    bool canPan;
    bool canZoom;
    bool canTilt;
    bool canRotate;
};

}
}

#endif // UI_COMMON_GL_CAMERA_CONTROLLER_H

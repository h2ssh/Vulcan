/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     open_gl_widget.h
* \author   Collin Johnson
*
* Declaration of OpenGLWidget base class for handling mouse events
* and camera control.
*/

#ifndef UI_COMPONENTS_OPEN_GL_WIDGET_H
#define UI_COMPONENTS_OPEN_GL_WIDGET_H

#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <ui/common/gl_camera.h>
#include <ui/common/gl_camera_controller.h>
#include <deque>
#include <memory>

namespace vulcan
{
namespace ui
{
    
class GLMouseHandler;
class GLKeyboardHandler;

/**
* OpenGLWidget is a base class for display widgets using OpenGL. The widget handles mouse events
* used for controlling the position of the camera. The camera can be controlled with the following actions:
*
*   - left button + mouse motion  = pan the camera
*   - right button + mouse motion = tilt and rotate the camera
*   - mouse wheel                 = zoom in/out
*
* The following keyboard shortcuts are used:
*
*   - arrow keys         = pan the camera
*   - ctrl + arrow keys  = tilt and rotate the camera
*   - shift + arrow keys = zoom in/out
*
* The camera functionality can be fine-tuned using the public methods for turning on/off various capabilities.
*/
class OpenGLWidget : public wxGLCanvas
{
public:

    /**
    * Constructor for OpenGLWidget.
    */
    OpenGLWidget(wxWindow* parent,
                 wxWindowID id = wxID_ANY,
                 const wxPoint& pos = wxDefaultPosition,
                 const wxSize& size = wxDefaultSize,
                 long style = 0,
                 const wxString& name = wxString((const wxChar*)("GLCanvas")),
                 const wxPalette& palette = wxNullPalette);

    /**
    * Destructor for OpenGLWidget.
    */
    virtual ~OpenGLWidget(void);

    // Methods for controlling wx-specific state
    void setRenderContext(wxGLContext* context) { this->context = context; }

    // Methods for controlling properties of rendering
    /**
    * setMinViewDimensions sets the minimum dimensions of the viewing area displayed by the camera in the widget's coordinate
    * system, not the pixel coordinates. The minimum dimensions puts a limit on how close the camera will zoom.
    */
    void setMinViewDimensions(float minWidth, float minHeight);

    /**
    * setMaxViewDimensions sets the maximum dimensions of the viewing area displayed by the camera in the widget's coordinate
    * system, not the pixel coordinates. The maximum dimensions puts a limit on how far out the camera will zoom.
    */
    void setMaxViewDimensions(float maxWidth, float maxHeight);

    /**
    * setViewRegion sets the region to be rendered in the widget's coordinate system, not the pixel coordinates. The camera
    * will be zoomed accordingly to show the desired area.
    */
    void setViewRegion(float width, float height);

    /**
    * setCameraFocalPoint sets the position on which the camera should focus. This method can be used to set an initial camera position
    * and then the built-in camera control can handle the rest. If tracking some object, each time the position of the object
    * is encountered, the new focal point should be set.
    */
    void setCameraFocalPoint(const math::Point3D<float>& focalPoint);
    void setCameraFocalPoint(const Point<float>&   focalPoint);

    /**
    * setCameraPosition sets the position of the camera relative to the focal point in spherical coordinates.
    */
    void setCameraPosition(const math::SphericalPoint& cameraPosition);

    /**
    * getCameraFocalPoint retrieves the current position at which the camera is pointing.
    */
    math::Point3D<float> getCameraFocalPoint(void) const { return camera.getFocalPoint(); }

    /**
    * getCameraPosition retrieves the position of the camera in the coordinate frame. The position is where the camera is placed.
    * The focal point is where it is looking.
    */
    math::Point3D<float> getCameraPosition(void) { return camera.getAbsolutePosition(); }
    
    /**
    * getCameraRelativePosition retrieves the position of the camera relative to the focal point.
    */
    math::SphericalPoint getCameraRelativePosition(void) const { return camera.getRelativePosition(); }

    // Methods for enabling/disabling the different camera controls
    /**
    * pushMouseHandler pushes a new GLMouseHandler onto the mouse event stack. The pushed handler will be the first
    * to receive the mouse events, until another handler is pushed on as well.
    * 
    * \param    handler         Handler to be pushed onto the stack
    */
    void pushMouseHandler(GLMouseHandler* handler);
    
    /**
    * removeMouseHandler removes the associated handler from the stack of event handlers. If the requested handler to
    * removed isn't found, then nothing happens.
    */
    void removeMouseHandler(GLMouseHandler* handler);
    
    /**
    * pushKeyboardHandler pushes a new GLKeyboardHandler onto the key event stack. The pushed handler will be the first
    * to receive the keyboard events, until another handler is pushed on as well.
    * 
    * \param    handler         Handler to be pushed onto the stack
    */
    void pushKeyboardHandler(GLKeyboardHandler* handler);
    
    /**
    * removeKeyboardHandler removes the associated handler from the stack of event handlers. If the requested handler to
    * removed isn't found, then nothing happens.
    */
    void removeKeyboardHandler(GLKeyboardHandler* handler);
    
    void enablePanning(void)  { cameraController->setPanning(true);  }
    void disablePanning(void) { cameraController->setPanning(false); }

    void enableZooming(void)  { cameraController->setZooming(true);  }
    void disableZooming(void) { cameraController->setZooming(false); }

    void enableTilting(void)  { cameraController->setTilting(true);  }
    void disableTilting(void) { cameraController->setTilting(false); }

    void enableRotating(void)  { cameraController->setRotating(true);  }
    void disableRotating(void) { cameraController->setRotating(false); }

protected:

    /**
    * getViewportBoundary retrieves the pixel boundary of the viewport to be displayed.
    * By default, this boundary will be set to the widget size. If a subclass is doing
    * more custom drawing than simply filling a single viewport, the boundary returned
    * should be adjusted accordingly.
    */
    virtual void getViewportBoundary(Point<int>& bottomLeft, int& width, int& height);

    /**
    * renderWidget is called from the repaint event and is where the custom widget
    * drawing needs to take place.
    *
    * The renderWidget() method is called after the following steps have already
    * been taken:
    *
    *   - Setting the viewport
    *   - Setting the perspective
    *   - Setting the camera position
    *
    * renderWidget() is called with the camera matrix at the top of the stack and
    * the matrix mode in GL_MODELVIEW.
    *
    * After renderWidget() returns, the repaint event will call in order to display
    * the actual output:
    *
    *   - glFlush();
    *   - SwapBuffers();
    */
    virtual void renderWidget(void) = 0;

private:

    // Internal event handlers
    void mouseWheel    (wxMouseEvent& event);
    void mouseMotion   (wxMouseEvent& event);
    void mouseLeftDown (wxMouseEvent& event);
    void mouseLeftUp   (wxMouseEvent& event);
    void mouseRightDown(wxMouseEvent& event);
    void mouseRightUp  (wxMouseEvent& event);

    void keyDown(wxKeyEvent& event);
    void keyUp(wxKeyEvent& event);

    void resize(wxSizeEvent& event);
    void paint (wxPaintEvent& event);
    void erase (wxEraseEvent& event);
    
    GLMouseEvent convertWxEventToGlEvent(const wxMouseEvent& wx);

    wxMouseEvent lastEvent;
    
    std::deque<GLMouseHandler*> mouseHandlers;
    std::deque<GLKeyboardHandler*> keyboardHandlers;

    math::Point3D<float> focalPoint;

    float    scale;
    float    zoom;
    float    minWidth;
    float    maxWidth;
    float    minHeight;
    float    maxHeight;

    GLCamera camera;
    std::unique_ptr<GLCameraController> cameraController;

    wxGLContext* context;

    DECLARE_EVENT_TABLE()
};

}
}

#endif // UI_COMPONENTS_OPEN_GL_WIDGET_H

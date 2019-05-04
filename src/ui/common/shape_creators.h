/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     shape_creators.h
* \author   Collin Johnson
* 
* Declaration of various GLMouseHandlers capable of creating different types of shapes:
* 
*   - RectangleCreator
*   - PolygonCreator
*   - CircleCreator
*/

#ifndef UI_COMMON_SHAPE_CREATORS_H
#define UI_COMMON_SHAPE_CREATORS_H

#include <ui/common/gl_event.h>
#include <math/geometry/rectangle.h>

namespace vulcan
{
namespace ui
{

/**
* RectangleCreator can be used to create a rectangle using the mouse. The following are the steps
* and buttons for creating a rectangle:
* 
*   1) Call reset() to make sure all state is at the default
*   2) First left mouse click sets initial vertex.
*   3) Move mouse around to reshape the rectangle.
*   4) Hold down Ctrl to rotate the rectangle.
*   5) Releasing the left mouse button will finish the rectangle.
* 
* The rectangle is created in OpenGL coordinates, rather than screen coordinates.
*/
class RectangleCreator : public GLMouseHandler
{
public:
    
    /**
    * Constructor for RectangleCreator.
    */
    RectangleCreator(void);
    
    /**
    * reset changes the creator back to the initial state, where no vertices or rotations exist yet.
    */
    void reset(void);
    
    /**
    * isFinished checks to see if the rectangle creation is complete. The rectangle is finished after
    * the left mouse button is released.
    */
    bool isFinished(void) const { return rectIsDone; }
    
    math::Rectangle<float> getRectangle(void) const { return rect; }
    
    // GLMouseHandler interface
    virtual GLEventStatus handleLeftMouseDown(const GLMouseEvent& event);
    virtual GLEventStatus handleLeftMouseUp  (const GLMouseEvent& event);
    virtual GLEventStatus handleMouseMoved   (const GLMouseEvent& event);
    
private:
    
    void  createRectFromCurrentMouse(void);
    float calculateRotation         (const Point<float>& mousePosition);
    
    math::Rectangle<float> rect;                ///< Rectangle created based on current mouse + initial vertex + rotation
    Point<float>     initialVertex;       ///< Initial position clicked
    Point<float>     mouseVertex;         ///< Position of most recent mouse vertex
    
    Point<float> rotationPosition;        ///< Position of mouse when rotation mode started
    float              rotation;                ///< Radians of rotation created
    
    bool leftWasDown;       ///< Flag indicating if the left button has been clicked yet
    bool ctrlWasDown;       ///< Flag indicating if control was held down on the last mouse event
    bool rectIsDone;        ///< Flag indicating if the rectangle is completed, i.e. left mouse button was released
};

// TODO: PolygonCreator and CircleCreator

}
}

#endif // UI_COMMON_SHAPE_CREATORS_H

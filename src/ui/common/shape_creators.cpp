/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     shape_creators.cpp
 * \author   Collin Johnson
 *
 * Definition of RectangleCreator, PolygonCreator, CircleCreator.
 */

#include "ui/common/shape_creators.h"

namespace vulcan
{
namespace ui
{

RectangleCreator::RectangleCreator(void)
{
    reset();
}


void RectangleCreator::reset(void)
{
    leftWasDown = false;
    ctrlWasDown = false;
    rectIsDone = false;
    rotation = 0;
}


GLEventStatus RectangleCreator::handleLeftMouseDown(const GLMouseEvent& event)
{
    // Start the rectangle building by first capturing the left down event. This keeps the rectangle
    // from starting if the mouse enters the widget with the button already held down.
    if (!leftWasDown && !rectIsDone) {
        initialVertex = event.glCoords;
        leftWasDown = true;

        return GLEventStatus::capture;
    }

    // Ignore any non-left click events
    return GLEventStatus::passthrough;
}


GLEventStatus RectangleCreator::handleLeftMouseUp(const GLMouseEvent& event)
{
    //     rectIsDone  = true;
    leftWasDown = false;

    return GLEventStatus::passthrough;
}


GLEventStatus RectangleCreator::handleMouseMoved(const GLMouseEvent& event)
{
    if (event.leftIsDown && leftWasDown) {
        if (!event.ctrlIsDown) {
            mouseVertex = event.glCoords;
        } else if (!ctrlWasDown) {
            rotationPosition = event.glCoords;
            ctrlWasDown = true;
        } else   // event.ctrlIsDown && ctrlWasDown
        {
            rotation = calculateRotation(event.glCoords);
        }

        createRectFromCurrentMouse();
        return GLEventStatus::capture;
    }

    return GLEventStatus::passthrough;
}


void RectangleCreator::createRectFromCurrentMouse(void)
{
    float width = mouseVertex.x - initialVertex.x;
    float height = mouseVertex.y - initialVertex.y;

    // Decide the ordering of vertices based on the calculated width and height. The width and height
    // specify the quadrant of mouseVertex relative to initialVertex
    if (width > 0 && height > 0) {
        rect.bottomLeft = initialVertex;
        rect.bottomRight = initialVertex + rotate(Point<float>(width, 0), rotation);
        rect.topRight = initialVertex + rotate(Point<float>(width, height), rotation);
        rect.topLeft = initialVertex + rotate(Point<float>(0, height), rotation);
    } else if (width < 0 && height > 0) {
        rect.bottomRight = initialVertex;
        rect.bottomLeft = initialVertex + rotate(Point<float>(width, 0), rotation);
        rect.topLeft = initialVertex + rotate(Point<float>(width, height), rotation);
        rect.topRight = initialVertex + rotate(Point<float>(0, height), rotation);
    } else if (width > 0 && height < 0) {
        rect.topLeft = initialVertex;
        rect.topRight = initialVertex + rotate(Point<float>(width, 0), rotation);
        rect.bottomRight = initialVertex + rotate(Point<float>(width, height), rotation);
        rect.bottomLeft = initialVertex + rotate(Point<float>(0, height), rotation);
    } else   // width < 0 && height < 0
    {
        rect.topRight = initialVertex;
        rect.topLeft = initialVertex + rotate(Point<float>(width, 0), rotation);
        rect.bottomLeft = initialVertex + rotate(Point<float>(width, height), rotation);
        rect.bottomRight = initialVertex + rotate(Point<float>(0, height), rotation);
    }
}


float RectangleCreator::calculateRotation(const Point<float>& mousePosition)
{
    return angle_diff(angle_to_point(initialVertex, mousePosition), angle_to_point(initialVertex, rotationPosition));
}

}   // namespace ui
}   // namespace vulcan

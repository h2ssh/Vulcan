/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     object_selector_event_handler.h
 * \author   Collin Johnson
 *
 * Definition of the ObjectSelectorEventHandler interface.
 */

#ifndef UI_COMMON_OBJECT_SELECTOR_EVENT_HANDLER_H
#define UI_COMMON_OBJECT_SELECTOR_EVENT_HANDLER_H

namespace vulcan
{
namespace ui
{

/**
 * ObjectSelectorEventHandler is an interface for widgets that will be receiving events from a selection object like the
 * GridObjectSelector or ShapeSeletor.
 */
template <class Object>
class ObjectSelectorEventHandler
{
public:
    virtual ~ObjectSelectorEventHandler(void) { }

    /**
     * objectSelected is triggered when the user presses the selection button while the mouse is inside the object.
     */
    virtual void objectSelected(Object& object) = 0;

    /**
     * objectEntered is triggered when the mouse enters one of the objects.
     */
    virtual void objectEntered(Object& object) = 0;

    /**
     * objectExited is triggered when the mouse leaves its current object.
     */
    virtual void objectExited(Object& object) = 0;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMMON_OBJECT_SELECTOR_EVENT_HANDLER_H

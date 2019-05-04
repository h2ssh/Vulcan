/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gl_buffer_consumer.h
* \author   Collin Johnson
*
* Declaration of GLBufferConsumer interface.
*/

#ifndef UI_COMMON_GL_BUFFER_CONSUMER_H
#define UI_COMMON_GL_BUFFER_CONSUMER_H

namespace vulcan
{
class Image;

namespace ui
{

/**
* GLBufferConsumer is an interface for classes that wish to handle the pixels
* drawn to the screen in an OpenGLWidget.
*
* The pixels are passed to the consumer as a Image.
*/
class GLBufferConsumer
{
public:

    virtual ~GLBufferConsumer(void) { }

    /**
    * handleBuffer is called each time the OpenGLWidget is refreshed. The updated
    * pixels are stored in the Image.
    *
    * \param    buffer          Buffer containing the pixels
    */
    virtual void handleBuffer(const Image& buffer) = 0;
};

}
}

#endif // UI_COMMON_GL_BUFFER_CONSUMER_H

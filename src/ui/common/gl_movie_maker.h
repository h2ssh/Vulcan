/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gl_movie_maker.h
* \author   Collin Johnson
*
* Declaration of GLMovieMaker.
*/

#ifndef UI_COMMON_GL_MOVIE_MAKER_H
#define UI_COMMON_GL_MOVIE_MAKER_H

#include <ui/common/gl_buffer_consumer.h>

namespace vulcan
{
namespace ui
{

/**
* GLMovieMaker
*/
class GLMovieMaker : public GLBufferConsumer
{
public:

    // GLBufferConsumer interface
    virtual void handleBuffer(const Image& buffer);

private:

};

}
}

#endif // UI_COMMON_GL_MOVIE_MAKER_H

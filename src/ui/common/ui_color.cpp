/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gl_color.cpp
* \author   Collin Johnson
* 
* Implementation of GLColor.
*/

#include <ui/common/ui_color.h>
#include <boost/algorithm/clamp.hpp>
#include <GL/gl.h>
#include <cstdint>
#include <cstdlib>

namespace vulcan
{
namespace ui
{
    
float extract_number(const std::string& colorString, std::size_t& startPos);
    

GLColor::GLColor(int red, int green, int blue, int alpha)
: red_(boost::algorithm::clamp(red, 0, 255)     / 255.0f)
, green_(boost::algorithm::clamp(green, 0, 255) / 255.0f)
, blue_(boost::algorithm::clamp(blue, 0, 255)   / 255.0f)
, alpha_(boost::algorithm::clamp(alpha, 0, 255) / 255.0f)
{
}

GLColor::GLColor(double red, double green, double blue, double alpha)
: red_(boost::algorithm::clamp(red, 0.0f, 1.0f))
, green_(boost::algorithm::clamp(green, 0.0f, 1.0f))
, blue_(boost::algorithm::clamp(blue, 0.0f, 1.0f))
, alpha_(boost::algorithm::clamp(alpha, 0.0f, 1.0f))
{
}


GLColor::GLColor(const std::string& colorString)
{
    std::size_t numberIndex = 0;
    
    red_   = extract_number(colorString, numberIndex);
    green_ = extract_number(colorString, numberIndex);
    blue_  = extract_number(colorString, numberIndex);
    alpha_ = extract_number(colorString, numberIndex);
}


void GLColor::set(float alphaScale) const
{
    alphaScale = boost::algorithm::clamp(alphaScale, 0.0f, 1.0f);
    glColor4f(red_, green_, blue_, alpha_*alphaScale);
}


float extract_number(const std::string& colorString, size_t& startPos)
{
    const std::string numbers("0123456789");
    
    std::size_t numberStart = colorString.find_first_of(numbers, startPos);
    std::size_t numberEnd   = colorString.find_first_not_of(numbers, numberStart);
    
    int number = 0;
    
    if(numberStart != numberEnd)
    {
        number = atoi(colorString.substr(numberStart, numberEnd-numberStart).c_str());
    }
    
    startPos = numberEnd;

    number = boost::algorithm::clamp(number, 0, 255);
    
    return static_cast<float>(number)/255.0f;
}

}
}

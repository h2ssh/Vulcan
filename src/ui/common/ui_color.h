/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gl_color.h
* \author   Collin Johnson
* 
* Definition of GLColor.
*/

#ifndef UI_COMMON_GL_COLOR_H
#define UI_COMMON_GL_COLOR_H

#include <string>

namespace vulcan
{
namespace ui
{

/**
* GLColor defines a color to use for drawing something. The GLColor can be created explicitly or parsed
* from a string.
*
* The range of each element is [0,1] and corresponds to a color being set using glColor4f.
*/
class GLColor
{
public:
    
    explicit GLColor(int red, int green, int blue, int alpha);
    explicit GLColor(double red = 0.0, double green = 0.0, double blue = 0.0, double alpha = 1.0);
    
    /**
    * Constructor for GLColor.
    * 
    * Parses a GLColor from a string. The string contains the four values in the range [0,255].
    * The values can be separated by any non-numeric delimiter. For example, (0,0,0,0) or 255 255 255 255 are
    * both valid representations.
    */
    explicit GLColor(const std::string& colorString);
    
    // Operations
    /**
    * set sets the color using a call to glColor4f.
    * 
    * The optional parameter allows the alpha value to be scaled. Scaling the alpha is useful for drawing a 
    * dark border with a lighter interior.
    * 
    * \param    alphaScale      Factor by which to scale the alpha (optional, default = 1.0, range [0.0, 1.0].
    */
    void set(float alphaScale = 1.0f) const;
    
    // Accessors
    float red(void)   const { return red_; }
    float green(void) const { return green_; }
    float blue(void)  const { return blue_; }
    float alpha(void) const { return alpha_; }
    
    void alpha(float a) { alpha_ = a; }

private:
    
    float red_;
    float green_;
    float blue_;
    float alpha_;
};

template <class ostream>
ostream& operator<<(ostream& out, const GLColor& color)
{
    out<<'('<<color.red()<<','<<color.green()<<','<<color.blue()<<','<<color.alpha()<<')';
    return out;
}

} // namespace ui
} // namespace vulcan

#endif // UI_COMMON_GL_COLOR_H

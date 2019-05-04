/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <cmath>
#include <ui/common/gl_texture_helpers.h>

namespace vulcan
{
namespace ui
{

uint8_t* create_texture(uint16_t width, uint16_t height, uint8_t bytesPerTexel)
{
    uint16_t textureWidth  = round_to_power_of_two(width);
    uint16_t textureHeight = round_to_power_of_two(height);
    
    return new uint8_t[textureWidth * textureHeight * bytesPerTexel];
}


void free_textures(uint8_t** textures, uint8_t numTextures)
{
    for(int n = numTextures; --n >= 0;)
    {
        delete textures[n];
        textures[n] = 0;
    }
}


void initialize_texture(GLuint textureName, uint8_t* data, uint16_t width, uint16_t height, GLint internalFormat, GLenum displayFormat)
{
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    
    glBindTexture(GL_TEXTURE_2D, textureName);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, 0, displayFormat, GL_UNSIGNED_BYTE, data);
}


void initialize_texture_16(GLuint textureName, uint16_t* data, uint16_t width, uint16_t height, GLint internalFormat, GLenum displayFormat)
{
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    
    glBindTexture(GL_TEXTURE_2D, textureName);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, 0, displayFormat, GL_UNSIGNED_SHORT, data);
}


void activate_texture(GLuint textureName, GLuint textureNumber, GLuint textureMode)
{
    glActiveTexture(textureNumber);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, textureName);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, textureMode);
}


void disable_texture(GLuint textureNumber)
{
    glActiveTexture(textureNumber);
    glDisable(GL_TEXTURE_2D);
}


void set_sub_texture(uint8_t* texture, uint16_t width, uint16_t height, GLint format)
{
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, format, GL_UNSIGNED_BYTE, texture);
}


void set_sub_texture_16(uint16_t* texture, uint16_t width, uint16_t height, GLint format)
{
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, format, GL_UNSIGNED_SHORT, texture);
}


void draw_one_texture_on_rectangle(float bottomLeftX, float bottomLeftY, float rectWidth, float rectHeight, float textureWidth, float textureHeight)
{
    glBegin(GL_QUADS);  // grid is a series of quadrilaterals
    
    glTexCoord2f(0.0, 0.0);
    glVertex2f(bottomLeftX, bottomLeftY);
    
    glTexCoord2f(textureWidth, 0.0);
    glVertex2f(bottomLeftX+rectWidth, bottomLeftY);
    
    glTexCoord2f(textureWidth, textureHeight);
    glVertex2f(bottomLeftX+rectWidth, bottomLeftY+rectHeight);
    
    glTexCoord2f(0.0, textureHeight);
    glVertex2f(bottomLeftX, bottomLeftY+rectHeight);
    
    glEnd();
}


void draw_two_textures_on_rectangle(float bottomLeftX, float bottomLeftY, float rectWidth, float rectHeight, float textureWidth, float textureHeight)
{
    glBegin(GL_QUADS);  // grid is a series of quadrilaterals
    
    glMultiTexCoord2f(GL_TEXTURE0, 0.0, 0.0);
    glMultiTexCoord2f(GL_TEXTURE1, 0.0, 0.0);
    glVertex2f(bottomLeftX, bottomLeftY);
    
    glMultiTexCoord2f(GL_TEXTURE0, textureWidth, 0.0);
    glMultiTexCoord2f(GL_TEXTURE1, textureWidth, 0.0);
    glVertex2f(bottomLeftX+rectWidth, bottomLeftY);
    
    glMultiTexCoord2f(GL_TEXTURE0, textureWidth, textureHeight);
    glMultiTexCoord2f(GL_TEXTURE1, textureWidth, textureHeight);
    glVertex2f(bottomLeftX+rectWidth, bottomLeftY+rectHeight);
    
    glMultiTexCoord2f(GL_TEXTURE0, 0.0, textureHeight);
    glMultiTexCoord2f(GL_TEXTURE1, 0.0, textureHeight);
    glVertex2f(bottomLeftX, bottomLeftY+rectHeight);
    
    glEnd();
}


void draw_three_textures_on_rectangle(float bottomLeftX, float bottomLeftY, float rectWidth, float rectHeight, float textureWidth, float textureHeight)
{
    glBegin(GL_QUADS);  // grid is a series of quadrilaterals
    
    glMultiTexCoord2f(GL_TEXTURE0, 0.0, 0.0);
    glMultiTexCoord2f(GL_TEXTURE1, 0.0, 0.0);
    glMultiTexCoord2f(GL_TEXTURE2, 0.0, 0.0);
    glVertex2f(bottomLeftX, bottomLeftY);
    
    glMultiTexCoord2f(GL_TEXTURE0, textureWidth, 0.0);
    glMultiTexCoord2f(GL_TEXTURE1, textureWidth, 0.0);
    glMultiTexCoord2f(GL_TEXTURE2, textureWidth, 0.0);
    glVertex2f(bottomLeftX+rectWidth, bottomLeftY);
    
    glMultiTexCoord2f(GL_TEXTURE0, textureWidth, textureHeight);
    glMultiTexCoord2f(GL_TEXTURE1, textureWidth, textureHeight);
    glMultiTexCoord2f(GL_TEXTURE2, textureWidth, textureHeight);
    glVertex2f(bottomLeftX+rectWidth, bottomLeftY+rectHeight);
    
    glMultiTexCoord2f(GL_TEXTURE0, 0.0, textureHeight);
    glMultiTexCoord2f(GL_TEXTURE1, 0.0, textureHeight);
    glMultiTexCoord2f(GL_TEXTURE2, 0.0, textureHeight);
    glVertex2f(bottomLeftX, bottomLeftY+rectHeight);
    
    glEnd();
}


void draw_four_textures_on_rectangle(float bottomLeftX, float bottomLeftY, float rectWidth, float rectHeight, float textureWidth, float textureHeight)
{
    glBegin(GL_QUADS);  // grid is a series of quadrilaterals
    
    glMultiTexCoord2f(GL_TEXTURE0, 0.0, 0.0);
    glMultiTexCoord2f(GL_TEXTURE1, 0.0, 0.0);
    glMultiTexCoord2f(GL_TEXTURE2, 0.0, 0.0);
    glMultiTexCoord2f(GL_TEXTURE3, 0.0, 0.0);
    glVertex2f(bottomLeftX, bottomLeftY);
    
    glMultiTexCoord2f(GL_TEXTURE0, textureWidth, 0.0);
    glMultiTexCoord2f(GL_TEXTURE1, textureWidth, 0.0);
    glMultiTexCoord2f(GL_TEXTURE2, textureWidth, 0.0);
    glMultiTexCoord2f(GL_TEXTURE3, textureWidth, 0.0);
    glVertex2f(bottomLeftX+rectWidth, bottomLeftY);
    
    glMultiTexCoord2f(GL_TEXTURE0, textureWidth, textureHeight);
    glMultiTexCoord2f(GL_TEXTURE1, textureWidth, textureHeight);
    glMultiTexCoord2f(GL_TEXTURE2, textureWidth, textureHeight);
    glMultiTexCoord2f(GL_TEXTURE3, textureWidth, textureHeight);
    glVertex2f(bottomLeftX+rectWidth, bottomLeftY+rectHeight);
    
    glMultiTexCoord2f(GL_TEXTURE0, 0.0, textureHeight);
    glMultiTexCoord2f(GL_TEXTURE1, 0.0, textureHeight);
    glMultiTexCoord2f(GL_TEXTURE2, 0.0, textureHeight);
    glMultiTexCoord2f(GL_TEXTURE3, 0.0, textureHeight);
    glVertex2f(bottomLeftX, bottomLeftY+rectHeight);
    
    glEnd();
}


uint16_t round_to_power_of_two(uint16_t value)
{
    // log2(value) = 2^i = value. ceiling of that takes it to the next integer value of 2, which is needed
    // lots of lrint conversions to make sure that the proper integer pops out and don't have to worry about rounding
    // errors giving a non-power of 2
    return static_cast<uint16_t>(lrint(pow(2, lrint(ceil(log2(value))))));
}

} // namespace ui
} // namespace vulcan

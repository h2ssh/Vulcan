/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UI_COMMON_GL_TEXTURE_HELPERS_H
#define UI_COMMON_GL_TEXTURE_HELPERS_H

#include <GL/gl.h>
#include <stdint.h>

namespace vulcan
{
namespace ui
{

void initialize_texture_mapping(GLuint* textureNames, uint8_t numTextures);
uint8_t* create_texture(uint16_t width, uint16_t height, uint8_t bytesPerTexel);
void free_textures(uint8_t** textures, uint8_t numTextures);

void initialize_texture(GLuint textureName,
                        uint8_t* data,
                        uint16_t width,
                        uint16_t height,
                        GLint internalFormat,
                        GLenum displayFormat);
void initialize_texture_16(GLuint textureName,
                           uint16_t* data,
                           uint16_t width,
                           uint16_t height,
                           GLint internalFormat,
                           GLenum displayFormat);
void activate_texture(GLuint textureName, GLuint textureNumber, GLuint textureMode);
void disable_texture(GLuint textureNumber);

void set_sub_texture(uint8_t* texture, uint16_t width, uint16_t height, GLint format);
void set_sub_texture_16(uint16_t* texture, uint16_t width, uint16_t height, GLint format);

// If doing many texture layers, this allows up to four, assuming they use identical texture coordinates
// otherwise, need to roll your own
void draw_one_texture_on_rectangle(float bottomLeftX,
                                   float bottomLeftY,
                                   float rectWidth,
                                   float rectHeight,
                                   float textureWidth,
                                   float textureHeight);
void draw_two_textures_on_rectangle(float bottomLeftX,
                                    float bottomLeftY,
                                    float rectWidth,
                                    float rectHeight,
                                    float textureWidth,
                                    float textureHeight);
void draw_three_textures_on_rectangle(float bottomLeftX,
                                      float bottomLeftY,
                                      float rectWidth,
                                      float rectHeight,
                                      float textureWidth,
                                      float textureHeight);
void draw_four_textures_on_rectangle(float bottomLeftX,
                                     float bottomLeftY,
                                     float rectWidth,
                                     float rectHeight,
                                     float textureWidth,
                                     float textureHeight);

uint16_t round_to_power_of_two(uint16_t value);

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMMON_GL_TEXTURE_HELPERS_H

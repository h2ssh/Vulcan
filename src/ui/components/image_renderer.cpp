/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <cstring>
#include <iostream>
#include <GL/gl.h>
#include <GL/glu.h>
#include "core/point.h"
#include "core/image.h"
#include "vision/filters.h"
#include "ui/common/gl_texture_helpers.h"
#include "ui/components/image_renderer.h"


using namespace vulcan;
using namespace vulcan::ui;


void set_image_base_color(void);
void draw_image(Point<float> bottomLeft, float imageWidth, float imageHeight, float textureWidth, float textureHeight);


ImageRenderer::ImageRenderer(void) :
                        imageWidth(0),
                        imageHeight(0),
                        textureWidth(0),
                        textureHeight(0),
                        texture(0)
{

}


void ImageRenderer::renderImage(const Image& image)
{
    if((imageWidth != image.getWidth()) || (imageHeight != image.getHeight()))
    {
        free_textures(&texture, 1);
        
        imageWidth  = image.getWidth();
        imageHeight = image.getHeight();
        
        initializeImageTexture(image);
    }

    enableImageTexture(image);

    float textureXMax = imageWidth / static_cast<float>(textureWidth);
    float textureYMax = imageHeight / static_cast<float>(textureHeight);

    set_image_base_color();
    
    draw_image(Point<float>(0, 0),
               imageWidth,
               imageHeight,
               textureXMax,
               textureYMax);
          
    disableImageTexture();
}


void ImageRenderer::initializeImageTexture(const Image& image)
{
    glGenTextures(1, &textureName);
    
    textureWidth  = round_to_power_of_two(imageWidth);
    textureHeight = round_to_power_of_two(imageHeight);
    
    texture = create_texture(textureWidth, textureHeight, 3);
    
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    
    glBindTexture(GL_TEXTURE_2D, textureName);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureWidth, textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, texture);
}


void ImageRenderer::enableImageTexture(const Image& image)
{
    glActiveTexture(GL_TEXTURE0);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, textureName);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    convertImageToTexture(image);
}


void ImageRenderer::disableImageTexture(void)
{
    glActiveTexture(GL_TEXTURE0);
    glDisable(GL_TEXTURE_2D);
}


void ImageRenderer::convertImageToTexture(const Image& image)
{
    Image filteredImage(image);
    
    vision::Gaussian2DFilter<2> gaussian(0.8);
    
//     gaussian.apply(image, filteredImage);
    
    uint8_t* pixels = image.getPixelBuffer();

    if(image.getColorspace() == MONO)
    {
        int total = imageWidth * imageHeight;
        int textureValue = 0;
        for(int n = 0; n < total; ++n, textureValue += 3)
        {
            texture[textureValue]   = pixels[n];
            texture[textureValue+1] = pixels[n];
            texture[textureValue+2] = pixels[n];
        }
    }
    else // colorspace == RGB
    {
        memcpy(texture, pixels, imageWidth*imageHeight*3);
    }

    set_sub_texture(texture, imageWidth, imageHeight, GL_RGB);
}


void set_image_base_color(void)
{
    // dealing with black and white images for now, no color is supported
    glColor4f(0.0, 0.0, 0.0, 1.0);
}


void draw_image(Point<float> bottomLeft, float imageWidth, float imageHeight, float textureWidth, float textureHeight)
{
    glBegin(GL_QUADS);

    // NOTE: The texture is drawn here "upside down" on purpose because the image is defined in a left-handed coordinate system.
    //       It won't likely be changed to the more desirable right-handed coordinates
    glTexCoord2f(0.0, textureHeight);
    glVertex2f(bottomLeft.x, bottomLeft.y);

    glTexCoord2f(textureWidth, textureHeight);
    glVertex2f(bottomLeft.x+imageWidth, bottomLeft.y);
    
    glTexCoord2f(textureWidth, 0);
    glVertex2f(bottomLeft.x+imageWidth, bottomLeft.y+imageHeight);
    
    glTexCoord2f(0.0, 0);
    glVertex2f(bottomLeft.x, bottomLeft.y+imageHeight);

    glEnd();
}

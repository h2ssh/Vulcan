/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     image_segment_renderer.cpp
 * \author   Collin Johnson
 *
 * Definition of ImageSegmentRenderer.
 */

#include "ui/components/image_segment_renderer.h"
#include "ui/common/gl_texture_helpers.h"
#include "vision/image_segment.h"
#include <GL/gl.h>
#include <cassert>
#include <cstring>


namespace vulcan
{
namespace ui
{

ImageSegmentRenderer::ImageSegmentRenderer(void)
: imageWidth(0)
, imageHeight(0)
, textureWidth(0)
, textureHeight(0)
, alpha(200)
, initialized(false)
{
}


void ImageSegmentRenderer::setImageSegments(const std::vector<vision::image_segment_t>& segments,
                                            int imageWidth,
                                            int imageHeight)
{
    assert(imageWidth * imageHeight > 0);

    if (!initialized || (imageWidth * imageHeight > this->imageWidth * this->imageHeight)) {
        initializeSegmentsTexture(imageWidth, imageHeight);
    }

    assert(textureWidth * textureHeight > 0);

    this->imageWidth = imageWidth;
    this->imageHeight = imageHeight;

    activate_texture(textureName, GL_TEXTURE0, GL_DECAL);
    convertSegmentsToTexture(segments);
    set_sub_texture(texture.data(), imageWidth, imageHeight, GL_RGBA);
    disable_texture(GL_TEXTURE0);

    initialized = true;
}


void ImageSegmentRenderer::renderImageSegments(void)
{
    if (initialized) {
        enableSegmentTexture();

        float textureXMax = maxSegmentX / static_cast<float>(textureWidth);
        float textureYMax = maxSegmentY / static_cast<float>(textureHeight);

        glPushMatrix();
        glTranslatef(0.0, imageHeight, 0.0);
        glScalef(1.0, -1.0f, 1.0);
        glBegin(GL_QUADS);

        draw_one_texture_on_rectangle(0, 0, imageWidth, imageHeight, textureXMax, textureYMax);

        glEnd();

        disableSegmentTexture();
    }
}


void ImageSegmentRenderer::initializeSegmentsTexture(int imageWidth, int imageHeight)
{
    if (!initialized) {
        glGenTextures(1, &textureName);
    }

    textureWidth = round_to_power_of_two(imageWidth);
    textureHeight = round_to_power_of_two(imageHeight);

    texture.resize(textureWidth * textureHeight * 4);

    glPixelStorei(GL_PACK_ALIGNMENT, 1);

    glBindTexture(GL_TEXTURE_2D, textureName);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, textureWidth, textureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture.data());
}


void ImageSegmentRenderer::enableSegmentTexture(void)
{
    activate_texture(textureName, GL_TEXTURE0, GL_REPLACE);
}


void ImageSegmentRenderer::disableSegmentTexture(void)
{
    disable_texture(GL_TEXTURE0);
}


void ImageSegmentRenderer::convertSegmentsToTexture(const std::vector<vision::image_segment_t>& segments)
{
    int pixelIndex = 0;

    maxSegmentY = 0;
    maxSegmentX = 0;

    minSegmentY = imageHeight;
    minSegmentX = imageWidth;

    std::fill(texture.begin(), texture.end(), 255);

    for (auto& segment : segments) {
        for (auto& pixel : segment.pixels) {
            pixelIndex = (pixel.x + pixel.y * imageWidth) * 4;

            texture[pixelIndex] = static_cast<uint8_t>(segment.averageColor[0]);
            texture[pixelIndex + 1] = static_cast<uint8_t>(segment.averageColor[1]);
            texture[pixelIndex + 2] = static_cast<uint8_t>(segment.averageColor[2]);
            texture[pixelIndex + 3] = alpha;
        }

        for (auto& pixel : segment.boundaryPixels) {
            pixelIndex = (pixel.x + pixel.y * imageWidth) * 4;

            texture[pixelIndex] = 0;
            texture[pixelIndex + 1] = 0;
            texture[pixelIndex + 2] = 0;
            texture[pixelIndex + 3] = 255;

            if (pixel.x > maxSegmentX) {
                maxSegmentX = pixel.x;
            } else if (pixel.x < minSegmentX) {
                minSegmentX = pixel.x;
            }

            if (pixel.y > maxSegmentY) {
                maxSegmentY = pixel.y;
            } else if (pixel.y < minSegmentY) {
                minSegmentY = pixel.y;
            }
        }
    }

    // Increase the max segment indices by 1 because the actual range is [min, max)
    ++maxSegmentX;
    ++maxSegmentY;
}

}   // namespace ui
}   // namespace vulcan

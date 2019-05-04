/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     image_segment_renderer.h
* \author   Collin Johnson
*
* Declaration of ImageSegmentRenderer.
*/

#ifndef UI_COMPONENTS_IMAGE_SEGMENT_RENDERER_H
#define UI_COMPONENTS_IMAGE_SEGMENT_RENDERER_H

#include <vector>
#include <cstdint>

namespace vulcan
{
namespace vision { struct image_segment_t; }
namespace ui
{

/**
* ImageSegmentRenderer renders a set of image segments onto the screen as a translucent overlay.
* The segment colors are assigned based on their average color.
*/
class ImageSegmentRenderer
{
public:

    /**
    * Constructor for ImageSegmentRenderer.
    */
    ImageSegmentRenderer(void);

    /**
    * setImageSegments set the image segments to be rendered.
    */
    void setImageSegments(const std::vector<vision::image_segment_t>& segments, int imageWidth, int imageHeight);

    /**
    * renderImageSegments renders the current set of segments.
    */
    void renderImageSegments(void);

private:

    void initializeSegmentsTexture(int imageWidth, int imageHeight);

    void enableSegmentTexture(void);
    void disableSegmentTexture(void);

    void convertSegmentsToTexture(const std::vector<vision::image_segment_t>& segments);

    int imageWidth;
    int imageHeight;

    // Only want to draw the segments where they exist in the image. If no segments are in some part of
    // the overall image, then just let the regular image be drawn instead.
    int maxSegmentY;
    int maxSegmentX;

    int minSegmentY;
    int minSegmentX;

    int textureWidth;
    int textureHeight;

    uint8_t alpha;

    bool initialized;

    unsigned int textureName;
    std::vector<uint8_t> texture;
};

}
}

#endif // UI_COMPONENTS_IMAGE_SEGMENT_RENDERER_H

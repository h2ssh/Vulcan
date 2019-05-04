/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UI_COMPONENTS_IMAGE_RENDERER_H
#define UI_COMPONENTS_IMAGE_RENDERER_H

namespace vulcan
{
class Image;

namespace ui
{

/**
* ImageRenderer renders an image in the current OpenGL context.
*/
class ImageRenderer
{
public:

    /**
    * Constructor for ImageRenderer.
    */
    ImageRenderer(void);

    /**
    * renderImage renders the provided image to the screen.
    */
    void renderImage(const Image& image);

private:

    void initializeImageTexture(const Image& image);

    void enableImageTexture(const Image& image);
    void disableImageTexture(void);

    void convertImageToTexture(const Image& image);

    uint16_t imageWidth;
    uint16_t imageHeight;

    uint16_t textureWidth;
    uint16_t textureHeight;

    GLuint   textureName;
    uint8_t* texture;
};

}
}

#endif // UI_COMPONENTS_IMAGE_RENDERER_H

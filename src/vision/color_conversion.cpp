/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <cmath>
#include <iostream>
#include "utils/minmax.h"
#include "vision/color_conversion.h"


using namespace vulcan::vision;


typedef unsigned char uchar;
typedef unsigned int uint;


template <typename T>
T abs(const T& t) { return (t < 0) ? -t : t; }


/** rgbPixelToHSV converts an RGB pixel to HSV. */
inline void rgbPixelToHSV(uchar r, uchar g, uchar b, uchar& h, uchar& s, uchar& v);

/** rgbPixelToYUV converts an RGB pixel to YUV. */
inline void rgbPixelToYUV(uchar r, uchar g, uchar b, uchar& y, uchar& u, uchar& v);

/** hsvPixelToRGB converts an HSV pixel to RGB. */
inline void hsvPixelToRGB(uchar h, uchar s, uchar v, uchar& r, uchar& g, uchar& b);

/** hsvPixelToYUV converts an HSV pixel to YUV. */
inline void hsvPixelToYUV(uchar h, uchar s, uchar v, uchar& y, uchar& u, uchar& vv);

/** yuvPixelToRGB converts a YUV pixel to RGB. */
inline void yuvPixelToRGB(uchar y, uchar u, uchar v, uchar& r, uchar& g, uchar& b);

/** yuvPixelToHSV converts a YUV pixel to HSV. */
inline void yuvPixelToHSV(uchar y, uchar u, uchar v, uchar& h, uchar& s, uchar& vv);

/** convertImage converts the pixels from one colorspace to another using the provided transformation equation. */
void convertImage(uchar* from, uchar* to, uint width, uint height, void (*transform)(uchar, uchar, uchar, uchar&, uchar&, uchar&));

/** rgbImageToHSV converts a buffer of RGB pixels into a buffer of HSV pixels. */
uchar* rgbImageToHSV(uchar* rgbImage, uchar* hsvImage, uint width, uint height);

/** hsvImageToRGB converts a buffer of HSV pixels into a buffer of RGB pixels. */
uchar* hsvImageToRGB(uchar* hsvImage, uchar* rgbImage, uint width, uint height);


/**
* convertPixelColorspace converts a pixel from one colorspace to another. Conversions exist for any
* combination of values in the Colorspace enumeration.
*
* \param    fromA           First pixel value to convert from, i.e. 'R' if from is RGB
* \param    fromB           Second pixel value to convert from, i.e. 'G' if from is RGB
* \param    fromC           Third pixel value to convert from, i.e. 'B' if from is RGB
* \param    toA             First pixel value to convert to, i.e. 'H' if to is HSV
* \param    toB             Second pixel value to convert to, i.e. 'S' if to is HSV
* \param    toC             Third pixel value to convert to, i.e. 'V' if to is HSV
* \param    from            Colorspace of the from pixel values
* \param    to              Colorspace for the to pixel values
*/
void vulcan::vision::convertPixelColorspace(uchar fromA, uchar fromB, uchar fromC, uchar& toA, uchar& toB, uchar& toC, Colorspace from, Colorspace to)
{
    // Based on to and from, call the appropriate conversion algorithm -- if unknown parameter, then take no action
    
    switch(from)
    {
    case RGB:
        if(to == YUV)
        {
            rgbPixelToYUV(fromA, fromB, fromC, toA, toB, toC);
        }
        else if(to == HSV)
        {
            rgbPixelToHSV(fromA, fromB, fromC, toA, toB, toC);
        }
        break;
        
    case HSV:
        if(to == YUV)
        {
            hsvPixelToYUV(fromA, fromB, fromC, toA, toB, toC);
        }
        else if(to == RGB)
        {
            hsvPixelToRGB(fromA, fromB, fromC, toA, toB, toC);
        }
        break;
        
    case YUV:
        if(to == RGB)
        {
            yuvPixelToRGB(fromA, fromB, fromC, toA, toB, toC);
        }
        else if(to == HSV)
        {
            yuvPixelToHSV(fromA, fromB, fromC, toA, toB, toC);
        }
        break;
        
    default:
        std::cerr<<"ERROR: convertPixelColorspace: Colorspace "<<from<<" is not supported"<<std::endl;
    }
}


/**
* convertImageColorspace converts an image from one colorspace to another. The provided image buffer
* are assumed to be the same size, length * height * 3, and non-null. The layout of pixels in the buffers
* is assumed to sequential. For example, if the colorspace is RGB, then the pixels are arranged ...rgbrgbrgbrgb...
*
* \param    from                Buffer containing original image pixels
* \param    to                  Buffer in which to place converted pixels (output)
* \param    width               Width of the image
* \param    height              Height of the image
* \param    csFrom              Colorspace of pixels in the from buffer
* \param    csIn                Colorspace of pixels in the to buffer after completion
*/
void vulcan::vision::convertImageColorspace(uchar* from, uchar* to, uint width, uint height, Colorspace csFrom, Colorspace csTo)
{
    // Call convert image with the appropriate transform function based on from and to
    
    switch(csFrom)
    {
    case HSV:
        if(csTo == YUV)
        {
            convertImage(from, to, width, height, hsvPixelToYUV);
        }
        else if(csTo == RGB)
        {
            convertImage(from, to, width, height, hsvPixelToRGB);
        }
        break;
        
    case RGB:
        if(csTo == YUV)
        {
            convertImage(from, to, width, height, rgbPixelToYUV);
        }
        else if(csTo == HSV)
        {
            convertImage(from, to, width, height, rgbPixelToHSV);
        }
        break;
        
    case YUV:
        if(csTo == RGB)
        {
            convertImage(from, to, width, height, yuvPixelToRGB);
        }
        else if(csTo == HSV)
        {
            convertImage(from, to, width, height, yuvPixelToHSV);
        }
        break;
        
    default:
        std::cerr<<"ERROR: convertImageColorspace: Colorspace "<<csFrom<<" is not supported"<<std::endl;
    }
}


void rgbPixelToHSV(uchar r, uchar g, uchar b, uchar& h, uchar& s, uchar& v)
{
    /*
    * The algorithm used to make the conversion is described in source code available at:
    *
    * http://www.efg2.com/Lab/Graphics/Colors/HSV.htm
    *
    * Currently using the naive implementation of this function, it can definitely be improved upon
    * if the need arises.
    */
    
    int rr = r;
    int gg = g;
    int bb = b;
    
    int hh = 0;
    int ss = 0;
    int vv = 0;
    
    int minrg = (rr < gg)    ? rr : gg;
    int min   = (bb < minrg) ? bb : minrg;
    int maxrg = (rr > gg)    ? rr : gg;
    int max   = (bb > maxrg) ? bb : maxrg;
    int delta = max - min;
    
    if(delta == 0)
    {
        delta = 1;
    }
    
    vv = max;
    
    if(max == 0)
    {
        ss = 0;
        hh = 0;
    }
    else
    {
        ss = 255*delta/max;
        
        if(r==max)
        {
            hh = 43 * (gg-bb)/delta;
            
            if(hh<0)
            {
                hh += 256;
            }
        }
        else if(gg == max)
        {
            hh = 85+43*(bb-rr)/delta;
        }
        else
        {
            hh = 170+43*(rr-gg)/delta;
        }
    }
    
    h = hh;
    s = ss;
    v = vv;
}


void hsvPixelToRGB(uchar h, uchar s, uchar v, uchar& r, uchar& g, uchar& b)
{
    /*
    * The conversion algorithm used below can be found on the web at:
    *
    * http://www.easyrgb.com/math.html
    *
    * Currently the following uses a naive implementation, speedup is definitely possible.
    */
    
    // First, if no saturation, then color is black!
    if(s == 0)
    {
        r = g = b = v;
    }
    else  // not so lucky, need to do full calculation
    {
        float hVal = h / 255.0;
        float sVal = s / 255.0;
        float vVal = v / 255.0;
        
        float tempH = hVal * 6.0;
        
        if(tempH >= 6.0)   // can't have H > 1
        {
            tempH = 0;
        }
        
        // temporary values for use later in the calculation
        int   tempI = static_cast<int>(tempH);
        float tempX = vVal * (1.0 - sVal);
        float tempY = vVal * (1.0 - (sVal * (tempH - tempI)));
        float tempZ = vVal * (1.0 - (sVal * (1 - (tempH - tempI))));
        
        float tempR = 0;
        float tempG = 0;
        float tempB = 0;
        
        #ifdef DEBUG_CONV
        std::cout<<"tempH: "<<tempH<<" tempI: "<<tempI<<std::endl;
        std::cout<<"tempX: "<<(uchar)(tempX * 255)<<" tempY: "<<(uchar)(tempY * 255)<<" tempZ: "<<(uchar)(tempZ * 255)<<" V: "<<(uchar)(v * 255)<<std::endl;
        #endif
        
        // determine which temp values gets assigned where
        switch(tempI)
        {
        case 0:
            tempR = vVal;
            tempG = tempZ;
            tempB = tempX;
            break;
            
        case 1:
            tempR = tempY;
            tempG = vVal;
            tempB = tempX;
            break;
            
        case 2:
            tempR = tempX;
            tempG = vVal;
            tempB = tempZ;
            break;
            
        case 3:
            tempR = tempX;
            tempG = tempY;
            tempB = vVal;
            break;
            
        case 4:
            tempR = tempZ;
            tempG = tempX;
            tempB = vVal;
            break;
            
        default:
            tempR = vVal;
            tempG = tempX;
            tempB = tempY;
        }
        
        // Now scale all the results to be from 0 - 255
        r = static_cast<uchar>(round(tempR * 255));
        g = static_cast<uchar>(round(tempG * 255));
        b = static_cast<uchar>(round(tempB * 255));
    }
}


/**
* rgbImageToHSV converts an image represented in the RGB colorspace into an image
* represented in the HSV colorspace.
*
* The rgbImage buffer is assumed to be in the format of a linear sequence of RGB values,
* i.e. rgbImage[0] = r, rgbImage[1] = g, rgbImage[2] = b.
*
* The size of the rgbImage buffer needs to be at least (width * height * 3) or
* else the results of the conversion are unpredictable.
*
* If hsvImage != 0, then it is assumed it is the same size as the rgbImage. If the
* hsvImage buffer is not allocate, then it will be initialized.
*
* After conversion, hsvImage will contain a linear sequence of HSV values in the
* same format as the rgbImage, that is, hsvImage[0] = h, hsvImage[1] = s, hsvImage[2] = v, etc.
*
* \param    rgbImage            Buffer of RGB pixels to be converted
* \param    hsvImage            Buffer to write HSV pixel values into
* \param    width               Width of the image
* \param    height              Height of the image
* \return   The new buffer of HSV pixel values. If there is an error, 0 is returned.
*/
uchar* rgbImageToHSV(uchar* rgbImage, uchar* hsvImage, uint width, uint height)
{
    // HSV image may contain nothing, if this is the case, then allocate it some memory before continuing/
    // If rgbImage == 0, then we're in trouble, so bail out immediately because there is no image to convert
    
    if(rgbImage == 0)
    {
        return 0;
    }
    
    // no image, so allocate one
    if(hsvImage == 0)
    {
        hsvImage = new uchar[width * height * 3];
    }
    
    for(int x = (width * height * 3) - 3; x >= 0; x -= 3)
    {
        rgbPixelToHSV(rgbImage[x], rgbImage[x + 1], rgbImage[x + 2], hsvImage[x], hsvImage[x + 1], hsvImage[x + 2]);
    }
    
    return hsvImage;
}


/**
* hsvImageToRGB converts an image represented in the HSV colorspace into an image
* represented in the RGB colorspace.
*
* The hsvImage buffer is assumed to be a linear sequence of HSV values, that is
* hsvImage[0] = h, hsvImage[1] = s, hsvImage[2] = v, and so forth.
*
* The size of hsvImage should be at least (width * height * 3). If this is not the
* case, then it is likely that the program will crash.
*
* If rgbImage != 0, it must be at least as large as hsvImage or else writes outside
* the allocated space will occur leading to unpredicatable results. If rgbImage
* is uninitialized, memory will be allocated to contain the new pixels.
*
* After conversion, rgbImage will contain a sequence of RGB pixel values stored
* in the form R,G,B, that is rgbImage[0] = r, rgbImage[1] = g, rgbImage[2] = b, etc.
*
* \param    hsvImage                Buffer of HSV pixels to be converted
* \param    rgbImage                Buffer to write RGB pixels to
* \param    width                   Width of the image
* \param    height                  Height of the image
* \return   The new buffer of RGB pixel values. If there is an error, 0 is returned.
*/
uchar* hsvImageToRGB(uchar* hsvImage, uchar* rgbImage, uint width, uint height)
{
    // rgbImage may be an empty buffer, if this is the case, then allocate a new buffer to fill
    // If hsvImage has no contents, the we can do nothing, so return an error
    
    if(hsvImage == 0)
    {
        return 0;
    }
    
    // No data, hook up the buffer with some memory
    if(rgbImage == 0)
    {
        rgbImage = new uchar[width * height * 3];
    }
    
    for(int x = (width * height * 3) - 3; x >= 0; x -= 3)
    {
        hsvPixelToRGB(hsvImage[x], hsvImage[x + 1], hsvImage[x + 2], rgbImage[x], rgbImage[x + 1], rgbImage[x + 2]);
    }
    
    return rgbImage;
}


void hsvPixelToYUV(uchar h, uchar s, uchar v, uchar& y, uchar& u, uchar& vo)
{
    // For now, just do it real slow and go hsv->rgb->yuv, it shouldn't really matter much as I doubt
    // this transform will ever need to occur
    
    uchar r = 0;
    uchar g = 0;
    uchar b = 0;
    
    hsvPixelToRGB(h, s, v, r, g, b);
    rgbPixelToYUV(r, g, b, y, u, vo);
}


void yuvPixelToHSV(uchar y, uchar u, uchar v, uchar& h, uchar& s, uchar& vo)
{
    // Be slow and sloppy -- go YUV->RGB->HSV, if there is a need to alter this, then it will happen
    // but it doesn't seem likely
    
    uchar r = 0;
    uchar g = 0;
    uchar b = 0;
    
    yuvPixelToRGB(y, u, v, r, g, b);
    rgbPixelToHSV(r, g, b, h, s, vo);
}


void rgbPixelToYUV(uchar r, uchar g, uchar b, uchar& y, uchar& u, uchar& v)
{
    // The implementation shown here uses the algorithm from the libdc1394 library
    
    uint yTemp = vulcan::min(abs(r * 2104 + g * 4130 + b * 802 + 4096 + 131072) >> 13,235);
    uint uTemp = vulcan::min(abs(r * -1214 + g * -2384 + b * 3598 + 4096 + 1048576) >> 13,240);
    uint vTemp = vulcan::min(abs(r * 3598 + g * -3013 + b * -585 + 4096 + 1048576) >> 13,240);
    
    y = yTemp;
    u = uTemp;
    v = vTemp;
}


void yuvPixelToRGB(uchar y, uchar u, uchar v, uchar& r, uchar& g, uchar& b)
{
    // The implementation here uses the algorithm from the libdc1394 library
    const unsigned int MAX_VAL = 255;
    
    uint bTemp = ((y << 20) + u * 1858076 + 0x80000) >> 20;
    uint gTemp = ((y << 20) - u * 360857 - v * 748830 + 0x80000) >> 20;
    uint rTemp = ((y << 20) + v * 1470103 + 0x80000) >> 20;
    
    r = rTemp > MAX_VAL ? MAX_VAL : rTemp;
    g = gTemp > MAX_VAL ? MAX_VAL : gTemp;
    b = bTemp > MAX_VAL ? MAX_VAL : bTemp;
    
}


/**
* convertImage converts a buffer of pixels from one colorspace to another.
*
* The two provided pixels buffers are assumed to be the same size, length * height * 3. Additionally,
* the pixels in the buffers should be layed out in sequential order. For example, if a buffer contains
* RGB pixels, the layout of the pixels is ...rgbrgbrgbrgb... in the buffer.
*
* convertImage is colorspace-independent. A function, transform, is provided that handles the task
* of converting the pixel values. In this way, convertImage provides a nice abstraction layer for
* all pixel-conversion tasks.
*
* \param    from            Source image containing pixels to be transformed
* \param    to              Destination image that will contain the transformed pixels (output)
* \param    width           Width of the image, in pixels
* \param    height          Height of the image, in pixels
* \param    transform       Function to be used for transforming pixel values
*/
void convertImage(uchar* from, uchar* to, uint width, uint height, void (*transform)(uchar, uchar, uchar, uchar&, uchar&, uchar&))
{
    // The task for converting is very simple. Just loop through all the pixels and call transform for each
    
    // Make sure the pointers are not null to avoid this glaringly obvious error
    if((from == 0) || (to == 0))
    {
        std::cerr<<"ERROR: Pixels buffers must not be null in convertImage. "<<std::endl;
        return;
    }
    
    for(int x = (width * height * 3) - 3; x >= 0; x -= 3)
    {
        transform(from[x], from[x + 1], from[x + 2], to[x], to[x + 1], to[x + 2]);
    }
}

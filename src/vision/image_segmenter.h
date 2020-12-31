/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_IMAGE_SEGMENTER_H
#define SENSORS_VISION_IMAGE_SEGMENTER_H

#include <boost/shared_ptr.hpp>

namespace vulcan
{
class Image;

namespace vision
{

struct image_segmenter_params_t;
struct image_segment_t;

/**
 * ImageSegmenter is an abstract base class for an image processing algorithm that accepts a full image as
 * input and produces a set of image_segment_t, which represent connected components of the image
 * using some definition of connected as determined by the particular algorithm.
 */
class ImageSegmenter
{
public:
    virtual ~ImageSegmenter(void) { }

    /**
     * segmentImage finds the connected components in an image and creates a set of image_segment_t
     * to represent these components. The image segments are bounded by a convex hull polygon to give
     * a loose bound of the area encompassed.
     */
    void segmentImage(const Image& image, std::vector<image_segment_t>& segments);

protected:
    /**
     * findImageSegments searches through the image to find the connected components that form
     * a segmentation of the image. The calculation of the bounding polygon is handled in the
     * base class, so findImageSegments only needs to determine the pixels and boundary pixels
     * for each segment.
     */
    virtual void findImageSegments(const Image& image, std::vector<image_segment_t>& segments) = 0;
};


/**
 * create_image_segmenter is a factory for creating an implementation of the ImageSegmenter base class.
 * The type created is based on the provided string.
 */
boost::shared_ptr<ImageSegmenter> create_image_segmenter(const std::string& type,
                                                         const image_segmenter_params_t& params);


}   // namespace vision
}   // namespace vulcan

#endif   // SENSORS_VISION_IMAGE_SEGMENTER_H

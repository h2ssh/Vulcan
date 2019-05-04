/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_NAVTEXTURE_IMAGE_OBJECT_IDENTIFIER_H
#define SENSORS_VISION_NAVTEXTURE_IMAGE_OBJECT_IDENTIFIER_H

#include <vector>
#include <core/laser_scan.h>
#include <vision/graph_based_segmenter.h>
#include <vision/distortion.h>
#include <vision/homography.h>
#include <vision/navtexture/navtexture_params.h>
#include <vision/navtexture/image_segment_clusterer.h>

namespace vulcan
{

namespace vision
{

struct image_object_t;

/**
* ImageObjectIdentifier handles the task of breaking the image down into
* segments and then identifying which of those segments belong to the ground
* plane and which segments are on the border of the ground plane.
*
* The process is broken into three parts:
*
* 0) Doing a fine-grained segmentation of the image. This segmentation
*    is performed using a standard image segmentation algorithm from
*    the literature.
* 1) Performing k-means clustering on these fine-grained segments to try
*    and identify the dominant segments in the image.
* 2) Taking the coarse segments and matching them to the laser data. This
*    classifies the segments into ground plane and non-ground plane.
*/
class ImageObjectIdentifier
{
public:

    /**
    * Constructor for ImageObjectIdentifier.
    */
    ImageObjectIdentifier(const navtexture_params_t& params);

    /**
    * identifyObjects find the relevant objects in an image. The relevant objects
    * are those on the ground plane and those adjacent to the ground plane.
    */
    void identifyObjects(const Image&                 image,
                         const polar_laser_scan_t&     scan,
                         const laser::dynamic_laser_points_t& dynamicPoints,
                         std::vector<image_object_t>&         objects);

    /**
    * getImageSegments retrieves the raw segments from which the image objects were
    * created. The segments are labelled with the center of the k-means cluster
    * to which each segment was associated.
    */
    std::vector<image_segment_t>& getImageSegments(void) { return finalSegments; }

private:

    void undistortImageSegments(void);

    void matchLaserToSegments(const polar_laser_scan_t&     scan,
                              const laser::dynamic_laser_points_t& dynamicPoints,
                              const Image&                         image,
                              std::vector<image_object_t>&         objects);
    void convertLaserToPixels(const polar_laser_scan_t&    scan,
                              int                                 imageHeight,
                              std::vector<Point<int16_t>>& laserPixels);
    void calculateSegmentCenters(const std::vector<image_segment_t>& segments,
                                std::vector<Point<int16_t>>&  distortedCenters,
                                std::vector<Point<int16_t>>&  undistortedCenters);
    math::Polygon<int16_t> calculateLaserHull(const std::vector<Point<int16_t>>& laserPixels, int imageWidth);

    void findPotentialGroundPlaneSegments(const std::vector<Point<int16_t>>& segmentCenters,
                                          const math::Polygon<int16_t>&             laserHull,
                                          std::vector<size_t>&                      planeSegmentIndices);
    void filterPlaneSegments(const std::vector<Point<int16_t>>& laserPixels,
                             const std::vector<Point<int16_t>>& segmentCenters,
                             std::vector<size_t>&                      planeIndices);

    void matchDynamicPointsToSegments(const laser::dynamic_laser_points_t& points, const std::vector<size_t>& planeIndices, std::vector<size_t>& dynamicIndices);
    void convertDynamicPointsToImage(const laser::dynamic_laser_points_t& points, std::vector<Point<int16_t>>& dynamic);

    void findPlaneSegmentsAdjacentToDynamic(const std::vector<size_t>& planeIndices, const std::vector<size_t>& dynamicIndices, std::vector<size_t>& adjacentIndices);
    void findAdjacentSegments(const Point<int16_t>& boundaryPixel,
                              unsigned int                segmentId,
                              const std::vector<size_t>&  planeIndices,
                              std::vector<unsigned int>&  adjacentSegmentIds,
                              std::vector<size_t>&        adjacentIndices);

    void constructImageObjectsForGroundSegments(const std::vector<size_t>&      planeIndices,
                                                const std::vector<size_t>&      agentIndices,
                                                const std::vector<size_t>&      adjacentIndices,
                                                const std::vector<Point<int16_t>>& segmentCenters,
                                                std::vector<image_object_t>& objects);

    void calculateFeatureDescriptorsForGroundSegments(const Image& image, std::vector<image_object_t>& objects);

    boost::shared_ptr<GraphBasedSegmenter> segmenter;
    ImageSegmentClusterer                  clusterer;

    std::vector<image_segment_t> segments;
    std::vector<image_segment_t> finalSegments;

    cartesian_laser_scan_t cartesian;

    int imageWidth;
    int imageHeight;

    float maxLaserDistance;
    int   minDynamicMatches;

    vision::homography_matrix_t  matrix;
    vision::camera_calibration_t calibration;

    image_object_identifier_params_t params;
};

}
}

#endif // SENSORS_VISION_NAVTEXTURE_IMAGE_OBJECT_IDENTIFIER_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <functional>
#include <fstream>
#include <iostream>
#include <set>
#include "math/coordinates.h"
#include "math/geometry/convex_hull.h"
#include "core/line.h"
#include "core/point.h"
#include "core/laser_scan.h"
#include "core/laser_scan.h"
#include "laser/dynamic_laser_points.h"
#include "vision/image_segment.h"
#include "vision/navtexture/image_object.h"
#include "vision/navtexture/image_object_identifier.h"


// #define DEBUG_GROUND


using namespace vulcan;
using namespace vulcan::vision;


ImageObjectIdentifier::ImageObjectIdentifier(const navtexture_params_t& params) :
                                        clusterer(params.identifierParams.numSegmentClusters, params.identifierParams.maxClusteringIterations),
                                        params(params.identifierParams)
{
    segmenter = create_graph_based_image_segmenter(params.identifierParams.segmenterType, params.identifierParams.segmenterParams);

    std::ifstream homographyIn(params.identifierParams.homographyFile.c_str());

    assert(homographyIn.is_open());

    homographyIn>>matrix;

    std::ifstream calibrationIn(params.identifierParams.calibrationFile.c_str());

    assert(calibrationIn.is_open());

    calibrationIn>>calibration;
}


void ImageObjectIdentifier::identifyObjects(const Image&                 image,
                                            const polar_laser_scan_t&     scan,
                                            const laser::dynamic_laser_points_t& dynamicPoints,
                                            std::vector<image_object_t>&         objects)
{
    segments.clear();
    finalSegments.clear();

    imageWidth  = image.getWidth();
    imageHeight = image.getHeight();

    std::vector<image_segment_t> imageSegments;

    segmenter->segmentImage(image, imageSegments);
    clusterer.clusterSegments(imageSegments, segmenter->getForest(), image, finalSegments);

    undistortImageSegments();
    matchLaserToSegments(scan, dynamicPoints, image, objects);

    assert(objects.size() == finalSegments.size());

    calculateFeatureDescriptorsForGroundSegments(image, objects);
}


void ImageObjectIdentifier::undistortImageSegments(void)
{
    for(auto segmentIt = finalSegments.begin(), endSegment = finalSegments.end(); segmentIt != endSegment; ++segmentIt)
    {
        undistort(segmentIt->boundingPolygon.vertices, calibration, segmentIt->boundingPolygon.vertices);
    }
}


void ImageObjectIdentifier::matchLaserToSegments(const polar_laser_scan_t&     scan,
                                                 const laser::dynamic_laser_points_t& dynamicPoints,
                                                 const Image&                 image,
                                                 std::vector<image_object_t>&         objects)
{
    std::vector<Point<int16_t>> laserPixels;
    std::vector<Point<int16_t>> distortedCenters;
    std::vector<Point<int16_t>> undistortedCenters;
    std::vector<size_t>                groundPlaneIndices;
    std::vector<size_t>                dynamicIndices;
    std::vector<size_t>                adjacentIndices;
    math::Polygon<int16_t>             laserHull;

    convertLaserToPixels   (scan, image.getHeight(), laserPixels);
    calculateSegmentCenters(finalSegments, distortedCenters, undistortedCenters);

    laserHull = calculateLaserHull(laserPixels, image.getWidth());

    findPotentialGroundPlaneSegments(undistortedCenters, laserHull, groundPlaneIndices);

    filterPlaneSegments(laserPixels, undistortedCenters, groundPlaneIndices);

    if(!dynamicPoints.points.empty())
    {
        matchDynamicPointsToSegments(dynamicPoints, groundPlaneIndices, dynamicIndices);
        findPlaneSegmentsAdjacentToDynamic(groundPlaneIndices, dynamicIndices, adjacentIndices);
    }

    // Need to use the distorted centers for the image objects because further processing of the objects happens in the distorted
    // image, not the undistorted.
    constructImageObjectsForGroundSegments(groundPlaneIndices, dynamicIndices, adjacentIndices, distortedCenters, objects);

//     for(size_t n = 0; n < groundPlaneIndices.size(); ++n)
//     {
//         finalSegments[groundPlaneIndices[n]].averageColor[0] = 0;
//         finalSegments[groundPlaneIndices[n]].averageColor[1] = 0;
//         finalSegments[groundPlaneIndices[n]].averageColor[2] = 255;
//     }
//
//     for(size_t n = 0; n < dynamicIndices.size(); ++n)
//     {
//         finalSegments[dynamicIndices[n]].averageColor[0] = 0;
//         finalSegments[dynamicIndices[n]].averageColor[1] = 255;
//         finalSegments[dynamicIndices[n]].averageColor[2] = 0;
//     }
//
//     for(size_t n = 0; n < adjacentIndices.size(); ++n)
//     {
//         finalSegments[adjacentIndices[n]].averageColor[0] = 0;
//         finalSegments[adjacentIndices[n]].averageColor[1] = 255;
//         finalSegments[adjacentIndices[n]].averageColor[2] = 255;
//     }
}


void ImageObjectIdentifier::convertLaserToPixels(const polar_laser_scan_t& scan, int imageHeight, std::vector<Point<int16_t>>& laserPixels)
{
    /*
    * When converting, ignore all pixels that don't fall within the y-value of the image. These laser pixels would be pure noise, as
    * the homography does not include them. Their values swing around wildly, so disregarding is the best approach.
    */

    polar_scan_to_cartesian_scan_in_robot_frame(scan, cartesian, true);

    Point<float> laserPixel;

    for(size_t n = 0; n < cartesian.numPoints-1; n += 2)
    {
        // Don't pay attention to the scan points which fall behind the robot because they
        // aren't in the field-of-view of the camera
        if(cartesian.scanPoints[n].x > 0 && cartesian.scanPoints[n].y != 0 && scan.ranges[n] < params.maxLaserDistance)
        {
            laserPixel = world_to_image_coordinates(cartesian.scanPoints[n], matrix);

            // Only add pixels which have a positive y and less than half the image -- meaning they could appear in the view of the camera
            if(laserPixel.y >= 0 && laserPixel.y <= imageHeight)
            {
                laserPixels.push_back(Point<int16_t>(laserPixel.x, laserPixel.y));
            }
        }
    }
}


void ImageObjectIdentifier::calculateSegmentCenters(const std::vector<image_segment_t>& segments,
                                                    std::vector<Point<int16_t>>& distortedCenters,
                                                    std::vector<Point<int16_t>>& undistortedCenters)
{
    distortedCenters.reserve(segments.size());

    for(auto segmentIt = segments.begin(), segmentEnd = segments.end(); segmentIt != segmentEnd; ++segmentIt)
    {
        int sumX = 0;
        int sumY = 0;

        for(auto pointIt = segmentIt->boundaryPixels.begin(), pointEnd = segmentIt->boundaryPixels.end(); pointIt != pointEnd; ++pointIt)
        {
            sumX += pointIt->x;
            sumY += pointIt->y;
        }

        distortedCenters.push_back(Point<int16_t>(sumX / segmentIt->boundaryPixels.size(), sumY / segmentIt->boundaryPixels.size()));
    }

    undistortedCenters.resize(distortedCenters.size());

    // Centers need to be undistorted, as homography exists in undistorted image
    undistort(distortedCenters, calibration, undistortedCenters);
}


math::Polygon<int16_t> ImageObjectIdentifier::calculateLaserHull(const std::vector<Point<int16_t>>& laserPixels, int imageWidth)
{
    /*
    * If a pixel goes outside the view of the image, but the y-coordinate is valid, then clamp the x-coordinate to the
    * image width.
    */

    math::Polygon<int16_t> laserHull;

    laserHull.vertices.push_back(Point<int16_t>(imageWidth / 2, 0));

    for(auto pixelIt = laserPixels.begin(), pixelEnd = laserPixels.end(); pixelIt != pixelEnd; ++pixelIt)
    {
        Point<int16_t> toAdd(*pixelIt);

//         if(toAdd.x < 0)
//         {
//             toAdd.x = 0;
//         }
//         if(toAdd.x > imageWidth)
//         {
//             toAdd.x = imageWidth;
//         }

        laserHull.vertices.push_back(toAdd);
    }

    return laserHull;
}


void ImageObjectIdentifier::findPotentialGroundPlaneSegments(const std::vector<Point<int16_t>>& segmentCenters,
                                                             const math::Polygon<int16_t>&             laserHull,
                                                             std::vector<size_t>&                      planeSegmentIndices)
{
    #ifdef DEBUG_GROUND
    std::cout<<"INFO:ObjectIdentifier:Laser hull:"<<laserHull<<'\n';
    #endif

    // Potential ground plane segments are those whose center falls inside the convex hull of laser points
    for(size_t n = 0; n < segmentCenters.size(); ++n)
    {
        if(laserHull.contains(segmentCenters[n]))
        {
            planeSegmentIndices.push_back(n);

            #ifdef DEBUG_GROUND
            std::cout<<"INFO:ObjectIdentifier:Ground segment:"<<n<<"->"<<segmentCenters[n]<<'\n';
            #endif
        }
    }

    #ifdef DEBUG_GROUND
    std::cout<<"INFO:ObjectIdentifier:"<<planeSegmentIndices.size()<<'/'<<segmentCenters.size()<<" segments on ground plane\n";
    #endif
}


void ImageObjectIdentifier::filterPlaneSegments(const std::vector<Point<int16_t>>& laserPixels,
                                                const std::vector<Point<int16_t>>& segmentCenters,
                                                std::vector<size_t>&                      planeIndices)
{
    /*
    * Plane segments in which a laser point lands are less likely to be ground plane. Right now, they are completely
    * ignored as being on the ground plane. A laser point could land in two or more convex hulls, in which case the
    * segment whose x-coordinate is furthest from the center of the image is selected. The reason for this choice is
    * the laser defines the border of freespace. An x-coordinate further from the center defines a wider freespace,
    * which is more likely to be correct.
    */

    // Sort the set in ascending order because all these elements are removed and they need to be removed starting with
    // the end of the vector, otherwise the contents get disorganized
    std::set<int, std::greater<int>> toRemove;

    for(auto laserIt = laserPixels.begin(), laserEnd = laserPixels.end(); laserIt != laserEnd; ++laserIt)
    {
        int polygonCount  = 0;
        int indexToRemove = -1;
        int segmentXDist  = 0;

        for(int n = planeIndices.size(); --n >= 0;)
        {
            assert(planeIndices[n] < finalSegments.size());

            if(finalSegments[planeIndices[n]].boundingPolygon.contains(*laserIt))
            {
                ++polygonCount;
                if(abs(segmentCenters[planeIndices[n]].x - imageWidth/2) > segmentXDist)
                {
                    indexToRemove = n;

                    segmentXDist = abs(segmentCenters[planeIndices[n]].x - imageWidth/2);
                }
            }
        }

        if(indexToRemove >= 0)
        {
            toRemove.insert(indexToRemove);
        }
    }

    for(auto removeIt = toRemove.begin(), removeEnd = toRemove.end(); removeIt != removeEnd; ++removeIt)
    {
        planeIndices.erase(planeIndices.begin() + *removeIt);
    }
}


void ImageObjectIdentifier::matchDynamicPointsToSegments(const laser::dynamic_laser_points_t& points, const std::vector<size_t>& planeIndices, std::vector<size_t>& dynamicIndices)
{
    std::vector<Point<int16_t>> dynamicPoints;

    convertDynamicPointsToImage(points, dynamicPoints);

    // If there are no points, there's no need to match to segments, so exit immediately to save some cycles
    if(dynamicPoints.empty())
    {
        return;
    }

    for(size_t n = 0; n < finalSegments.size(); ++n)
    {
        // Only match dynamic points to segments NOT on the ground plane
        if(std::find(planeIndices.begin(), planeIndices.end(), n) != planeIndices.end())
        {
            continue;
        }

        int dynamicCount = 0;

        for(auto pointIt = dynamicPoints.begin(), pointEnd = dynamicPoints.end(); pointIt != pointEnd; ++pointIt)
        {
            if(finalSegments[n].boundingPolygon.contains(*pointIt))
            {
                ++dynamicCount;
            }

            if(dynamicCount >= params.minDynamicMatches)
            {
                dynamicIndices.push_back(n);
                break;
            }
        }
    }
}


void ImageObjectIdentifier::convertDynamicPointsToImage(const laser::dynamic_laser_points_t& points, std::vector<Point<int16_t>>& dynamic)
{
    // The dynamic points are in the global frame, so they needed to be converted back to the robot frame, where all calculations for the plane analysis occur
    for(size_t n = 0; n < points.points.size(); ++n)
    {
        Point<float> converted = math::global_frame_to_robot_frame(points.points[n], points.pose);

        if(sqrt(converted.x*converted.x + converted.y*converted.y) > params.maxLaserDistance)
        {
            continue;
        }

        converted = world_to_image_coordinates(converted, matrix);

        if(converted.y > 0)
        {
            dynamic.push_back(Point<int16_t>(converted.x, converted.y));
        }
    }

    // Are there any worthwhile values to consider?
    if(dynamic.empty())
    {
        return;
    }

    undistort(dynamic, calibration, dynamic);
}


void ImageObjectIdentifier::findPlaneSegmentsAdjacentToDynamic(const std::vector<size_t>& planeIndices, const std::vector<size_t>& dynamicIndices, std::vector<size_t>& adjacentIndices)
{
    utils::DisjointSetForest& components = segmenter->getForest();

    std::vector<unsigned int> adjacentIds;

    // Go through all the dynamic segments and find all of the adjacent ground plane segments
    for(auto dynIndex = dynamicIndices.begin(), dynEnd = dynamicIndices.end(); dynIndex != dynEnd; ++dynIndex)
    {
        image_segment_t& dynamicSegment = finalSegments[*dynIndex];

        unsigned int segmentId  = components.findSet(dynamicSegment.segmentId);

        for(auto borderIt = dynamicSegment.boundaryPixels.begin(), borderEnd = dynamicSegment.boundaryPixels.end(); borderIt != borderEnd; ++borderIt)
        {
            findAdjacentSegments(*borderIt, segmentId, planeIndices, adjacentIds, adjacentIndices);
        }
    }
}


void ImageObjectIdentifier::findAdjacentSegments(const Point<int16_t>& boundaryPixel,
                                                 unsigned int                segmentId,
                                                 const std::vector<size_t>&  planeIndices,
                                                 std::vector<unsigned int>&  adjacentSegmentIds,
                                                 std::vector<size_t>&        adjacentIndices)
{
    utils::DisjointSetForest& components = segmenter->getForest();

    unsigned int pixelIndex = boundaryPixel.x + boundaryPixel.y*imageWidth;
    unsigned int adjacentId = segmentId;

    if(boundaryPixel.x > 0)
    {
        adjacentId = components.findSet(pixelIndex-1);
    }

    if((adjacentId == segmentId) && (boundaryPixel.x < imageWidth-1))
    {
        adjacentId = components.findSet(pixelIndex + 1);
    }

    if((adjacentId == segmentId) && (boundaryPixel.y > 0))
    {
        adjacentId = components.findSet(pixelIndex - imageWidth);
    }

    if((adjacentId == segmentId) && (boundaryPixel.y < imageHeight-1))
    {
        adjacentId = components.findSet(pixelIndex + imageWidth);
    }

    // Check to see if the adjacentId exists and has not already been added to the segments. Saves serious processing time
    // as opposed to looking through all the plane indices over and over again
    if(adjacentId == segmentId || std::find(adjacentSegmentIds.begin(), adjacentSegmentIds.end(), adjacentId) != adjacentSegmentIds.end())
    {
        return;
    }

    for(auto planeIndex = planeIndices.begin(), planeEnd = planeIndices.end(); planeIndex != planeEnd; ++planeIndex)
    {
        image_segment_t& planeSegment = finalSegments[*planeIndex];

        // If an adjacent ground segment index is found, add it to the collection and jump out immediately
        if(components.findSet(planeSegment.segmentId) == adjacentId)
        {
            adjacentSegmentIds.push_back(adjacentId);
            adjacentIndices.push_back(*planeIndex);
            break;
        }
    }
}


void ImageObjectIdentifier::constructImageObjectsForGroundSegments(const std::vector<size_t>& planeIndices,
                                                                   const std::vector<size_t>& agentIndices,
                                                                   const std::vector<size_t>& adjacentIndices,
                                                                   const std::vector<Point<int16_t>>& segmentCenters,
                                                                   std::vector<image_object_t>& objects)
{
    // Need to establish the colors, centers, and agent properties of an object. The feature descriptor is handled elsewhere.

    objects.resize(finalSegments.size());

    for(size_t n = 0; n < finalSegments.size(); ++n)
    {
        objects[n].center          = segmentCenters[n];
        objects[n].boundaryPixels  = finalSegments[n].boundaryPixels;
        objects[n].averageColor[0] = finalSegments[n].averageColor[0];
        objects[n].averageColor[1] = finalSegments[n].averageColor[1];
        objects[n].averageColor[2] = finalSegments[n].averageColor[2];

        objects[n].groundPlane = (std::find(planeIndices.begin(), planeIndices.end(), n) != planeIndices.end()) ? 1  : -1;
        objects[n].hasAgent    = (std::find(agentIndices.begin(), agentIndices.end(), n) != agentIndices.end()) ? -1 : 0;

        if(objects[n].hasAgent != -1)
        {
            objects[n].hasAgent = (std::find(adjacentIndices.begin(), adjacentIndices.end(), n) != adjacentIndices.end()) ? 1 : 0;
        }
    }
}


void ImageObjectIdentifier::calculateFeatureDescriptorsForGroundSegments(const Image& image, std::vector<image_object_t>& objects)
{
    assert(objects.size() == finalSegments.size());
    assert(objects.size() > 0);

    for(size_t n = 0; n < finalSegments.size(); ++n)
    {
        objects[n].histogram = PixelHistogram::createPixelHistogram(params.histogramType, image, finalSegments[n].pixels, params.histogramParams);
        objects[n].spin.calculate(image, objects[n].center, 10, 20);
    }
}

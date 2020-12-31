/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "vision/graph_based_segmenter.h"
#include "core/image.h"
#include "core/point.h"
#include "utils/counting_sort.h"
#include "utils/timestamp.h"
#include "vision/felzenszwalb_segmenter.h"
#include "vision/image_segment.h"
#include "vision/wassenberg_segmenter.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <map>

#define DEBUG_TIME

namespace vulcan
{
namespace vision
{

void update_image_segment(const Point<uint16_t>& pixel, bool border, const Image& image, image_segment_t& segment);


boost::shared_ptr<GraphBasedSegmenter> create_graph_based_image_segmenter(const std::string& type,
                                                                          const image_segmenter_params_t& params)
{
    if (type == FELZENSZWALB_SEGMENTER_TYPE) {
        return boost::shared_ptr<GraphBasedSegmenter>(new FelzenszwalbSegmenter(params.felzParams));
    } else if (type == WASSENBERG_SEGMENTER_TYPE) {
        return boost::shared_ptr<GraphBasedSegmenter>(new WassenbergSegmenter(params.wassenParams));
    }

    assert(false);
    return boost::shared_ptr<GraphBasedSegmenter>();
}


GraphBasedSegmenter::GraphBasedSegmenter(const graph_based_segmenter_params_t& params)
: gaussianFilter(params.sigma)
, numEdges(0)
, borderFlag(0)
, componentForest(0)
, params(params)
{
}


void GraphBasedSegmenter::findImageSegments(const Image& image, std::vector<image_segment_t>& segments)
{
    int64_t startTime = 0;
    int64_t initializeTime = 0;
    int64_t edgesTime = 0;
    int64_t segmentTime = 0;
    int64_t mergeTime = 0;
    int64_t extractTime = 0;

    startTime = utils::system_time_us();
    initializeSegmentation(image);
    initializeTime = utils::system_time_us() - startTime;

    startTime = utils::system_time_us();
    calculatePixelEdges(filteredImage);
    edgesTime = utils::system_time_us() - startTime;

    startTime = utils::system_time_us();
    int mergeIndex = segmentGraph();
    segmentTime = utils::system_time_us() - startTime;

    startTime = utils::system_time_us();
    mergeSmallSegments(mergeIndex);
    mergeTime = utils::system_time_us() - startTime;

    startTime = utils::system_time_us();
    extractImageSegments(filteredImage, segments);
    extractTime = utils::system_time_us() - startTime;

#ifdef DEBUG_TIME
    std::cout << "INFO: GraphBased timing:initialize:" << initializeTime / 1000 << " edges:" << edgesTime / 1000
              << " segment:" << segmentTime / 1000 << " merge:" << mergeTime / 1000 << " extract:" << extractTime / 1000
              << '\n';
#endif
}


void GraphBasedSegmenter::initializeSegmentation(const Image& image)
{
    unsigned int numImagePixels = image.getWidth() * image.getHeight();

    // Gaussian blur the source image and use the filtered image for the actual processing
    if ((image.getWidth() != filteredImage.getWidth()) || (image.getHeight() != filteredImage.getHeight())) {
        // Let the image operations handle the resizing
        filteredImage = image;
    }

    gaussianFilter.apply(image, filteredImage);

    // Setup pixels, components, and the component forest
    unsortedEdges.resize(numImagePixels * 4);
    pixelEdges.resize(numImagePixels * 4);
    numEdges = 0;

    // Setup the border flag for determining segment borders
    if ((borderFlag == 255) || (numImagePixels != borderPixels.size())) {
        borderPixels.resize(numImagePixels, borderFlag);
        std::fill(borderPixels.begin(), borderPixels.end(), 0);
        borderFlag = 0;
    }

    ++borderFlag;

    components.resize(numImagePixels);
    for (auto componentIt = components.begin(), endIt = components.end(); componentIt != endIt; ++componentIt) {
        componentIt->size = 1;
        componentIt->threshold = params.initialThreshold;   // all components size 1
    }

    if ((componentForest == 0) || (componentForest->size() < numImagePixels)) {
        delete componentForest;

        componentForest = new utils::DisjointSetForest(numImagePixels);
    } else {
        componentForest->reset();
    }
}


void GraphBasedSegmenter::calculatePixelEdges(const Image& image)
{
    // Create the edges
    for (size_t y = 0; y < image.getHeight(); ++y) {
        for (size_t x = 0; x < image.getWidth(); ++x) {
            connectPixelToAdjacent(x, y, image);
        }
    }

    // Sort them appropriately
    if (image.getColorspace() == MONO) {
        utils::counting_sort<256>(unsortedEdges.begin(), unsortedEdges.end(), pixelEdges.begin());
    } else {
        utils::counting_sort<768>(unsortedEdges.begin(), unsortedEdges.end(), pixelEdges.begin());
    }
}


void GraphBasedSegmenter::mergeSmallSegments(int startIndex)
{
    // Go through all the edges and see what size set they belong to. If size < min, merge with adjacent
    for (auto pixelIt = pixelEdges.begin() + startIndex, endIt = pixelEdges.end(); pixelIt != endIt; ++pixelIt) {
        unsigned int componentA = componentForest->findSet(pixelIt->pixelA);
        unsigned int componentB = componentForest->findSet(pixelIt->pixelB);

        if ((componentA != componentB)
            && ((components[componentA].size < params.minSegmentSize)
                || (components[componentB].size < params.minSegmentSize))) {
            mergeComponents(componentA, componentB);
        } else if (componentA != componentB)   // unmerged edges signify the borders between two segments
        {
            borderPixels[pixelIt->pixelA] = borderFlag;
            borderPixels[pixelIt->pixelB] = borderFlag;
        }
    }
}


void GraphBasedSegmenter::extractImageSegments(const Image& image, std::vector<image_segment_t>& segments)
{
    // Map a component ID to a segments ID as #segments << #image components
    std::map<unsigned int, int> componentToSegment;
    int nextSegment = 0;

    segments.clear();

    unsigned int component = 0;
    int segmentIndex = 0;

    Point<uint16_t> pixel;

    for (pixel.y = 0; pixel.y < image.getHeight(); ++pixel.y) {
        for (pixel.x = 0; pixel.x < image.getWidth(); ++pixel.x) {
            unsigned int pixelIndex = pixel.y * image.getWidth() + pixel.x;
            // Find the correct segment index. If none, add a new segment
            component = componentForest->findSet(pixelIndex);

            auto segmentIndexIt = componentToSegment.find(component);

            // Initialize a new segment
            if (segmentIndexIt == componentToSegment.end()) {
                componentToSegment.insert(std::make_pair(component, nextSegment));

                segments.push_back(image_segment_t());

                segments.back().segmentId = component;

                segmentIndex = nextSegment;
                ++nextSegment;
            } else {
                segmentIndex = segmentIndexIt->second;
            }

            update_image_segment(pixel, borderPixels[pixelIndex] == borderFlag, image, segments[segmentIndex]);
        }
    }

    // Take the summed pixel values and calculate the averages from them
    for (auto segmentIt = segments.begin(), endIt = segments.end(); segmentIt != endIt; ++segmentIt) {
        segmentIt->averageColor[0] /= segmentIt->pixels.size();
        segmentIt->averageColor[1] /= segmentIt->pixels.size();
        segmentIt->averageColor[2] /= segmentIt->pixels.size();
    }
}


void GraphBasedSegmenter::connectPixelToAdjacent(size_t x, size_t y, const Image& image)
{
    // Use 4-way connected graph
    bool useMonoWeight = image.getColorspace() == MONO;

    size_t width = image.getWidth();
    size_t height = image.getHeight();

    if (x < width - 1) {
        pixel_edge_t newEdge;
        newEdge.pixelA = x + y * width;
        newEdge.pixelB = x + 1 + y * width;
        newEdge.value = useMonoWeight ? monoPixelWeight(newEdge, image) : rgbPixelWeight(newEdge, image);

        unsortedEdges[numEdges++] = newEdge;
    }

    if (x > 0) {
        pixel_edge_t newEdge;
        newEdge.pixelA = x + y * width;
        newEdge.pixelB = x - 1 + y * width;
        newEdge.value = useMonoWeight ? monoPixelWeight(newEdge, image) : rgbPixelWeight(newEdge, image);

        unsortedEdges[numEdges++] = newEdge;
    }

    if (y < height - 1) {
        pixel_edge_t newEdge;
        newEdge.pixelA = x + y * width;
        newEdge.pixelB = x + (y + 1) * width;
        newEdge.value = useMonoWeight ? monoPixelWeight(newEdge, image) : rgbPixelWeight(newEdge, image);

        unsortedEdges[numEdges++] = newEdge;
    }

    if (y > 0) {
        pixel_edge_t newEdge;
        newEdge.pixelA = x + y * width;
        newEdge.pixelB = x + (y - 1) * width;
        newEdge.value = useMonoWeight ? monoPixelWeight(newEdge, image) : rgbPixelWeight(newEdge, image);

        unsortedEdges[numEdges++] = newEdge;
    }
}


unsigned int GraphBasedSegmenter::mergeComponents(unsigned int componentA, unsigned int componentB)
{
    unsigned int mergedComponent = componentForest->setUnion(componentA, componentB);

    if (mergedComponent != componentA) {
        components[componentB].size += components[componentA].size;
        components[componentA].size = 0;
    } else {
        components[componentA].size += components[componentB].size;
        components[componentB].size = 0;
    }

    return mergedComponent;
}


float GraphBasedSegmenter::rgbPixelWeight(const pixel_edge_t& edge, const Image& image) const
{
    // RGB weight is simply the distance in RGB space

    unsigned char* pixels = image.getPixelBuffer();
    int redA = pixels[3 * edge.pixelA];
    int greenA = pixels[3 * edge.pixelA + 1];
    int blueA = pixels[3 * edge.pixelA + 2];

    int redB = pixels[3 * edge.pixelB];
    int greenB = pixels[3 * edge.pixelB + 1];
    int blueB = pixels[3 * edge.pixelB + 2];

    return sqrt(pow(redA - redB, 2) + pow(greenA - greenB, 2) + pow(blueA - blueB, 2));
}


float GraphBasedSegmenter::monoPixelWeight(const pixel_edge_t& edge, const Image& image) const
{
    // Simple intensity difference for MONO images

    unsigned char* pixels = image.getPixelBuffer();
    int intensityA = pixels[edge.pixelA];
    int intensityB = pixels[edge.pixelB];

    return abs(intensityA - intensityB);
}


void update_image_segment(const Point<uint16_t>& pixel, bool border, const Image& image, image_segment_t& segment)
{
    uint8_t pixelValues[3];

    image.getPixel(pixel.x, pixel.y, pixelValues[0], pixelValues[1], pixelValues[2]);

    segment.averageColor[0] += pixelValues[0];
    segment.averageColor[1] += pixelValues[1];
    segment.averageColor[2] += pixelValues[2];

    segment.pixels.push_back(pixel);

    if (border || (pixel.x == 0) || (pixel.x == image.getWidth() - 1) || (pixel.y == 0)
        || (pixel.y == image.getHeight() - 1)) {
        segment.boundaryPixels.push_back(pixel);
    }
}

}   // namespace vision
}   // namespace vulcan

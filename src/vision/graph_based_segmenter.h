/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_VISION_GRAPH_BASED_SEGMENTER_H
#define SENSORS_VISION_GRAPH_BASED_SEGMENTER_H

#include "core/image.h"
#include "utils/disjoint_set_forest.h"
#include "vision/filters.h"
#include "vision/image_segmenter.h"
#include "vision/vision_params.h"
#include <vector>

namespace vulcan
{
namespace vision
{

/**
 * GraphBasedSegmenter is an abstract base class for the family of segmentation algorithms
 * that look for graph cuts to make in a graph of weighted edges between adjacent pixels.
 *
 * The most popular algorithm to fall into this class is the Felzenszwalb '04 segmenter.
 * A newer alternative is the Wassenberg '09 segmenter.
 *
 * All of the algorithms used the same graph structure, which is created by the GraphBasedSegmenter.
 * The difference between the algorithms is determining where to make the cuts. This task
 * is left to the subclasses to do via the segmentGraph() method.
 */
class GraphBasedSegmenter : public ImageSegmenter
{
public:
    /**
     * Constructor for GraphBasedSegmenter.
     */
    GraphBasedSegmenter(const graph_based_segmenter_params_t& params);

    /**
     * getForest retrieves the DisjointSetForest containing the component information
     * for each image in the most recently segmented image.
     */
    utils::DisjointSetForest& getForest(void) { return *componentForest; }

protected:
    struct pixel_edge_t
    {
        unsigned int pixelA;
        unsigned int pixelB;
        uint16_t value;

        // Pixel edges have to be sorted -- won't lambda functions be nice to avoid these simple methods?
        // Want to sort in non-decreasing order
        bool operator<(const pixel_edge_t& rhs) const { return this->value < rhs.value; }
    };

    struct component_set_info_t
    {
        unsigned int size;
        float threshold;
        float credit;
    };

    /**
     * findImageSegments is the method called by the ImageSegmenter for performing the actual segmentation.
     */
    virtual void findImageSegments(const Image& image, std::vector<image_segment_t>& segments);

    /**
     * segmentGraph is the method to be implemented by subclasses of the GraphBasedSegmenter. At the time
     * segmentGraph() is called the following invariants exist:
     *
     *   - pixelEdges contains all pixel edges sorted in non-decreasing order
     *   - components contains a size one component for each pixel in the image being segmented
     *
     * \return   The minimum index in pixelEdges for which all edges below the index were guaranteed to be merged.
     *           Provides some speedup for the merging of small segments.
     */
    virtual int segmentGraph(void) = 0;

    void initializeSegmentation(const Image& image);
    void calculatePixelEdges(const Image& image);
    void mergeSmallSegments(int startIndex);
    void extractImageSegments(const Image& image, std::vector<image_segment_t>& segments);

    void connectPixelToAdjacent(size_t x, size_t y, const Image& image);

    // Helper function to handle the task of merging
    unsigned int mergeComponents(unsigned int componentA, unsigned int componentB);

    float rgbPixelWeight(const pixel_edge_t& edge, const Image& image) const;
    float monoPixelWeight(const pixel_edge_t& edge, const Image& image) const;


    Image filteredImage;   // hold the Gaussian filtered image
    Gaussian2DFilter<4> gaussianFilter;

    std::vector<pixel_edge_t> unsortedEdges;
    std::vector<pixel_edge_t> pixelEdges;
    int numEdges;

    std::vector<uint8_t> borderPixels;
    uint8_t borderFlag;

    std::vector<component_set_info_t> components;
    utils::DisjointSetForest* componentForest;

    graph_based_segmenter_params_t params;
};

/**
 * create_graph_based_image_segmenter is a factory for creating instances of graph-based segmenters.
 */
boost::shared_ptr<GraphBasedSegmenter> create_graph_based_image_segmenter(const std::string& type,
                                                                          const image_segmenter_params_t& params);

}   // namespace vision
}   // namespace vulcan

#endif   // SENSORS_VISION_GRAPH_BASED_SEGMENTER_H

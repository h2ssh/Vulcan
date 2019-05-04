/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <cassert>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <utils/disjoint_set_forest.h>
#include <core/image.h>
#include <vision/image_segment.h>
#include <vision/navtexture/image_segment_clusterer.h>


// #define DEBUG_K_MEANS
// #define DEBUG_MERGE


using namespace vulcan;
using namespace vulcan::vision;


// cluster_t contains the info for a cluster
struct cluster_t
{
    // Implement interface needed for using with std::vector
    cluster_t(void)
    {
    }

    cluster_t(const cluster_t& toCopy) :
        segmentIndices(toCopy.segmentIndices)
    {
        center[0] = toCopy.center[0];
        center[1] = toCopy.center[1];
        center[2] = toCopy.center[2];
    }

    cluster_t& operator=(const cluster_t& rhs)
    {
        segmentIndices = rhs.segmentIndices;

        center[0] = rhs.center[0];
        center[1] = rhs.center[1];
        center[2] = rhs.center[2];

        return *this;
    }


    float            center[3];
    std::vector<int> segmentIndices;
};

// Functions for k-means
void run_k_means            (const std::vector<image_segment_t>& segments, int maxIterations, std::vector<cluster_t>& clusters);
void initialize_clusters    (const std::vector<image_segment_t>& segments, std::vector<cluster_t>& clusters);
bool assign_clusters        (const std::vector<image_segment_t>& segments, const std::vector<cluster_t>& oldClusters, std::vector<cluster_t>& newClusters);
void calculate_cluster_means(const std::vector<image_segment_t>& segments, std::vector<cluster_t>& clusters);
int  find_closest_cluster   (const vulcan::vision::image_segment_t& segment, const std::vector<cluster_t>& clusters);

// Functions for merging segments based on k-means
void merge_segments           (std::vector<image_segment_t>& segments, std::vector<cluster_t>& clusters, utils::DisjointSetForest& components, const Image& image);
void merge_segments_in_cluster(std::vector<image_segment_t>& segments, const std::vector<int>& clusterIndices, utils::DisjointSetForest& components, const Image& image);
int recursively_merge_segments_in_cluster(std::vector<image_segment_t>& segments,
                                          std::vector<int>&            clusterIndices,
                                          utils::DisjointSetForest&    components,
                                          const Image&        image);
void recursive_merge_helper (int                           indexToCheck,
                             int                           pixelId,
                             std::vector<image_segment_t>& segments,
                             std::vector<int>&             clusterIndices,
                             utils::DisjointSetForest&     components,
                             const Image&         image);
void find_adjacent_segments (const image_segment_t&  segment, const utils::DisjointSetForest& components, std::vector<unsigned int>& adjacent, const Image& image);
int  segment_is_in_cluster  (unsigned int segmentId, const std::vector<int>& indices, const std::vector<image_segment_t>& segments, const utils::DisjointSetForest& components);
void join_two_segments      (image_segment_t& into, image_segment_t& from);
void filter_boundary_pixels (image_segment_t& segment, const vulcan::Image& image, const vulcan::utils::DisjointSetForest& components);

inline float cluster_distance(const image_segment_t& segment, const cluster_t& cluster)
{
    // Ignore the sqrt, as ordering won't change and the actual distance doesn't currently matter
    return pow(segment.averageColor[0] - cluster.center[0], 2) +
           pow(segment.averageColor[1] - cluster.center[1], 2) +
           pow(segment.averageColor[2] - cluster.center[2], 2);
}


ImageSegmentClusterer::ImageSegmentClusterer(uint16_t numClusters, uint16_t maxIterations) :
                                        numClusters(numClusters),
                                        maxIterations(maxIterations)
{
}


void ImageSegmentClusterer::clusterSegments(std::vector<image_segment_t>& segments,
                                            utils::DisjointSetForest&     components,
                                            const Image&         image,
                                            std::vector<image_segment_t>& finalSegments)
{
    std::vector<cluster_t> clusters(numClusters);

    run_k_means(segments, maxIterations, clusters);

    // Copy over the clustered values as a test
    for(auto clusterIt = clusters.begin(), endIt = clusters.end(); clusterIt != endIt; ++clusterIt)
    {
        for(auto indexIt = clusterIt->segmentIndices.begin(), indexEnd = clusterIt->segmentIndices.end(); indexIt != indexEnd; ++indexIt)
        {
            segments[*indexIt].averageColor[0] = clusterIt->center[0];
            segments[*indexIt].averageColor[1] = clusterIt->center[1];
            segments[*indexIt].averageColor[2] = clusterIt->center[2];

            assert(segments[*indexIt].pixels.size() > 0);
        }
    }

    merge_segments(segments, clusters, components, image);

    // Remove all empty segments
    for(int n = segments.size(); --n >= 0;)
    {
        if(!segments[n].pixels.empty())
        {
            finalSegments.push_back(segments[n]);
        }
    }
}

// Functions for k-means
void run_k_means(const std::vector<image_segment_t>& segments, int maxIterations, std::vector<cluster_t>& clusters)
{
    std::vector<cluster_t> tempClusters[2];
    tempClusters[0].resize(clusters.size());
    tempClusters[1].resize(clusters.size());

    int activeIndex = 0;

    initialize_clusters(segments, tempClusters[activeIndex]);

    bool changed = false;

    int iterations = 0;

    do
    {
        calculate_cluster_means(segments, tempClusters[activeIndex]);

        changed = assign_clusters(segments, tempClusters[activeIndex], tempClusters[abs(1-activeIndex)]);

        // If finished, then copy the active clusters to the output
        // Otherwise, switch to the newly active clusters as established via assign_clusters and repeat the process
        if(!changed || iterations == maxIterations)
        {
            clusters = tempClusters[activeIndex];
        }
        else
        {
            activeIndex = abs(1-activeIndex);
        }

    } while(++iterations <= maxIterations && changed);

    #ifdef DEBUG_K_MEANS
    std::cout<<"INFO:k-means: "<<iterations<<" iterations\n";
    std::cout<<"INFO:k-means: Clusters:\n";

    for(auto clusterIt = clusters.begin(), endIt = clusters.end(); clusterIt != endIt; ++clusterIt)
    {
        std::cout<<"Size:"<<clusterIt->segmentIndices.size()<<" Means:("<<clusterIt->center[0]<<','<<clusterIt->center[1]<<','<<clusterIt->center[2]<<")\n";
    }
    #endif
}


void initialize_clusters(const std::vector<image_segment_t>& segments, std::vector<cluster_t>& clusters)
{
    // Current initialization is to divide segments evenly among the clusters. Cluster means will be
    // calculated elsewhere
    int segmentsPerCluster = segments.size() / clusters.size();

    assert(segmentsPerCluster > 0);  // less segments than clusters is a degenerate case

    size_t currentCluster = 0;

    for(size_t n = 0; n < segments.size(); ++n)
    {
        // Switch to filling the next cluster once enough segments have been assigned to the current
        // cluster. Don't switch if filling the last cluster, just put the extra values there.
        if((n % segmentsPerCluster == 0) && (currentCluster < clusters.size()-1))
        {
            currentCluster = n / segmentsPerCluster;
        }

        clusters[currentCluster].segmentIndices.push_back(n);
    }
}


bool assign_clusters(const std::vector<image_segment_t>& segments, const std::vector<cluster_t>& oldClusters, std::vector<cluster_t>& newClusters)
{
    bool changedAssignment = false;

    // Go through each clusters and find the closest cluster for the given segment. If it doesn't
    // match the current cluster, then the assignment changes, so note it

    // Ensure everything is initialized properly before going beginning to switch around cluster values
    for(auto clusterIt = newClusters.begin(), endIt = newClusters.end(); clusterIt != endIt; ++clusterIt)
    {
        clusterIt->segmentIndices.clear();
    }

    size_t closestCluster = 0;

    for(size_t n = 0; n < oldClusters.size(); ++n)
    {
        const std::vector<int>& indices = oldClusters[n].segmentIndices;

        for(size_t n = 0, end = indices.size(); n < end; ++n)
        {
            closestCluster = find_closest_cluster(segments[indices[n]], oldClusters);

            changedAssignment |= closestCluster != n;

            newClusters[closestCluster].segmentIndices.push_back(indices[n]);
        }
    }

    return changedAssignment;
}


void calculate_cluster_means(const std::vector<image_segment_t>& segments, std::vector<cluster_t>& clusters)
{
    // The center of a cluster is the mean of the segment average colors
    for(auto clusterIt = clusters.begin(), endIt = clusters.end(); clusterIt != endIt; ++clusterIt)
    {
        clusterIt->center[0] = 0;
        clusterIt->center[1] = 0;
        clusterIt->center[2] = 0;

        std::vector<int>& indices = clusterIt->segmentIndices;

        int clusterPixels = 0;

        for(size_t n = 0, end = indices.size(); n < end; ++n)
        {
            clusterIt->center[0] += segments[indices[n]].averageColor[0]*segments[indices[n]].pixels.size();
            clusterIt->center[1] += segments[indices[n]].averageColor[1]*segments[indices[n]].pixels.size();
            clusterIt->center[2] += segments[indices[n]].averageColor[2]*segments[indices[n]].pixels.size();

            clusterPixels += segments[indices[n]].pixels.size();
        }

        clusterIt->center[0] /= clusterPixels;
        clusterIt->center[1] /= clusterPixels;
        clusterIt->center[2] /= clusterPixels;
    }
}


int find_closest_cluster(const image_segment_t& segment, const std::vector<cluster_t>& clusters)
{
    float minDistance    = HUGE_VALF;
    int   closestCluster = 0;

    for(size_t n = 0; n < clusters.size(); ++n)
    {
        float distance = cluster_distance(segment, clusters[n]);

        if(distance < minDistance)
        {
            minDistance    = distance;
            closestCluster = n;
        }
    }

    return closestCluster;
}

// Functions for merging segments based on k-means
void merge_segments(std::vector<image_segment_t>& segments, std::vector<cluster_t>& clusters, utils::DisjointSetForest& components, const Image& image)
{
    for(auto clusterIt = clusters.begin(), endIt = clusters.end(); clusterIt != endIt; ++clusterIt)
    {
        merge_segments_in_cluster(segments, clusterIt->segmentIndices, components, image);
    }
}


void merge_segments_in_cluster(std::vector<image_segment_t>& segments,
                               const std::vector<int>&       clusterIndices,
                               utils::DisjointSetForest&     components,
                               const Image&         image)
{
    // Need to store the final merged id of each segment to ensure it isn't searched again, which would take
    // a long time to do and be completely useless
    for(auto clusterIt = clusterIndices.begin(), endIt = clusterIndices.end(); clusterIt != endIt; ++clusterIt)
    {
        std::vector<int> indicesToConsider(clusterIt+1, endIt);

        recursively_merge_segments_in_cluster(segments, indicesToConsider, components, image);
    }

    // Go through the cluster and for all merged segments, remove the extraneous boundary pixels
    for(auto clusterIt = clusterIndices.begin(), endIt = clusterIndices.end(); clusterIt != endIt; ++clusterIt)
    {
        image_segment_t& activeSegment = segments[*clusterIt];

        unsigned int activeId = components.findSet(activeSegment.segmentId);

        for(auto adjacentIt = clusterIt+1; adjacentIt != endIt; ++adjacentIt)
        {
            if(!segments[*adjacentIt].pixels.empty() && components.findSet(segments[*adjacentIt].segmentId) == activeId)
            {
                join_two_segments(activeSegment, segments[*adjacentIt]);
            }
        }

        if(!activeSegment.boundaryPixels.empty())
        {
            filter_boundary_pixels(activeSegment, image, components);
        }
    }
}


int recursively_merge_segments_in_cluster(std::vector<image_segment_t>& segments,
                                           std::vector<int>&            clusterIndices,
                                           utils::DisjointSetForest&    components,
                                           const Image&        image)
{
    // Check to see if recursion has bottomed out.
    if(clusterIndices.empty())
    {
        return -1;
    }

    image_segment_t& segment = segments[clusterIndices[0]];

    unsigned int pixelId = components.findSet(segment.segmentId);

    int imageWidth  = image.getWidth();
    int imageHeight = image.getHeight();

    for(auto borderIt = segment.boundaryPixels.begin(), endIt = segment.boundaryPixels.end(); borderIt != endIt; ++borderIt)
    {
        unsigned int pixelIndex    = borderIt->x + borderIt->y*image.getWidth();
        unsigned int adjacentIndex = 0;

        if(borderIt->x > 0)
        {
            adjacentIndex = pixelIndex - 1;

            recursive_merge_helper(adjacentIndex, pixelId, segments, clusterIndices, components, image);
        }

        if(borderIt->x < imageWidth-1)
        {
            adjacentIndex = pixelIndex + 1;

            recursive_merge_helper(adjacentIndex, pixelId, segments, clusterIndices, components, image);
        }

        if(borderIt->y > 0)
        {
            adjacentIndex = pixelIndex - image.getWidth();

            recursive_merge_helper(adjacentIndex, pixelId, segments, clusterIndices, components, image);
        }

        if(borderIt->y < imageHeight-1)
        {
            adjacentIndex = pixelIndex + image.getWidth();

            recursive_merge_helper(adjacentIndex, pixelId, segments, clusterIndices, components, image);
        }
    }

    return components.findSet(segments[clusterIndices[0]].segmentId);
}


void recursive_merge_helper(int                           indexToCheck,
                            int                           pixelId,
                            std::vector<image_segment_t>& segments,
                            std::vector<int>&             clusterIndices,
                            utils::DisjointSetForest&     components,
                            const Image&         image)
{
    int adjacentId   = components.findSet(indexToCheck);
    int clusterIndex = segment_is_in_cluster(adjacentId, clusterIndices, segments, components);

    // Only merge the cluster if the IDs are different, the segment is in the cluster, and the segment isn't already going to be merged
    if((adjacentId != pixelId) && (clusterIndex > 0))
    {
        std::vector<int> indices(clusterIndices.begin()+1, clusterIndices.end());
        std::swap(indices[0], indices[clusterIndex-1]);

        #ifdef DEBUG_MERGE
        int resultId = components.setUnion(adjacentId, pixelId);
        std::cout<<"Merged:"<<pixelId<<','<<adjacentId<<"->"<<resultId<<'\n';
        #endif

        recursively_merge_segments_in_cluster(segments, indices, components, image);
    }
}


int segment_is_in_cluster(unsigned int segmentId, const std::vector<int>& indices, const std::vector<image_segment_t>& segments, const utils::DisjointSetForest& components)
{
    for(size_t n = 0; n < indices.size(); ++n)
    {
        if(components.findSet(segments[indices[n]].segmentId) == segmentId)
        {
            return n;
        }
    }

    return -1;
}


void join_two_segments(image_segment_t& into, image_segment_t& from)
{
    into.pixels.insert(into.pixels.begin(), from.pixels.begin(), from.pixels.end());
    into.boundaryPixels.insert(into.boundaryPixels.begin(), from.boundaryPixels.begin(), from.boundaryPixels.end());

    from.pixels.clear();
    from.boundaryPixels.clear();
}


void filter_boundary_pixels(image_segment_t& segment, const Image& image, const utils::DisjointSetForest& components)
{
    unsigned int segmentId = components.findSet(segment.segmentId);

    int imageWidth  = image.getWidth();
    int imageHeight = image.getHeight();

    // Need to count backwards because loop is erasing values as it goes along
    for(int n = segment.boundaryPixels.size(); --n >= 0;)
    {
        unsigned int pixelIndex    = segment.boundaryPixels[n].x + segment.boundaryPixels[n].y*imageWidth;
        unsigned int adjacentIndex = 0;

        bool isBoundary = (segment.boundaryPixels[n].x == 0) || (segment.boundaryPixels[n].x == imageWidth-1) ||
                          (segment.boundaryPixels[n].y == 0) || (segment.boundaryPixels[n].y == imageHeight-1);

        // A pixel is on the boundary if one of its edges connects to another component, as indicated by
        // having a different id in the set forest
        if(segment.boundaryPixels[n].x > 0)
        {
            adjacentIndex = pixelIndex - 1;

            isBoundary |= components.findSet(adjacentIndex) != segmentId;
        }

        if(!isBoundary && segment.boundaryPixels[n].x < imageWidth-1)
        {
            adjacentIndex = pixelIndex + 1;

            isBoundary |= components.findSet(adjacentIndex) != segmentId;
        }

        if(!isBoundary && segment.boundaryPixels[n].y > 0)
        {
            adjacentIndex = pixelIndex - image.getWidth();

            isBoundary |= components.findSet(adjacentIndex) != segmentId;
        }

        if(!isBoundary && segment.boundaryPixels[n].y < imageHeight-1)
        {
            adjacentIndex = pixelIndex + image.getWidth();

            isBoundary |= components.findSet(adjacentIndex) != segmentId;
        }

        // If no boundary was found, then the pixel is extraneous, so it should be removed from the set
        if(!isBoundary)
        {
            segment.boundaryPixels.erase(segment.boundaryPixels.begin() + n);
        }
    }
}

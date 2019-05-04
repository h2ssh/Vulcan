/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     tracking_laser_scan.cpp
* \author   Collin Johnson
* 
* Definition of ObjectDetector.
*/

#include <tracker/object_detector.h>
#include <hssh/local_metric/lpm.h>
#include <laser/moving_laser_scan.h>
#include <math/coordinates.h>
#include <core/pose_distribution.h>
#include <utils/config_file.h>
#include <utils/ray_tracing.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <iostream>
#include <unordered_map>
#include <cassert>

#define DEBUG_LEG_MERGE

namespace vulcan 
{
namespace tracker 
{

struct AdjGraphVertex
{
    const LaserObject* object = nullptr;
    
    AdjGraphVertex(void) = default;
    AdjGraphVertex(const LaserObject* obj)
    : object(obj)
    {
    }
};

using AdjGraphType = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, AdjGraphVertex>;
using AdjGraphIds = std::unordered_map<const LaserObject*, int>;


const std::string DETECTOR_HEADING  ("ObjectDetectorParameters");
const std::string RAY_DIST_KEY      ("max_adjacent_ray_distance_m");
const std::string CLUSTER_SIZE_KEY  ("min_cluster_rays");
const std::string MAX_TRACK_DIST_KEY("max_tracking_distance_m");
const std::string LASER_VARIANCE_KEY("laser_variance_m2");
const std::string FILTER_GLASS_KEY  ("should_filter_glass_rays");
const std::string SAVE_POSITIONS_KEY("should_save_positions");
    
    
// Helper functions
bool neighbor_is_occupied(const Point<int>& cell, const hssh::LocalPerceptualMap& lpm);
void match_collections(const LaserObjectCollection& lhs, 
                       const LaserObjectCollection& rhs, 
                       const AdjGraphIds& ids,
                       AdjGraphType& matches);


object_detector_params_t::object_detector_params_t(const utils::ConfigFile& config)
: maxAdjacentRayDistance (config.getValueAsFloat (DETECTOR_HEADING, RAY_DIST_KEY))
, minClusterSize         (config.getValueAsUInt32(DETECTOR_HEADING, CLUSTER_SIZE_KEY))
, maxTrackingDistance    (config.getValueAsFloat (DETECTOR_HEADING, MAX_TRACK_DIST_KEY))
, laserVariance(config.getValueAsFloat (DETECTOR_HEADING, LASER_VARIANCE_KEY))
, filterGlassRays(config.getValueAsBool(DETECTOR_HEADING, FILTER_GLASS_KEY))
, savePositions(config.getValueAsBool(DETECTOR_HEADING, SAVE_POSITIONS_KEY))
{
    assert(maxAdjacentRayDistance  >  0.0f);
    assert(maxAdjacentRayDistance  <  maxTrackingDistance);
    assert(minClusterSize          >= 3);
    assert(maxTrackingDistance     >  0.0f);
    assert(laserVariance           >  0.0f);
}
    
    
ObjectDetector::ObjectDetector(const object_detector_params_t& params)
: params_(params)
{
    if(params_.savePositions)
    {
        positionOut_.open("tracker_detected_positions.dat");
    }
}


LaserObjectCollection ObjectDetector::detectObjects(const std::vector<laser::MovingLaserScan>& scans,
                                                    const pose_distribution_t& pose,
                                                    const hssh::LocalPerceptualMap&  lpm)
{
    std::vector<LaserObjectCollection> collections;
    
    for(auto& scan : scans)
    {
        identifyDynamicRays(scan, lpm);

        collections.emplace_back(segmentScan(scan, pose), scan.laserId(), scan.timestamp());

        if(params_.savePositions)
        {
            for(auto& object : collections.back())
            {
                positionOut_ << object.center().x << ' ' << object.center().y << '\n';
            }
        }
    }

    return mergeCollections(collections);
}


void ObjectDetector::identifyDynamicRays(const laser::MovingLaserScan& scan, const hssh::LocalPerceptualMap& lpm)
{
    // Find the dynamic endpoints in the scan
    // Take each cartesian point and look to see if it is classified as DYNAMIC. If so, add it to the
    // set of dynamic points.

    isDynamic_.resize(scan.size());
    std::fill(isDynamic_.begin(), isDynamic_.end(), false);

    // Need to maintain the ordering of the points, as that is part of the invariant for isDynamic_
    for(std::size_t n = 0; n < scan.size(); ++n)
    {
        const auto& ray = scan[n];
        
        // Ignore any invalidated ranges, though with a negative range
        if(ray.range < 0)
        {
            continue;
        }

        auto gridPoint = utils::global_point_to_grid_cell(ray.endpoint, lpm);

        bool cellTypeIsDynamic = lpm.getCellType(gridPoint) & (hssh::kDynamicOccGridCell | hssh::kFreeOccGridCell);
        bool cellIsProbablyFree = lpm.getCost(gridPoint) < 127; // if cost is less than "0", probably free, so just assume it is for now

        if((cellTypeIsDynamic || cellIsProbablyFree) && !neighbor_is_occupied(gridPoint, lpm))
        {
            isDynamic_[n] = !doesRayIntersectGlass(ray, lpm);
        }
    }
}


bool ObjectDetector::doesRayIntersectGlass(const laser::adjusted_ray_t& ray, const hssh::LocalPerceptualMap& lpm)
{
    bool rayIntersectsGlass = false;

    // Check to see if the laser crosses known glass, in which case the laser point is not
    if(params_.filterGlassRays)
    {
        auto rayIntersection = utils::trace_ray_until_condition
        (
            utils::global_point_to_grid_point(ray.position, lpm),
         angle_to_point(ray.position, ray.endpoint),
         ray.range,
         lpm,
         [](const hssh::LocalPerceptualMap& lpm, Point<int> cell) {
             return lpm.getCellType(cell) & hssh::kLimitedVisibilityOccGridCell;
         });

        rayIntersectsGlass = lpm.getCellType(rayIntersection) & hssh::kLimitedVisibilityOccGridCell;
    }

    return rayIntersectsGlass;
}


std::vector<LaserObject> ObjectDetector::segmentScan(const laser::MovingLaserScan& scan, 
                                                     const pose_distribution_t& robotPose)
{
    // LaserObjects are created via a vector of endpoints a la cartesian_laser_scan_t, so create those for the
    // moving laser scan
    laserEndpoints_.clear();
    laserEndpoints_.resize(scan.size());
    std::transform(scan.begin(), scan.end(), laserEndpoints_.begin(), [](const laser::adjusted_ray_t& ray) {
        return ray.endpoint;
    });
    
    std::vector<LaserObject> objects;
    
    for(std::size_t n = 1; n < scan.size(); ++n)
    {
        std::size_t clusterStart = n - 1;
        std::size_t clusterEnd   = n;
        
        const auto& ray = scan[n];
        
        if((ray.range > params_.maxTrackingDistance) || (ray.range < 0))
        {
            continue;
        }
        
        // Expand the cluster until consecutive rays are too far apart
        for(std::size_t m = n; (m < scan.size()) && isDynamic_[m]; ++m)
        {
            if(std::abs(scan[m-1].range - scan[m].range) < params_.maxAdjacentRayDistance)
            {
                clusterEnd = m + 1;
            }
            else
            {
                break;
            }
        }
        
        if(clusterEnd - clusterStart >= params_.minClusterSize)
        {
            double xyUncertainty = robotPose.uncertainty(0, 0) + robotPose.uncertainty(1, 1) 
                + (robotPose.uncertainty(2, 2) * ray.range);   // magnify the uncertainty a lot for far away points when
                // orientation becomes very uncertain
            
            objects.emplace_back(scan.timestamp(),
                                 laserEndpoints_.begin() + clusterStart,
                                 laserEndpoints_.begin() + clusterEnd,
                                 ray.position,
                                 params_.laserVariance + xyUncertainty);
        }
        
        n = clusterEnd; // jump to evaluating starting at the end of the cluster so the same cluster isn't duplicated
    }
    
    return objects;
}


LaserObjectCollection ObjectDetector::mergeCollections(const std::vector<LaserObjectCollection>& collections)
{
    // Merge the collections by constructing an adjacent graph amongst all related laser objects
    // Each connected component is a unique laser object
    AdjGraphType matches;
    AdjGraphIds ids;
    
    // Create a vertex for each laser object in the map
    for(auto& c : collections)
    {
        for(auto& obj : c)
        {
            ids[&obj] = add_vertex(AdjGraphVertex{&obj}, matches);
        }
    }

    // Establish the edges between the vertices
    for(std::size_t n = 0; n < collections.size(); ++n)
    {
        for(std::size_t m = n + 1; m < collections.size(); ++m)
        {
            match_collections(collections[n], collections[m], ids, matches);
        }
    }
    
    // Once the edges are added, find the connected components in the graph
    std::vector<int> components(ids.size());
    int numComponents = connected_components(matches, &components[0]);
    
    // Create the subgraphs from the connected components
    std::vector<std::vector<const LaserObject*>> adjObjects(numComponents);
    for(auto& objToId : ids)
    {
        // The component is stored in the particular index
        int component = components[objToId.second];
        adjObjects[component].push_back(objToId.first);
    }
    
    // Create the final objects from the components
    std::vector<LaserObject> finalObjects;
    for(auto& object : adjObjects)
    {
        finalObjects.emplace_back(object);
    }
    
    return LaserObjectCollection(std::move(finalObjects), 
                                 3, 
                                 collections.front().timestamp());
}


bool neighbor_is_occupied(const Point<int>& cell, const hssh::LocalPerceptualMap& lpm)
{
    const int kMaxNeighborDist = 2;

    for(int y = -kMaxNeighborDist; y <= kMaxNeighborDist; ++y)
    {
        for(int x = -kMaxNeighborDist; x <= kMaxNeighborDist; ++x)
        {
            if(lpm.getCellType(Point<int>(cell.x+x, cell.y+y)) &
                (hssh::kUnsafeOccGridCell | hssh::kUnknownOccGridCell))
            {
                return true;
            }
        }
    }

    return false;
}


void match_collections(const LaserObjectCollection& lhs, 
                       const LaserObjectCollection& rhs, 
                       const AdjGraphIds& ids,
                       AdjGraphType& matches)
{
    for(auto& lhsObj : lhs)
    {
        auto lhsBoundary = lhsObj.minErrorBoundary();
        
        for(auto& rhsObj : rhs)
        {
            auto rhsBoundary = rhsObj.minErrorBoundary();
            auto lhsDist = lhsBoundary.distanceToObject(rhsObj.begin(), rhsObj.end());
            auto rhsDist = rhsBoundary.distanceToObject(lhsObj.begin(), lhsObj.end());
            if((std::get<0>(lhsDist) < 0.1) || (std::get<0>(rhsDist) < 0.1))
            {
//                 std::cout << "Matched " << lhsBoundary.type() << " to " << rhsBoundary.type() << " at "
//                     << lhsObj.circleApproximation() << " and " << rhsObj.circleApproximation()
//                     << " lhs dist:" << std::get<0>(lhsDist) << ',' << std::get<1>(lhsDist)
//                     << " rhs dist:" << std::get<0>(rhsDist) << ',' << std::get<1>(rhsDist) << '\n';
                add_edge(ids.at(&rhsObj), ids.at(&lhsObj), matches);
            }
        }
    }
}
    
} // namespace tracker
} // namespace vulcan 

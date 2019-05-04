/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     beam_star_builder.cpp
* \author   Collin Johnson
*
* Definition of BeamStarBuilder.
*/

#include <hssh/local_topological/area_detection/labeling/beam_star_builder.h>
#include <hssh/local_topological/small_scale_star.h>
#include <core/line.h>
#include <utils/stub.h>
#include <algorithm>
#include <iostream>

// #define DEBUG_INTERSECTIONS
// #define DEBUG_FRAGMENTS
// #define DEBUG_FINAL_STAR

namespace vulcan
{
namespace hssh
{

struct gateway_endpoints_t
{
    Gateway          gateway;
    Line<double> endpointNormals[2];

    int  endpointIndex;         // index of the endpoint in the array of endpoints
    int  otherEndpointIndex;    // index of the other half of the path fragment
    bool ambiguousGateway;      // flag indicating the gateway has been found to belong to more than one path
                                // and therefore should belong to no paths
};

// Helper functions to handle the actual construction of the star
gateway_endpoints_t create_gateway_endpoints(const Gateway& gateway);

bool   gateways_intersect           (const gateway_endpoints_t& first,  const gateway_endpoints_t& second, float similarityThreshold);
bool   beam_intersects_gateway      (const gateway_endpoints_t& beam,   const Line<double>&  gateway);
double calculate_intersection_value (const Line<double>&  normal, const Line<double>&  gateway);
bool   gateway_angles_are_similar   (const gateway_endpoints_t& first,  const gateway_endpoints_t& second, float similarityThreshold);
void   connect_intersecting_gateways(gateway_endpoints_t& first,        gateway_endpoints_t&       second);

std::vector<local_path_fragment_t> create_path_fragments_from_endpoints(const std::vector<gateway_endpoints_t>& endpoints);
Point<double>                center_of_gateways                  (const std::vector<gateway_endpoints_t>& endpoints);
void                               create_endpoint_fragments           (int                                     index,
                                                                        const std::vector<gateway_endpoints_t>& endpoints,
                                                                        const Point<double>&              center,
                                                                        std::vector<local_path_fragment_t>&     fragments);

float            calculate_fragment_orientation     (int index, const std::vector<gateway_endpoints_t>& endpoints);
path_fragment_direction_t calculate_single_fragment_direction(const gateway_endpoints_t& endpoint, const Point<double>& center);
path_fragment_direction_t calculate_two_fragment_direction   (const gateway_endpoints_t& endpoint, const gateway_endpoints_t& otherEndpoint);

void print_fragment(const local_path_fragment_t& fragment);

inline bool path_is_ambiguous(const gateway_endpoints_t& gateway, const std::vector<gateway_endpoints_t>& endpoints)
{
    return (gateway.otherEndpointIndex != -1) &&
           (gateway.ambiguousGateway ||
            endpoints[gateway.otherEndpointIndex].ambiguousGateway);
}


BeamStarBuilder::BeamStarBuilder(const beam_star_builder_params_t& params)
    : params(params)
{
}


SmallScaleStar BeamStarBuilder::buildStar(const std::vector<Gateway>& gateways) const
{
    if(gateways.empty())
    {
        return SmallScaleStar();
    }
    
    std::vector<gateway_endpoints_t> endpoints(gateways.size());

    int currentGateway = 0;
    for(auto& gateway : gateways)
    {
        endpoints[currentGateway]               = create_gateway_endpoints(gateway);
        endpoints[currentGateway].endpointIndex = currentGateway;
    }

    for(std::size_t n = 0; n < endpoints.size(); ++n)
    {
        for(std::size_t i = n + 1; i < endpoints.size(); ++i)
        {
            if(gateways_intersect(endpoints[n], endpoints[i], params.angleDeviationThreshold))
            {
                connect_intersecting_gateways(endpoints[n], endpoints[i]);
            }
        }
    }

    return SmallScaleStar(create_path_fragments_from_endpoints(endpoints));
}


SmallScaleStar BeamStarBuilder::buildStar(const std::vector<Gateway>&   gateways, 
                                          const Point<float>&     center,
                                          const math::Rectangle<float>& boundary) const
{
    return buildStar(gateways);
}


int BeamStarBuilder::numGatewaysAlignedToAxis(const std::vector<Gateway>& gateways, double axisDirection) const
{
    std::cerr << "STUB! BeamStarBuilder::areGatewaysAlignedToAxis() \n";
    return 0;
}


bool BeamStarBuilder::areGatewaysAligned(const Gateway& lhs, const Gateway& rhs, const Point<float>& center) const
{
    PRINT_STUB("BeamStarBuilder::areGatewaysAligned");
    return false;
}


gateway_endpoints_t create_gateway_endpoints(const Gateway& gateway)
{
    Point<double> gatewayDirection = gateway.boundary().b - gateway.boundary().a;

    // To get the two normal endpoints, rotate the direction of the gateway by pi/2, equivalent to subtracting dir.y
    // from the x coordinate and adding dir.x to the y coordinate
    gateway_endpoints_t endpoint;
    endpoint.gateway = gateway;

    endpoint.endpointNormals[0].a   = endpoint.gateway.boundary().a;
    endpoint.endpointNormals[0].b.x = endpoint.gateway.boundary().a.x - gatewayDirection.y;
    endpoint.endpointNormals[0].b.y = endpoint.gateway.boundary().a.y + gatewayDirection.x;

    endpoint.endpointNormals[1].a   = endpoint.gateway.boundary().b;
    endpoint.endpointNormals[1].b.x = endpoint.gateway.boundary().b.x - gatewayDirection.y;
    endpoint.endpointNormals[1].b.y = endpoint.gateway.boundary().b.y + gatewayDirection.x;

    endpoint.otherEndpointIndex = -1;
    endpoint.ambiguousGateway   = false;

    return endpoint;
}


bool gateways_intersect(const gateway_endpoints_t& first, const gateway_endpoints_t& second, float similarityThreshold)
{
    // Intersection only occurs if beams of both gateways intersect each other and the angles are close enough to one another
    return beam_intersects_gateway(first, second.gateway.boundary()) &&
           beam_intersects_gateway(second, first.gateway.boundary()) &&
           gateway_angles_are_similar(first, second, similarityThreshold);
}


bool beam_intersects_gateway(const gateway_endpoints_t& beam, const Line<double>& gateway)
{
    /*
    * One method of calculating the intersection of lines A and B yields a number. If the number is in [0, 1], then
    * A intersects B within the segment of B. <0 is off one end of B, >1 is off the other end. For a beam to intersect,
    * A has to intersect with B directly, or one endpoint has to fail to intersect on one side of B and the other
    * endpoint on the other end. This condition implies the gateway lies entirely within the extent of the beam.
    */

    double beamA = calculate_intersection_value(beam.endpointNormals[0], gateway);
    double beamB = calculate_intersection_value(beam.endpointNormals[1], gateway);

    return (beamA >  1.0 && beamB <  0.0) ||
           (beamA <  0.0 && beamB >  1.0) ||
           (beamA >= 0.0 && beamA <= 1.0) ||
           (beamB >= 0.0 && beamB <= 1.0);
}

// Parallel lines are given a value of HUGE_VAL
double calculate_intersection_value(const Line<double>& normal, const Line<double>& gateway)
{
    // Can use the line intersection data to handle the calculation here. The value needed here is sNum/den
    // because the beam is checking for the projection of the normal onto this line

    line_intersection_data_t<double> intersection(normal, gateway);

    if(intersection.den == 0)
    {
        return HUGE_VAL;
    }
    else
    {
        return intersection.sNum / intersection.den;
    }
}


bool gateway_angles_are_similar(const gateway_endpoints_t& first, const gateway_endpoints_t& second, float similarityThreshold)
{
    float firstAngle  = fabs(atan(slope(first.gateway.boundary())));
    float secondAngle = fabs(atan(slope(second.gateway.boundary())));

    return fabs(firstAngle - secondAngle) < similarityThreshold;
}


void connect_intersecting_gateways(gateway_endpoints_t& first, gateway_endpoints_t& second)
{
    // A gateway is ambiguous if it belongs to more than one path. If a gateway intersects multiple
    // other gateways, then it must belong to multiple paths. This is indicated by having otherEndpointIndex != -1
    if(first.otherEndpointIndex == -1)
    {
        first.otherEndpointIndex = second.endpointIndex;
    }
    else
    {
        first.ambiguousGateway = true;
    }

    if(second.otherEndpointIndex == -1)
    {
        second.otherEndpointIndex = first.endpointIndex;
    }
    else
    {
        second.ambiguousGateway = true;
    }

#ifdef DEBUG_INTERSECTIONS
    std::cout<<"DEBUG:SSStarBuilder:Connect:"<<first.endpointIndex<<"->"<<second.endpointIndex<<" ambiguous:"<<(first.ambiguousGateway || second.ambiguousGateway)<<'\n';
#endif
}


std::vector<local_path_fragment_t> create_path_fragments_from_endpoints(const std::vector<gateway_endpoints_t>& endpoints)
{
    std::vector<local_path_fragment_t> fragments;

    std::vector<bool>   madePath(endpoints.size(), false);
    int8_t              currentIndex = 0;
    Point<double> centerPoint  = center_of_gateways(endpoints);

    for(size_t n = 0; n < endpoints.size(); ++n)
    {
        if(!madePath[n])
        {
            create_endpoint_fragments(n, endpoints, centerPoint, fragments);

            madePath[n] = true;

            // If the endpoint was part of a two gateway path, then mark the other endpoint
            // as having made a path to ensure duplicate path fragments are not created
            if(endpoints[n].otherEndpointIndex != -1 && !path_is_ambiguous(endpoints[n], endpoints))
            {
                madePath[endpoints[n].otherEndpointIndex] = true;
            }

            // Set the index for the newly added fragments
            (fragments.end()-1)->pathId = currentIndex;
            (fragments.end()-2)->pathId = currentIndex++;
        }
    }

    std::sort(fragments.begin(), fragments.end());

#ifdef DEBUG_FINAL_STAR
    std::cout<<"DEBUG:StarBuilder:Final star:";
    for(auto fragIt = fragments.begin(), fragEnd = fragments.end(); fragIt != fragEnd; ++fragIt)
    {
        std::cout<<(int)fragIt->pathId<<':'<<fragIt->navigable<<' ';
    }
    std::cout<<'\n';
#endif

    return fragments;
}


Point<double> center_of_gateways(const std::vector<gateway_endpoints_t>& endpoints)
{
    Point<double> center;

    for(auto endpointIt = endpoints.begin(), endpointEnd = endpoints.end(); endpointIt != endpointEnd; ++endpointIt)
    {
        center += endpointIt->gateway.center();
    }

    // Two gateway coordinates per endpoint
    return Point<double>(center.x / endpoints.size(), center.y / endpoints.size());
}


void create_endpoint_fragments(int                                     index,
                               const std::vector<gateway_endpoints_t>& endpoints,
                               const Point<double>&              center,
                               std::vector<local_path_fragment_t>&     fragments)
{
    const gateway_endpoints_t& endpoint = endpoints[index];

    local_path_fragment_t fragment;
    local_path_fragment_t otherFragment;

    fragment.gateway = endpoints[index].gateway;

    if((endpoint.otherEndpointIndex == -1) || path_is_ambiguous(endpoint, endpoints))
    {
        fragment.direction        = calculate_single_fragment_direction(endpoint, center);
        otherFragment.navigable = false;
    }
    else // have a complete path
    {
        otherFragment.gateway   = endpoints[endpoint.otherEndpointIndex].gateway;
        otherFragment.navigable = true;

        fragment.direction = calculate_two_fragment_direction(endpoint, endpoints[endpoint.otherEndpointIndex]);
    }

    fragment.navigable    = true;
    otherFragment.direction = (fragment.direction == PATH_FRAGMENT_PLUS) ? PATH_FRAGMENT_MINUS : PATH_FRAGMENT_PLUS;

    fragment.pathId      = index;
    otherFragment.pathId = index;

    fragments.push_back(fragment);
    fragments.push_back(otherFragment);

#ifdef DEBUG_FRAGMENTS
    std::cout<<"DEBUG:SSStarBuilder:Fragments:";
    print_fragment(fragment);
    std::cout<<"\n                              ";
    print_fragment(otherFragment);
    std::cout<<'\n';
#endif
}


float calculate_fragment_orientation(int index, const std::vector<gateway_endpoints_t>& endpoints)
{
    const gateway_endpoints_t& endpoint = endpoints[index];

    float deltaX = 0.0f;
    float deltaY = 0.0f;

    deltaX += endpoint.endpointNormals[0].b.x - endpoint.endpointNormals[0].a.x;
    deltaY += endpoint.endpointNormals[0].b.y - endpoint.endpointNormals[0].a.y;

    if((endpoint.otherEndpointIndex != -1) && !endpoint.ambiguousGateway)
    {
        const gateway_endpoints_t& otherEndpoint = endpoints[endpoint.otherEndpointIndex];

        deltaX += otherEndpoint.endpointNormals[0].b.x - otherEndpoint.endpointNormals[0].a.x;
        deltaY += otherEndpoint.endpointNormals[0].b.y - otherEndpoint.endpointNormals[0].a.y;
    }

    // Make sure no divide by 0 happens
    if(deltaX == 0.0f)
    {
        return (deltaY >= 0) ? M_PI/2 : -M_PI/2;
    }
    else
    {
        return atan(deltaY / deltaX);
    }
}


path_fragment_direction_t calculate_single_fragment_direction(const gateway_endpoints_t& endpoint, const Point<double>& center)
{
    // The direction is assigned such that the right most part of the path is given a positive direction.
    // In the case of a single fragment, nothing to compare against, so just look at the center of the gateways for comparison
    // The direction isn't particularly important, it's only real function is to distinguish between the two fragments on the same path
    // More of an identifier than a direction
    return (endpoint.gateway.center().x >= center.x) ? PATH_FRAGMENT_PLUS : PATH_FRAGMENT_MINUS;
}


path_fragment_direction_t calculate_two_fragment_direction(const gateway_endpoints_t& endpoint, const gateway_endpoints_t& otherEndpoint)
{
    // The PLUS direction is the rightmost gateway. If the gateways are stacked vertically, the one with a larger y value is used
    double gatewayCenterX = endpoint.gateway.center().x;
    double otherCenterX   = otherEndpoint.gateway.center().x;

    if(gatewayCenterX > otherCenterX)
    {
        return PATH_FRAGMENT_PLUS;
    }
    else if(gatewayCenterX < otherCenterX)
    {
        return PATH_FRAGMENT_MINUS;
    }
    else // gateways stacked vertically
    {
        double gatewayCenterY = endpoint.gateway.center().y;
        double otherCenterY   = otherEndpoint.gateway.center().y;

        return (gatewayCenterY >= otherCenterY) ? PATH_FRAGMENT_PLUS : PATH_FRAGMENT_MINUS;
    }
}


void print_fragment(const local_path_fragment_t& fragment)
{
    std::cout<<"Angle:"<<fragment.gateway.direction()<<" Dir:"<<fragment.direction<<" Traverse:"<<fragment.navigable;
}


} // namespace hssh
} // namespace vulcan

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     isovist_voronoi_gateway_generator.h
* \author   Collin Johnson
* 
* Declaration of IsovistVoronoiGatewayGenerator.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISOVIST_VORONOI_GATEWAY_GENERATOR_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISOVIST_VORONOI_GATEWAY_GENERATOR_H

#include <hssh/local_topological/area_detection/gateways/generator.h>
#include <hssh/local_topological/area_detection/gateways/isovist_maxima.h>
#include <hssh/local_topological/area_detection/voronoi/voronoi_utils.h>
#include <hssh/local_topological/gateway.h>
#include <hssh/local_topological/params.h>
#include <hssh/types.h>
#include <boost/optional.hpp>
#include <vector>

namespace vulcan
{
namespace utils { class ConfigFile; }
namespace hssh
{

class  EndpointValidator;
class  Gateway;
class  VoronoiSkeletonGrid;
struct isovist_local_maximum_t;

const std::string kIsovistVoronoiGatewayGeneratorType("voronoi");

/**
* IsovistVoronoiGatewayGenerator generates Gateways from isovist_local_maximum_t. One Gateway exists for each local maximum.
* The generation process for the isovists guarantees that the gateway crosses the skeleton in the direction of the gradient.
* 
* When generating the gateways, gateways for all maxima are generated at the same time to ensure they are consistent with one
* another. In particular, gateways can't be intersecting, so if they intersect, then get rid of them.
*/
class IsovistVoronoiGatewayGenerator : public GatewayGenerator
{
public:
    
    /**
    * Constructor for IsovistVoronoiGatewayGenerator.
    *
    * \param    params          Parameters for the gateway generator
    */
    IsovistVoronoiGatewayGenerator(const isovist_voronoi_gateway_generator_params_t& params);
    

    // GatewayGenerator interface
    std::vector<WeightedGateway> generateGateways(const std::vector<WeightedGateway>& priorGateways,
                                                  const VoronoiIsovistField& isovists,
                                                  const VoronoiSkeletonGrid& grid,
                                                  const EndpointValidator& validator) override;
    
private:
    
    using MaximumSources = std::pair<cell_t, cell_t>;
    
    struct proposed_gateway_t
    {
        MaximumSources sources;
        cell_t         skeleton;
        double         length;
        double         angle;
        double         gradient;
        
        proposed_gateway_t(const MaximumSources& sources, cell_t skeleton)
        : sources(sources)
        , skeleton(skeleton)
        , length(distance_between_points(sources.first, skeleton) + distance_between_points(sources.second, skeleton))
        , angle(std::abs(angle_between_points(sources.first, sources.second, skeleton)))
        , gradient(1.0)
        {
        }

        double score(void) const { return gradient * angle / length; }
        bool operator<(const proposed_gateway_t& rhs) const { return score() < rhs.score(); }
    };

    int32_t                    nextGatewayId_;
    const VoronoiIsovistField* isovists_;
    const VoronoiSkeletonGrid* grid_;
    const EndpointValidator*   validator_;
    
    SourceToCellsMap sourceToCells_;
    std::vector<WeightedGateway> initialGateways_;
    
    // Cached memory for storing the sources and edge cells for the maximum under investigation
    std::vector<proposed_gateway_t> proposals_;
    CellVector sourceCells_;
    CellVector edgeCells_;
    
    isovist_voronoi_gateway_generator_params_t params_;
    
    bool createGatewayForMaximumIfValid(const isovist_local_maximum_t& maximum);
    void proposeGatewaysForMaximum(const isovist_local_maximum_t& maximum);
    boost::optional<proposed_gateway_t> proposeGatewayForSkeletonCell(cell_t skeleton);
    boost::optional<proposed_gateway_t> createProposalForSource(cell_t source, cell_t skeleton);
    bool isInvalidSkeletonCell(cell_t skeleton);
    bool isValidProposal(cell_t skeleton, cell_t sourceA, cell_t sourceB);
    Gateway toGateway(const proposed_gateway_t& proposed);
};
    
} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISOVIST_VORONOI_GATEWAY_GENERATOR_H

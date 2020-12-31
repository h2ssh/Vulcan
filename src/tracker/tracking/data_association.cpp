/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     data_association.cpp
 * \author   Collin Johnson
 *
 * Definition of create_data_association_strategy factory.
 */

#include "tracker/tracking/data_association.h"
#include "tracker/tracking/max_likelihood_association.h"
#include "tracker/tracking/nearest_neighbor_association.h"

namespace vulcan
{
namespace tracker
{

std::unique_ptr<DataAssociationStrategy> create_data_association_strategy(const std::string& type,
                                                                          const utils::ConfigFile& config)
{
    if (type == kNearestNeighborAssociationType) {
        nearest_neighbor_params_t params(config);
        return std::unique_ptr<DataAssociationStrategy>(new NearestNeighborAssociation(params));
    }

    std::cerr << "ERROR: create_data_association_strategy: Unknown strategy type:" << type << '\n';
    assert(false);
    return std::unique_ptr<DataAssociationStrategy>();
}

}   // namespace tracker
}   // namespace vulcan

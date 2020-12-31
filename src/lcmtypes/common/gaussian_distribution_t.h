/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_GAUSSIAN_DISTRIBUTION_T_H
#define LCMTYPES_GAUSSIAN_DISTRIBUTION_T_H

#include "lcmtypes/vulcan_lcm_gaussian_distribution_t.h"
#include <string>

namespace vulcan
{
class MultivariateGaussian;

namespace lcm
{

MultivariateGaussian convert_lcm_to_vulcan(const vulcan_lcm_gaussian_distribution_t& gaussianMessage); 
void                       convert_lcm_to_vulcan(const vulcan_lcm_gaussian_distribution_t& gaussianMessage, MultivariateGaussian& gaussian);
void                       convert_vulcan_to_lcm(const MultivariateGaussian& gaussian, vulcan_lcm_gaussian_distribution_t& gaussianMessage);

void free_gaussian_message(vulcan_lcm_gaussian_distribution_t& gaussianMessage);

}
}

#endif // LCMTYPES_GAUSSIAN_DISTRIBUTION_T_H

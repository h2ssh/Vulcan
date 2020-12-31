/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "lcmtypes/common/gaussian_distribution_t.h"
#include "core/matrix.h"
#include "core/multivariate_gaussian.h"
#include "core/vector.h"
#include "lcmtypes/common/matrix_t.h"
#include "lcmtypes/common/vector_t.h"
#include "lcmtypes/subscription_manager.h"
#include <cassert>


namespace vulcan
{
namespace lcm
{

MultivariateGaussian convert_lcm_to_vulcan(const vulcan_lcm_gaussian_distribution_t& gaussianMessage)
{
    Vector mean(gaussianMessage.mean.length);
    Matrix covariance(gaussianMessage.covariance.num_rows, gaussianMessage.covariance.num_columns);

    lcm_vector_to_vulcan_vector(gaussianMessage.mean, mean);
    lcm_matrix_to_vulcan_matrix(gaussianMessage.covariance, covariance);

    return MultivariateGaussian(mean, covariance);
}

void convert_lcm_to_vulcan(const vulcan_lcm_gaussian_distribution_t& gaussianMessage, MultivariateGaussian& gaussian)
{
    Vector mean(gaussianMessage.mean.length);
    Matrix covariance(gaussianMessage.covariance.num_rows, gaussianMessage.covariance.num_columns);

    lcm_vector_to_vulcan_vector(gaussianMessage.mean, mean);
    lcm_matrix_to_vulcan_matrix(gaussianMessage.covariance, covariance);

    gaussian.setDistributionStatistics(mean, covariance);
}


void convert_vulcan_to_lcm(const MultivariateGaussian& gaussian, vulcan_lcm_gaussian_distribution_t& gaussianMessage)
{
    Vector mean = gaussian.getMean();
    Matrix covariance = gaussian.getCovariance();

    allocate_lcm_vector(gaussianMessage.mean, mean.n_rows);
    allocate_lcm_matrix(gaussianMessage.covariance, covariance.n_rows, covariance.n_cols);

    vulcan_vector_to_lcm_vector(mean, gaussianMessage.mean);
    vulcan_matrix_to_lcm_matrix(covariance, gaussianMessage.covariance);
}


void free_gaussian_message(vulcan_lcm_gaussian_distribution_t& gaussianMessage)
{
    free_lcm_vector(gaussianMessage.mean);
    free_lcm_matrix(gaussianMessage.covariance);
}

}   // namespace lcm
}   // namespace vulcan

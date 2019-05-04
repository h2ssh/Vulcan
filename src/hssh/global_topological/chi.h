/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     chi.h
* \author   Collin Johnson
*
* Declaration of Chi.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_CHI_H
#define HSSH_GLOBAL_TOPOLOGICAL_CHI_H

#include <hssh/local_topological/lambda.h>
#include <hssh/utils/id.h>
#include <core/pose_distribution.h>
#include <cereal/access.hpp>
#include <cereal/types/unordered_map.hpp>

namespace vulcan
{
namespace hssh
{

class TopologicalMap;

/**
* Chi represents the optimized layout of the places in a topological map hypothesis. Each
* place is assigned a pose in a global reference frame. The optimized lambda between any two
* places can be calculated.
*
* Chi contains an error value p(Chi | Lambda), which is related to the remaining strain in the
* graph after the optimization is completed.
*/
class Chi
{
public:

    /**
    * Default constructor for Chi.
    */
    Chi(void) = default;

    /**
    * Constructor for Chi.
    *
    * Creates a Chi from an unoptimized map.
    *
    * When created in this fashion, the assumption is that no optimization has taken place and
    * therefore the Chi is exactly correct.
    */
    Chi(const TopologicalMap& map);

    /**
    * Constructor for Chi.
    *
    * Create a Chi with the specified place poses and covariance.
    *
    * \param    poses               Poses for the places
    * \param    logLikelihood       p(Chi | Lambda) -- related to the error
    */
    Chi(const std::unordered_map<Id, pose_distribution_t>& poses, double logLikelihood);

    // Mutators

    /**
    * setPlacePose sets the pose for a particular place.
    *
    * \param    placeId         Id of the place
    * \param    pose            Pose to set
    */
    void setPlacePose(Id placeId, const pose_distribution_t& pose);

    /**
    * setLogLikelihood sets the log-likelihood for the Chi.
    *
    * \param    logLikelihood           Log-likelihood of the Chi
    */
    void setLogLikelihood(double logLikelihood);

    // Accessors

    /**
    * getPlacePose retrieves the pose for the specified place.
    *
    * \param    placeId         Id of the place
    * \return   Pose of the place in the layout. If such a place doesn't exist, (0,0,0) is returned.
    */
    pose_distribution_t getPlacePose(Id placeId) const;

    /**
    * getLambda retrieves the Lambda value going from fromPlace to toPlace.
    *
    * \param    fromPlace       Start of the lambda
    * \param    toPlace         End of the lambda
    * \return   Lambda(from,to). If fromPlace or toPlace doesn't exist, then a zero-offset lambda is returned.
    */
    Lambda getLambda(Id fromPlace, Id toPlace) const;

    /**
    * getLogLikelihood retrieves the log-likelihood for the Chi. p(Chi | Lambda)
    *
    * \return   Log-likeliood of the layout.
    */
    double getLogLikelihood(void) const { return logLikelihood_; }

private:

    std::unordered_map<Id, pose_distribution_t> poses_;
    double logLikelihood_ = 0.0;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar( poses_,
            logLikelihood_
        );
    }
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_CHI_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     debug.h
* \author   Collin Johnson
* 
* Definition of DebugHypothesis.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREAS_DEBUG_H
#define HSSH_LOCAL_TOPOLOGICAL_AREAS_DEBUG_H

#include <hssh/local_topological/area_detection/labeling/hypothesis.h>
#include <hssh/local_topological/area_detection/labeling/type_distribution.h>
#include <hssh/local_topological/area_extent.h>
#include <core/point.h>
#include <array>
#include <cereal/types/vector.hpp>
#include <cassert>

namespace vulcan
{
namespace hssh
{
    
/**
* DebugHypothesis is a snapshot of the state of an AreaHypothesis. The DebugHypothesis is intended to be used
* for logging or visual debugging of the results of the area parsing process. The complete assignments can
* be saved as a collection of DebugHypotheses to allow the likelihood of the various assignments to be visualized
* and checked for correctness.
*/
class DebugHypothesis
{
public:
    
    using Endpoints   = std::array<Point<float>, 2>;
    using FeatureIter = std::vector<double>::const_iterator;
    
    /**
    * Constructor for DebugHypothesis.
    * 
    * Creates a DebugHypothesis for either a generic area or a place -- destination or decision point. The path
    * constructor should be used for paths because it allows for more detailed debugging information.
    * 
    * \pre  type != kPath
    * \param    type            Type of the place
    * \param    extent          Approximate extent
    * \param    eigRatio        Ratio of the eigenvalues
    * \param    eigDirection    Direction of the dominant eigenvector
    * \param    frontierRatio   Amount of the hypothesis that is a frontier
    * \param    likelihood      Likelihood of the hypothesis
    */
    DebugHypothesis(HypothesisType              type,
                    const AreaExtent&           extent,
                    HypothesisTypeDistribution  typeDistribution,
                    double                      eigRatio,
                    double                      eigDirection,
                    double                      frontierRatio,
                    const std::vector<double>&  featureValues)
    : type_(type)
    , extent_(extent)
    , distribution_(typeDistribution)
    , eigRatio_(eigRatio)
    , eigDirection_(eigDirection)
    , frontierRatio_(frontierRatio)
    , featureValues_(featureValues)
    {
    }
    
    /**
    * Constructor for DebugHypothesis.
    * 
    * Creates a DebugHypothesis for a path.
    * 
    * \param    endpoints       Endpoints of the path
    * \param    extent          Approximate extent
    * \param    eigRatio        Ratio of the eigenvalues
    * \param    eigDirection    Direction of the dominant eigenvector
    * \param    frontierRatio   Amount of the hypothesis that is a frontier
    */
    DebugHypothesis(const Endpoints&            endpoints,
                    const AreaExtent&           extent,
                    HypothesisTypeDistribution  typeDistribution,
                    double                      eigRatio,
                    double                      eigDirection,
                    double                      frontierRatio,
                    const std::vector<double>&  featureValues)
    : type_(HypothesisType::kPath)
    , extent_(extent)
    , distribution_(typeDistribution)
    , eigRatio_(eigRatio)
    , eigDirection_(eigDirection)
    , frontierRatio_(frontierRatio)
    , endpoints_(endpoints)
    , featureValues_(featureValues)
    {
    }
    
    // Observers
    HypothesisType getType(void)          const { return type_; }
    Endpoints      getPathEndpoints(void) const { assert(type_ == HypothesisType::kPath); return endpoints_; }
    const AreaExtent& getExtent(void)     const { return extent_; }
    HypothesisTypeDistribution getDistribution(void) const { return distribution_; }
    double         getEigRatio(void)      const { return eigRatio_; }
    double         getEigDirection(void)  const { return eigDirection_; }
    double         getFrontierRatio(void) const { return frontierRatio_; }
    
    double      numFeatures(void)               const { return featureValues_.size(); }
    double      featureValue(std::size_t index) const { return (index < featureValues_.size()) ? featureValues_[index] : 0.0; }
    FeatureIter featureBegin(void)              const { return featureValues_.begin(); }
    FeatureIter featureEnd(void)                const { return featureValues_.end();   }
    
    DebugHypothesis(void) { }  // private constructor for serialization
    
private:
    
    friend class AreaHypothesis;
    
    HypothesisType      type_;
    AreaExtent          extent_;
    HypothesisTypeDistribution distribution_;
    double              eigRatio_;
    double              eigDirection_;
    double              frontierRatio_;
    Endpoints           endpoints_;
    std::vector<double> featureValues_;
    
    // Serialization support
    friend class ::cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( type_,
            extent_,
            distribution_.decision,
            distribution_.destination,
            distribution_.path,
            eigRatio_,
            eigDirection_,
            frontierRatio_,
            endpoints_[0],
            endpoints_[1],
            featureValues_);
    }
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREAS_DEBUG_H

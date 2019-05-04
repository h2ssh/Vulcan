/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     boundary_classifier.h
* \author   Collin Johnson
*
* Declaration of BoundaryClassifier.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_BOUNDARY_CLASSIFIER_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_BOUNDARY_CLASSIFIER_H

#include <hssh/local_topological/area_detection/labeling/type_distribution.h>
#include <core/matrix.h>
#include <memory>
#include <string>

namespace vulcan
{
namespace hssh
{

class AreaHypothesisBoundary;
class LabeledBoundaryData;

/**
* BoundaryClassifier is a classifier (really a probabilistic model) of the area hypothesis boundaries. The learned
* model is a simple Dirichlet distribution that models the frequency of different types of boundaries existing in
* the environment.
*/
class BoundaryClassifier
{
public:

    /**
    * LearnClassifier learns the probability distribution for the provided examples created from labeled maps.
    *
    * \param    examples            Examples of boundaries
    * \return   A classifier that models the distribution of boundary types in the provided data.
    */
    static std::unique_ptr<BoundaryClassifier> LearnClassifier(const LabeledBoundaryData& examples);

    BoundaryClassifier(void) = default;

    /**
    * Constructor for BoundaryClassifier.
    *
    * \param    classifierName      Base name of the classifier file to be loaded
    */
    BoundaryClassifier(const std::string& classifierName);

    /**
    * classifyBoundary computes the probability of a particular boundary existing based on a simple learned
    * probabilistic model of which boundaries are most likely to occur in the environment.
    *
    * \param    typeA       Type on one side of the boundary
    * \param    typeB       Type on the other side of the boundary
    * \param    isOn        Is the gateway on?
    * \return   Prior probability of this combination of type and gateway on/offness.
    */
    double classifyBoundary(HypothesisType typeA, HypothesisType typeB, bool isOn) const;
    double classifyBoundaryLog(HypothesisType typeA, HypothesisType typeB, bool isOn) const;

    /**
    * boundaryDistribution retrieves the underlying distribution used for classifying the boundaries.
    */
    BoundaryTypeDistribution boundaryDistribution(void) const;

    /**
    * save saves the parameters used by the classifier to the specified file.
    *
    * \param    filename            Name of the file in which the classifier should be saved
    * \return   True if saving was successful.
    */
    bool save(const std::string& filename) const;

    /**
    * load loads the parameters used by the classifier from the specified file.
    *
    * \param    filename            Name of the file from which to load the stored classifier
    * \return   True if loading was successful.
    */
    bool load(const std::string& filename);

private:

    BoundaryTypeDistribution dist_;
    BoundaryTypeDistribution logDist_;

    BoundaryClassifier(const Matrix& posModel, const Matrix& negModel);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_BOUNDARY_CLASSIFIER_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     csp_solution.h
* \author   Collin Johnson
*
* Declaration of AreaLabel and CSPSolution.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_CSP_SOLUTION_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_CSP_SOLUTION_H

#include "hssh/local_topological/area_detection/labeling/hypothesis_type.h"
#include "hssh/local_topological/area_detection/labeling/type_distribution.h"
#include <vector>

namespace vulcan
{
namespace hssh
{

class AreaHypothesis;
class HypothesisGraph;

/**
* AreaLabel defines the hypotheses that form a single area in the final graph, along with the label to apply to this
* area.
*/
struct AreaLabel
{
    std::vector<AreaHypothesis*> areaHypotheses;
    HypothesisType label;
    HypothesisTypeDistribution distribution;        // distribution across possible label types

    AreaLabel(void)
    : label(HypothesisType::kArea)
    {
    }

    AreaLabel(const std::vector<AreaHypothesis*>& areas, HypothesisType label, HypothesisTypeDistribution dist)
    : areaHypotheses(areas)
    , label(label)
    , distribution(dist)
    {
    }
};


/**
* CSPSolution defines the solution to a CSP. The solution can be applied to the graph from which it was created.
*/
class CSPSolution
{
public:

    enum ErrorCode
    {
        too_many_attempts,
        no_solution_with_all_areas_split,
        no_valid_solution,
        fixed_area_failing_constraints,
        successful,
    };

    CSPSolution(void) = default;

    /**
    * Constructor for CSPSolution.
    *
    * Create an error solution.
    */
    CSPSolution(ErrorCode error);

    /**
    * Constructor for CSPSolution.
    *
    * \param    labels          Final labels to be applied to the graph
    * \param    logProb         Log-probability of labels for the solution
    */
    CSPSolution(const std::vector<AreaLabel>& labels, double logProb);

    /**
    * apply applies this solution to the provided HypothesisGraph, which results in a new HypothesisGraph with
    * consistent labels being created.
    *
    * \param[in,out]    graph           Graph to apply the solution to
    */
    void apply(HypothesisGraph& graph);

    /**
    * logProb retrieves the log probability of the solution.
    */
    double logProb(void) const { return logProb_; }

    /**
    * errorCode retrieves the error code associated with the solution.
    */
    ErrorCode errorCode(void) const { return error_; }

    // Output operator:  success? SUCCESS: num areas  log prob  error? ERROR: error code
    friend std::ostream& operator<<(std::ostream& out, const CSPSolution& solution);

private:

    std::vector<AreaLabel> labels_;
    double logProb_;
    ErrorCode error_;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_CSP_SOLUTION_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     labeling_csp.cpp
* \author   Collin Johnson
*
* Definition of CSPSolution.
*/

#include <hssh/local_topological/area_detection/labeling/csp_solution.h>
#include <hssh/local_topological/area_detection/labeling/hypothesis_graph.h>
#include <hssh/local_topological/area_detection/labeling/hypothesis.h>

namespace vulcan
{
namespace hssh
{

CSPSolution::CSPSolution(ErrorCode error)
: error_(error)
{
}


CSPSolution::CSPSolution(const std::vector<AreaLabel>& labels, double logProb)
: labels_(labels)
, logProb_(logProb)
, error_(successful)
{
}


void CSPSolution::apply(HypothesisGraph& graph)
{
    // Apply all the labels
    for(auto& label : labels_)
    {
        for(auto& area : label.areaHypotheses)
        {
            area->setType(label.label);
            area->setTypeDistribution(label.distribution);
        }

        graph.mergeHypotheses(label.areaHypotheses, label.label);
    }
}


std::ostream& operator<<(std::ostream& out, const CSPSolution& solution)
{
    if(solution.error_ == CSPSolution::successful)
    {
        std::cout << "CSPSolution: SUCCESS! Num areas: " << solution.labels_.size() << " Log-prob: "
            << solution.logProb_;
    }
    else
    {
        std::cout << "CSPSolution: ERROR! ";
        switch(solution.error_)
        {
        case CSPSolution::too_many_attempts:
            std::cout << "too many attempts";
            break;
        case CSPSolution::no_solution_with_all_areas_split:
            std::cout << "no solution with all areas split";
            break;
        case CSPSolution::no_valid_solution:
            std::cout << "no valid solution";
            break;
        case CSPSolution::fixed_area_failing_constraints:
            std::cout << "fixed area with failing constraints";
            break;
        case CSPSolution::successful:
            std::cout << "successful";
            break;
        }
    }

    return out;
}

} // namespace hssh
} // namespace vulcan

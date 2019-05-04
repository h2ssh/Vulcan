/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     edit_distance.h
* \author   Collin Johnson
*
* Declaration of edit_distance function.
*/

#ifndef UTILS_EDIT_DISTANCE_H
#define UTILS_EDIT_DISTANCE_H

#include <string>

namespace vulcan
{
namespace utils
{

/**
* edit_distance_weights_t defines the weights to be used for the edit distance calculation.
*
* The default weights are set such that substitutions are never used because an insertion and then deletion is a
* lower cost.
*/
struct edit_distance_weights_t
{
    double insertion = 1.0;
    double deletion = 1.0;
    double substitution = 3.0;
};


/**
* edit_distance calculates the edit distance from string x to string y. The edit distance is the minimum cost
* transformation (insertions, deletions, substitutions) to transform x into y.
*
* More formal destinations of edit distance can be found all over the Internet. This function implements the classic
* dynamic programming solution to the problem.
*
* \param    x               Starting string
* \param    y               Goal string
* \param    weights         Weights to use for the various operations (optional)
* \param    outputChanges   Flag indicating if the changes made should be output to stdout (optional, default = false)
* \return   Edit distance for transforming x into y.
*/
double edit_distance(const std::string& x,
                     const std::string& y,
                     edit_distance_weights_t weights = edit_distance_weights_t(),
                     bool outputChanges = false);

}
}

#endif // UTILS_EDIT_DISTANCE_H

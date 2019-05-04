/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     liblinear.h
* \author   Collin Johnson
*
* Declaration of functions to make training with liblinear easy.
*/

#ifndef UTILS_LIBLINEAR_H
#define UTILS_LIBLINEAR_H

#include <core/matrix.h>
#include <core/vector.h>
#include <liblinear/linear.h>
#include <memory>
#include <vector>

namespace vulcan
{
namespace utils
{

/**
* LibLinearDeleter is a deleter function to use for storing linear_model in a shared_ptr.
*/
struct LibLinearDeleter
{
    void operator()(model* model) const { free_and_destroy_model(&model); }
};


using LinearModelPtr = std::unique_ptr<model, LibLinearDeleter>;
using LinearFeatures = std::vector<feature_node>;

/**
* learn_linear_classifier learns a classifier for the provided learning problem, which associates a feature vector with
* a label for the given features.
*
* The learning process uses L2_LR in liblinear. It performs parameter selection to determine the best parameter for the
* linear classifier via the find_c_parameter function.
*
* The features must be pre-scaled.
*
* The returned model uses the optimal parameters.
*
* \param    features            Features for the different classes -- each column is one set of features
* \param    labels              Labels for each of the features
* \pre  labels.n_rows == features.n_cols
* \return   The linear_model to use for classifying using liblinear.
*/
model* learn_linear_classifier(const Matrix& features, const IntMatrix& labels);

/**
* features_to_nodes converts a feature vector into corresponding feature_node values suitable for classification via
* an model*.
*
* The nodes are pushed back into the nodes vector. The vector isn't cleared.
*
* \param    features            Scaled features
* \param    nodes               Node vector in which to save the features
*/
void features_to_nodes(const Vector& features, LinearFeatures& nodes);

} // namespace utils
} // namespace vulcan

#endif // UTILS_SVM_H

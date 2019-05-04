/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     svm.h
* \author   Collin Johnson
* 
* Declaration of learn_svm_classifier.
*/

#ifndef UTILS_SVM_H
#define UTILS_SVM_H

#include <core/matrix.h>
#include <core/vector.h>
#include <libsvm/svm.h>
#include <memory>

namespace vulcan
{
namespace utils
{
    
/**
* SVMDeleter is a deleter function to use for storing svm_model in a shared_ptr.
*/
struct SVMDeleter
{
    void operator()(svm_model* model) const { svm_free_and_destroy_model(&model); }
};

using SVMModelPtr = std::unique_ptr<svm_model, SVMDeleter>;
using SVMFeatures = std::vector<svm_node>;

/**
* learn_svm_classifier learns a classifier for the provided learning problem, which associates a feature vector with
* a label for the given features.
* 
* The learning process uses C-SVC with RBF in libsvm. It performs parameter selection to determine the best parameters
* given the particular learning problem.
* 
* The features must be pre-scaled.
* 
* The returned svm_model uses the optimal parameters and includes probability estimates.
* 
* \param    features            Features for the different classes -- each column is one set of features
* \param    labels              Labels for each of the features
* \pre  labels.n_rows == features.n_cols
* \return   The svm_model to use for classifying using libsvm.
*/
svm_model* learn_svm_classifier(const Matrix& features, const IntMatrix& labels);

/**
* features_to_svm_nodes converts a feature vector into corresponding svm_node values suitable for classification via
* an svm_model*.
* 
* The nodes are pushed back into the nodes vector. The vector isn't cleared.
* 
* \param    features            Scaled features
* \param    nodes               Node vector in which to save the features
*/
void features_to_svm_nodes(const Vector& features, SVMFeatures& nodes);

} // namespace utils
} // namespace hssh

#endif // UTILS_SVM_H

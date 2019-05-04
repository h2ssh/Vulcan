/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     softmax.h
* \author   
* 
* Functions for performing softmax regression and then performing classification using the learned model:
* 
*   - softmax_regression     : training a classifier for use with max_likelihood or classify using labeled data
*   - softmax_max_likelihood : find max likelihood label for a measurement
*   - softmax_classify       : classify features using a learned classifier -- gives full distribution
*/

#ifndef MATH_SOFTMAX_H
#define MATH_SOFTMAX_H

#include <core/matrix.h>
#include <core/vector.h>

namespace vulcan 
{
namespace math 
{
    
/**
* softmax_regression_options_t contains the options that control the behavior of the softmax regression algorithm:
* 
*   - learningRate  : learning rate to use in the gradient descent algorithm, theta_new = theta_old - lr*gradient
*   - l2Lambda       : constant to use for L2 regularization in the cost function (-lambda * weight) (0 = no regularization)
*   - weightDecay   : the cost function contains a regularizing term to help keep the final thetas low and make
*                     the cost function convex
*   - maxIterations : maximum number of iterations to run gradient descent
*   - convergenceThreshold : maximum amount of change between iterations for convergence
*   - verbose : flag indicating if verbose output should be made
*/
struct softmax_regression_options_t
{
    double learningRate  = 0.75;
    double l2Lambda      = 1.0;
    double weightDecay   = 1e-4;
    int    maxIterations = 1000;
    double convergenceThreshold = 1e-5;
    bool verbose = false;
};


/**
* softmax_regression performs softmax regression on a set of labeled training data. Softmax regression creates a
* 1-of-K linear classifier for features the belong to one of a discrete set of types. This implementation of softmax
* regression uses a simple gradient descent approach to find a solution.
* 
* The input features are stored in the features matrix. This matrix is an  n+1-by-m matrix. Each column should contain
* the n measured features plus an additional value 1 for the intercept term. There is one column for each of the m
* examples for training. The labels matrix is an m-by-1 matrix where each row contains the assigned label [0,K) for 
* each column in the features matrix.
*
* \param    features            Measured features for the examples
* \param    labels              Labels for each of the sets of features
* \param    numClasses          Number of differents classes the classifier will select amongst
* \param    options             Options controlling the behavior of the gradient descent algorithm (optional, defaults
*                               are specified in the softmax_regression_options_t definition.
* \return   A k-by-n+1 matrix that contains the weights for the features for each of the K classes in the data. This
*           matrix is for use with the softmax_max_likelihood or softmax_classify functions to determine the labels for
*           a new set of features.
*/
Matrix softmax_regression(const Matrix&    features, 
                          const IntMatrix& labels, 
                          int              numClasses, 
                          softmax_regression_options_t options = softmax_regression_options_t());

/**
* softmax_regression_with_search performs softmax regression as in softmax_regression. The search version performs
* a linear search of possible values for l2Lambda, thus maximizing the overall performance of the classifier.
*
* \param    features            Measured features for the examples
* \param    labels              Labels for each of the sets of features
* \param    numClasses          Number of differents classes the classifier will select amongst
* \param    options             Options controlling the behavior of the gradient descent algorithm (optional, defaults
*                               are specified in the softmax_regression_options_t definition.
* \param    lambdaMin           Minimum value of lambda to search (optional, default = .1)
* \param    lambdaMax           Maximum value of lambda to search (optional, default = 1)
* \param    lambdaStep          Step size for the lambda search (optional, default = .1)
* \return   A k-by-n+1 matrix that contains the weights for the features for each of the K classes in the data. This
*           matrix is for use with the softmax_max_likelihood or softmax_classify functions to determine the labels for
*           a new set of features.
*
*/
Matrix softmax_regression_with_search(const Matrix&    features,
                                      const IntMatrix& labels,
                                      int              numClasses,
                                      softmax_regression_options_t options = softmax_regression_options_t(),
                                      double lambdaMin = 0.1,
                                      double lambdaMax = 1,
                                      double lambdaStep = 0.1);

/**
* softmax_max_likelihood calculates the maximum likelihood label for a feature vector using the classification matrix
* estimated by softmax_regression.
* 
* The first input is a feature vector with size n+1. The order of the features must be the same as used with the
* regression. The second input is the classification matrix which is k-by-n+1.
* 
* \param    features            Measured features
* \param    classifier          Estimated classifier matrix 
* \return   An int-double pair with the max likelihood label and the value of the likelihood.
*/
std::pair<int, double> softmax_max_likelihood(const Vector& features, const Matrix& classifier);

/**
* softmax_classify calculates the probability for each class in the classifier. The returned vector is the probability
* distribution across all classes in the matrix.
* 
* \param    features            Measured features
* \param    classifier          Estimated classifier matrix
* \return   The probability of each class for the provided features. sum(retVal) == 1.0.
*/
Vector softmax_classify(const Vector& features, const Matrix& classifier);

}
}

#endif // MATH_SOFTMAX_H

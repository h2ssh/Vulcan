/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lev_mar_optimizer.h
* \author   Collin Johnson
*
* Declaration of LevMarOptimizer.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LEV_MAR_OPTIMIZER_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LEV_MAR_OPTIMIZER_H

#include <hssh/global_topological/mapping/map_optimizer.h>
#include <hssh/global_topological/params.h>
#include <hssh/utils/id.h>
#include <map>
#include <unordered_map>
#include <vector>

namespace vulcan
{
namespace hssh
{

class GlobalPlace;
class GlobalPathSegment;

const std::string LEV_MAR_OPTIMIZER_TYPE("lev-mar");

/**
* LevMarOptimizer is a MapOptimizer that uses the Levenberg-Marquardt algorithm to calculate the optimized
* layout for the places in a map. LevMarOptimizer uses the levmar library to run the actual optimization.
*
* The parameters for LevMarOptimizer are:
*
*   [LevMarOptimizerParameters]
*   max_iterations  = maximum number of iterations to run the optimization
*   initial_mu      = initial stepsize to use for the gradient descent portion
*   stop_threshold  = convergence threshold
*/
class LevMarOptimizer : public MapOptimizer
{
public:

    /**
    * Constructor for LevMarOptimizer.
    *
    * \param    params          Parameters for controlling optimizer performance
    */
    LevMarOptimizer(const lev_mar_optimizer_params_t& params);
    
    /////   MapOptimizer interface   /////
    Chi optimizeMap(const TopologicalMap& map);

private:

    // When calculating the residuals, need to know which values in the p-vector correspond to the measurements in the x-vector
    // When creating p-vector, store which placeId corresponds to which p-index. Then when building the x-vector, the path segments
    // give the placeIds involved and idToPIndex can be used to determine the correct p-index to use for calculating the new lambdas
    // when calculateLambdas is called by levmar_func
    std::unordered_map<Id, int> idToPIndex;
    std::vector<std::pair<int, int>> xLambdaIndices;

    // Fields for interacting with the levmar library
    std::size_t numPlaces = 0;   // number of places in current optimization run  (= m/3 in levmar)
    std::size_t numEdges = 0;    // number of edges between places in current run (= n/3 in levmar)
    Id initialId_ = kInvalidId;   // index of the fixed area
    double finalError = 0.0;     // Final error after the optimization converges

    std::vector<double> p;              ///< Parameter estimates -- input = initial guess, output = final optimization -- places
    std::vector<double> x;              ///< Measurements -- converted lambda values --- edges
    std::vector<double> variance;       ///< Variance of the measurements -- for scaling the errors
    std::vector<double> xError;         ///< Current error of the measurements -- for use in jacobian calculation
    std::vector<double> covariance;     ///< Covariance of the final parameter estimates -- numPlaces x numPlaces
    std::vector<double> work;           ///< Buffer to provide to levmar library
    std::vector<double> jacobian;       ///< Analytic Jacobian -- currently unused

    lev_mar_optimizer_params_t params_;
    
    
    void setupPVector  (const TopologicalMap& map);
    void setupXVector  (const TopologicalMap& map);
    void countEdges    (const TopologicalMap& map);
    void runLevmar     (void);
    Chi  createChiFromP(void);
    
    // The function to be called by the levmar_func callback
    void calculateLambdas (double* newP, double* hx, int m, int n);
    void calculateJacobian(double* j, int m, int n);
    
    // Create friend functions to issue as the callbacks used by levmar. The adata will point to this, which will then
    // be used to call back into the instance of LevMarOptimizer.
    friend void levmar_func    (double* p, double* hx, int m, int n, void* adata);
    friend void levmar_jacobian(double* p, double* j, int m, int n, void* adata);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LEV_MAR_OPTIMIZER_H

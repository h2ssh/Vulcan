/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     algebraic_circle_fitting.cpp
* \author   Collin Johnson
*
* Definition of circle_fit_by_taubin.
*
* Adapted the following files available at: http://people.cas.uab.edu/~mosya/cl/CPPcircle.html
*
*   - CircleFitByTaubin.cpp
*/

#include <math/geometry/algebraic_circle_fitting.h>
#include <numeric>
#include <cmath>

namespace vulcan
{
namespace math
{
namespace detail
{

Circle<float> circle_fit_by_taubin(DataIterator begin, DataIterator end)
/*
*      Circle fit to a given set of data points (in 2D)
*
*      This is an algebraic fit, due to Taubin, based on the journal article
*
*      G. Taubin, "Estimation Of Planar Curves, Surfaces And Nonplanar
*                  Space Curves Defined By Implicit Equations, With
*                  Applications To Edge And Range Image Segmentation",
*                  IEEE Trans. PAMI, Vol. 13, pages 1115-1138, (1991)
*
*      Input:  data     - the class of data (contains the given points):
*
*          data.n   - the number of data points
*          data.X[] - the array of X-coordinates
*          data.Y[] - the array of Y-coordinates
*
*     Output:
*               circle - parameters of the fitting circle:
*
*           circle.a - the X-coordinate of the center of the fitting circle
*           circle.b - the Y-coordinate of the center of the fitting circle
*           circle.r - the radius of the fitting circle
*           circle.s - the root mean square error (the estimate of sigma)
*           circle.j - the total number of iterations
*
*     The method is based on the minimization of the function
*
*                  sum [(x-a)^2 + (y-b)^2 - R^2]^2
*              F = -------------------------------
*                      sum [(x-a)^2 + (y-b)^2]
*
*     This method is more balanced than the simple Kasa fit.
*
*     It works well whether data points are sampled along an entire circle or
*     along a small arc.
*
*     It still has a small bias and its statistical accuracy is slightly
*     lower than that of the geometric fit (minimizing geometric distances),
*     but slightly higher than that of the very similar Pratt fit.
*     Besides, the Taubin fit is slightly simpler than the Pratt fit
*
*     It provides a very good initial guess for a subsequent geometric fit.
*
*       Nikolai Chernov  (September 2012)
*
*/
{
    int iter,IterMAX=99;

    double Xi,Yi,Zi;
    double Mz,Mxy,Mxx,Myy,Mxz,Myz,Mzz,Cov_xy,Var_z;
    double A0,A1,A2,A22,A3,A33;
    double Dy,xnew,x,ynew,y;
    double DET,Xcenter,Ycenter;

    auto dataMean = meanPoint(begin, end);
    //     computing moments

    Mxx=Myy=Mxy=Mxz=Myz=Mzz=0.;

    for(auto pIt = begin; pIt != end; ++pIt)
    {
        Xi = pIt->x - dataMean.x;   //  centered x-coordinates
        Yi = pIt->y - dataMean.y;   //  centered y-coordinates
        Zi = Xi*Xi + Yi*Yi;

        Mxy += Xi*Yi;
        Mxx += Xi*Xi;
        Myy += Yi*Yi;
        Mxz += Xi*Zi;
        Myz += Yi*Zi;
        Mzz += Zi*Zi;
    }

    auto size = std::distance(begin, end);
    Mxx /= size;
    Myy /= size;
    Mxy /= size;
    Mxz /= size;
    Myz /= size;
    Mzz /= size;

    //      computing coefficients of the characteristic polynomial

    Mz = Mxx + Myy;
    Cov_xy = Mxx*Myy - Mxy*Mxy;
    Var_z = Mzz - Mz*Mz;
    A3 = Four*Mz;
    A2 = -Three*Mz*Mz - Mzz;
    A1 = Var_z*Mz + Four*Cov_xy*Mz - Mxz*Mxz - Myz*Myz;
    A0 = Mxz*(Mxz*Myy - Myz*Mxy) + Myz*(Myz*Mxx - Mxz*Mxy) - Var_z*Cov_xy;
    A22 = A2 + A2;
    A33 = A3 + A3 + A3;

    //    finding the root of the characteristic polynomial
    //    using Newton's method starting at x=0
    //     (it is guaranteed to converge to the right root)

    for (x=0.,y=A0,iter=0; iter<IterMAX; iter++)  // usually, 4-6 iterations are enough
    {
        Dy = A1 + x*(A22 + A33*x);
        xnew = x - y/Dy;
        if ((xnew == x)||(!std::isfinite(xnew))) break;
        ynew = A0 + xnew*(A1 + xnew*(A2 + xnew*A3));
        if (std::abs(ynew)>=std::abs(y))  break;
        x = xnew;  y = ynew;
    }

    //       computing paramters of the fitting circle

    DET = x*x - x*Mz + Cov_xy;
    Xcenter = (Mxz*(Myy - x) - Myz*Mxy)/DET/Two;
    Ycenter = (Myz*(Mxx - x) - Mxz*Mxy)/DET/Two;

    //       assembling the output
    Point<float> center(Xcenter + dataMean.x, Ycenter + dataMean.y);
    double radius = sqrt(Xcenter*Xcenter + Ycenter*Ycenter + Mz);

    if(radius <= 0.0)
    {
        double totalDistance = std::accumulate(begin, end, 0.0,
                                               [&center](double total, const Point<float>& p)
                                               {
                                                   return total + distance_between_points(center, p);
                                               });
        radius = totalDistance / size;

        std::cerr << "ERROR: circle_fit_by_taubin: Estimated radius was 0. Using mean distance from center instead."
                  << " Mean:" << radius << '\n';

        if(radius == 0.0)
        {
            std::cerr << "ERROR: circle_fit_by_taubin: Mean radius is 0! Bad input!";

            radius = std::numeric_limits<float>::max();
            center = dataMean;
        }
    }
    else if((Mxx == 0.0) || (Myy == 0.0))
    {
        radius = std::numeric_limits<float>::max();
        center = dataMean;

        std::cerr << "ERROR: circle_fit_by_taubin: Trying to fit circle to a straight line."
                  << " Setting radius to maximum float and center to mean.\n";
    }

    return Circle<float>(radius, center);
}

} // namespace detail
} // namespace math
} // namespace vulcan

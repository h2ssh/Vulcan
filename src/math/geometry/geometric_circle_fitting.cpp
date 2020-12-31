/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     geometric_circle_fitting.cpp
* \author   Collin Johnson
* 
* Definition of circle_fit_by_lm_full and circle_fit_by_lma.
* 
* Adapted the following files available at: http://people.cas.uab.edu/~mosya/cl/CPPcircle.html
* 
*   - CircleFitByLevenbergMarquardtFull.cpp
*   - CircleFitByChernovLesort.cpp
*/

#include "math/geometry/geometric_circle_fitting.h"

namespace vulcan
{
namespace math
{
namespace detail
{

struct circle_fit_data_t
{
    double a, b, r, s, g, Gx, Gy;
    int i, j;
    
    explicit circle_fit_data_t(Point<double> center = Point<double>(0.0, 0.0), double r = 0.0)
    : a(center.x)
    , b(center.y)
    , r(r)
    , s(0.0)
    , i(0)
    , j(0)
    {
    }
};


int circle_fit_by_lm_full(DataIterator   begin, 
                          DataIterator   end, 
                          Circle<float>& circleIni, 
                          double         LambdaIni, 
                          Circle<float>& fitCircle)
/*                                     <------------------ Input ------------------->  <-- Output -->
* 
*       Geometric circle fit to a given set of data points (in 2D)
*        
*       Input:  data     - the class of data (contains the given points):
*        
*           size   - the number of data points
*           data.X[] - the array of X-coordinates
*           data.Y[] - the array of Y-coordinates
*                  
*               circleIni - parameters of the initial circle ("initial guess")
*                
*           circleIni.a - the X-coordinate of the center of the initial circle
*           circleIni.b - the Y-coordinate of the center of the initial circle
*           circleIni.r - the radius of the initial circle
*                
*           LambdaIni - the initial value of the control parameter "lambda"
*                       for the Levenberg-Marquardt procedure
*                       (common choice is a small positive number, e.g. 0.001)
*                
*       Output:
*           integer function value is a code:
*                      0:  normal termination, the best fitting circle is 
*                          successfully found
*                      1:  the number of outer iterations exceeds the limit (99)
*                          (indicator of a possible divergence)
*                      2:  the number of inner iterations exceeds the limit (99)
*                          (another indicator of a possible divergence)
*                      3:  the coordinates of the center are too large
*                          (a strong indicator of divergence)
*                           
*           circle - parameters of the fitting circle ("best fit")
*                
*           circle.a - the X-coordinate of the center of the fitting circle
*           circle.b - the Y-coordinate of the center of the fitting circle
*           circle.r - the radius of the fitting circle
*           circle.s - the root mean square error (the estimate of sigma)
*           circle.i - the total number of outer iterations (updating the parameters)
*           circle.j - the total number of inner iterations (adjusting lambda)
*                
*       Algorithm:  Levenberg-Marquardt running over the full parameter space (a,b,r)
*                         
*       See a detailed description in Section 4.5 of the book by Nikolai Chernov:
*       "Circular and linear regression: Fitting circles and lines by least squares"
*       Chapman & Hall/CRC, Monographs on Statistics and Applied Probability, volume 117, 2010.
*         
*        Nikolai Chernov,  February 2014
*/
{
    int code,iter,inner,IterMAX=99;
    
    double factorUp=10.,factorDown=0.04,lambda,ParLimit=1.e+6;
    double dx,dy,ri,u,v;
    double Mu,Mv,Muu,Mvv,Muv,Mr,UUl,VVl,Nl,F1,F2,F3,dX,dY,dR;
    double epsilon=3.e-8;
    double G11,G22,G33,G12,G13,G23,D1,D2,D3;
    
    circle_fit_data_t New(circleIni.center(), circleIni.radius());
    circle_fit_data_t Old;
    
    auto size     = std::distance(begin, end);
    auto dataMean = meanPoint(begin, end);
    
    //       starting with the given initial circle (initial guess)
    //       compute the root-mean-square error via function Sigma; see Utilities.cpp
    New.s = Sigma(begin, end, circleIni);
    
    //       initializing lambda, iteration counters, and the exit code
    
    lambda = LambdaIni;
    iter = inner = code = 0;
    
NextIteration:
    
    Old = New;
    if (++iter > IterMAX) {code = 1;  goto enough;}
    
    //       computing moments
    
    Mu=Mv=Muu=Mvv=Muv=Mr=0.;
    
    for(auto pIt = begin; pIt != end; ++pIt)
    {
        dx = pIt->x - Old.a;
        dy = pIt->y - Old.b;
        ri = sqrt(dx*dx + dy*dy);
        u = dx/ri;
        v = dy/ri;
        Mu += u;
        Mv += v;
        Muu += u*u;
        Mvv += v*v;
        Muv += u*v;
        Mr += ri;
    }
    
    Mu /= size;
    Mv /= size;
    Muu /= size;
    Mvv /= size;
    Muv /= size;
    Mr /= size;
    
    //       computing matrices
    
    F1 = Old.a + Old.r*Mu - dataMean.x;
    F2 = Old.b + Old.r*Mv - dataMean.y;
    F3 = Old.r - Mr;
    
    Old.g = New.g = sqrt(F1*F1 + F2*F2 + F3*F3);
    
try_again:
    
    UUl = Muu + lambda;
    VVl = Mvv + lambda;
    Nl = One + lambda;
    
    //         Cholesly decomposition
    
    G11 = sqrt(UUl);
    G12 = Muv/G11;
    G13 = Mu/G11;
    G22 = sqrt(VVl - G12*G12);
    G23 = (Mv - G12*G13)/G22;
    G33 = sqrt(Nl - G13*G13 - G23*G23);
    
    D1 = F1/G11;
    D2 = (F2 - G12*D1)/G22;
    D3 = (F3 - G13*D1 - G23*D2)/G33;
    
    dR = D3/G33;
    dY = (D2 - G23*dR)/G22;
    dX = (D1 - G12*dY - G13*dR)/G11;
    
    if ((std::abs(dR)+std::abs(dX)+std::abs(dY))/(One+Old.r) < epsilon) goto enough;
    
    //       updating the parameters
    
    New.a = Old.a - dX;
    New.b = Old.b - dY;
    
    if (std::abs(New.a)>ParLimit || std::abs(New.b)>ParLimit) {code = 3; goto enough;}
    
    New.r = Old.r - dR;
    
    if (New.r <= 0.)
    {
        lambda *= factorUp;
        if (++inner > IterMAX) {code = 2;  goto enough;}
        goto try_again;
    }
    
    //       compute the root-mean-square error via function Sigma; see Utilities.cpp
    
    // New.r can never be 0.0 because Anew isn't infinite, so the division always yields a number > 1
    assert(New.r > 0.0);
    New.s = Sigma(begin, end, Circle<float>(New.r, Point<float>(New.a, New.b)));
    
    //       check if improvement is gained
    
    if (New.s < Old.s)    //   yes, improvement
    {
        lambda *= factorDown;
        goto NextIteration;
    }
    else                       //   no improvement
    {
        if (++inner > IterMAX) {code = 2;  goto enough;}
        lambda *= factorUp;
        goto try_again;
    }
    
    //       exit
    
enough:
    
    Old.i = iter;    // total number of outer iterations (updating the parameters)
    Old.j = inner;   // total number of inner iterations (adjusting lambda)
    
    if(Old.r > 0.0)
    {
        fitCircle = Circle<float>(Old.r, Point<float>(Old.a, Old.b));
    }
    else
    {
        code = 4;
    }

    return code;
}


int circle_fit_by_lma(DataIterator   begin, 
                      DataIterator   end, 
                      Circle<float>& circleIni, 
                      double         LambdaIni, 
                      Circle<float>& fitCircle)
/*                            <------------------ Input ------------------->  <-- Output -->
* 
*       Geometric circle fit to a given set of data points (in 2D)
*        
*       Input:  data     - the class of data (contains the given points):
*        
*           size   - the number of data points
*           data.X[] - the array of X-coordinates
*           data.Y[] - the array of Y-coordinates
*                  
*               circleIni - parameters of the initial circle ("initial guess")
*                
*           circleIni.a - the X-coordinate of the center of the initial circle
*           circleIni.b - the Y-coordinate of the center of the initial circle
*           circleIni.r - the radius of the initial circle
*                
*           LambdaIni - the initial value of the control parameter "lambda"
*                       for the Levenberg-Marquardt procedure
*                       (common choice is a small positive number, e.g. 0.001)
*                
*       Output:
*           integer function value is a code:
*                      0:  normal termination, the best fitting circle is 
*                          successfully found
*                      1:  the number of outer iterations exceeds the limit (99)
*                          (indicator of a possible divergence)
*                      2:  the number of inner iterations exceeds the limit (99)
*                          (another indicator of a possible divergence)
*                      3:  the coordinates of the center are too large
*                          (a strong indicator of divergence)
*                           
*           circle - parameters of the fitting circle ("best fit")
*                
*           circle.a - the X-coordinate of the center of the fitting circle
*           circle.b - the Y-coordinate of the center of the fitting circle
*           circle.r - the radius of the fitting circle
*           circle.s - the root mean square error (the estimate of sigma)
*           circle.i - the total number of outer iterations (updating the parameters)
*           circle.j - the total number of inner iterations (adjusting lambda)
*                
*       Algorithm by Nikolai Chernov and Claire Lesort
*                         
*       See a detailed description in the journal paper:
*       
*       N. Chernov and C. Lesort, "Least squares fitting of circles"
*          in J. Math. Imag. Vision, volume 23, (2005), pages 239-251.
*          
*       the algorithm is designed to converge from any initial guess,
*       but it is complicated and generally very slow
*         
*        Nikolai Chernov,  February 2014
*/
{
    int code,iter,inner,IterMAX=99;
    
    double factorUp=10.,factorDown=0.04,lambda;
    double Aold,Fold,Told,Anew,Fnew,Tnew,DD,H,aabb;
    double Xi,Yi,Zi,Ui,Vi,Gi,CT,ST,D,ADF,SQ,DEN,FACT,DGDAi,DGDFi,DGDTi;
    double H11,H12,H13,H22,H23,H33,F1,F2,F3,dA,dF,dT;
    double epsilon=3.e-8;  
    double G11,G22,G33,G12,G13,G23,D1,D2,D3;
    double Xshift=0.,Yshift=0.,dX=One,dY=0.,aTemp,bTemp,rTemp;
    double deltaFit = 0.0;
    
    circle_fit_data_t New(circleIni.center(), circleIni.radius());
    circle_fit_data_t Old;
    
    //       starting with the given initial circle (initial guess)
    //       compute the root-mean-square error via function Sigma; see Utilities.cpp
    New.s = Sigma(begin, end, circleIni);
    
    Anew = One/Two/New.r;
    aabb = New.a*New.a + New.b*New.b;
    Fnew = (aabb - New.r*New.r)*Anew;
    Tnew = acos(-New.a/sqrt(aabb));
    if (New.b > 0.) Tnew = Two*Pi - Tnew;
    
    if (One+Four*Anew*Fnew < epsilon) 
    {
        Xshift += dX;
        Yshift += dY;
        
        New.a += dX;
        New.b += dY;
        aabb = New.a*New.a + New.b*New.b;
        Fnew = (aabb - New.r*New.r)*Anew;
        Tnew = acos(-New.a/sqrt(aabb));
        if (New.b > 0.) Tnew = Two*Pi - Tnew;
    }
    
    //       initializing lambda, iteration counters, and the exit code
    
    lambda = LambdaIni;
    iter = inner = code = 0;
    
NextIteration:
    
    Aold = Anew;
    Fold = Fnew;
    Told = Tnew;
    Old = New;
    
    if (++iter > IterMAX) {code = 1;  goto enough;}
    
    //       computing moments
    
shiftXY:
    
    DD = One + Four*Aold*Fold;
    D = sqrt(DD);
    CT = cos(Told);
    ST = sin(Told);
    
    H11=H12=H13=H22=H23=H33=F1=F2=F3=0.;
    
    for(auto pIt = begin; pIt != end; ++pIt)
    {
        Xi = pIt->x + Xshift;
        Yi = pIt->y + Yshift;
        Zi = Xi*Xi + Yi*Yi;
        Ui = Xi*CT + Yi*ST;
        Vi =-Xi*ST + Yi*CT;
        
        ADF = Aold*Zi + D*Ui + Fold;
        SQ = sqrt(Four*Aold*ADF + One);
        DEN = SQ + One;
        Gi = Two*ADF/DEN;
        FACT = Two/DEN*(One - Aold*Gi/SQ);
        DGDAi = FACT*(Zi + Two*Fold*Ui/D) - Gi*Gi/SQ;
        DGDFi = FACT*(Two*Aold*Ui/D + One);
        DGDTi = FACT*D*Vi;
        
        H11 += DGDAi*DGDAi;
        H12 += DGDAi*DGDFi;
        H13 += DGDAi*DGDTi;
        H22 += DGDFi*DGDFi;
        H23 += DGDFi*DGDTi;
        H33 += DGDTi*DGDTi;
        
        F1 += Gi*DGDAi;
        F2 += Gi*DGDFi;
        F3 += Gi*DGDTi;
    }
    Old.g = New.g = sqrt(F1*F1 + F2*F2 + F3*F3);
    
try_again:
    
    //        Cholesky decomposition
    
    G11 = sqrt(H11 + lambda);
    G12 = H12/G11;
    G13 = H13/G11;
    G22 = sqrt(H22 + lambda - G12*G12);
    G23 = (H23 - G12*G13)/G22;
    G33 = sqrt(H33 + lambda - G13*G13 - G23*G23);
    
    D1 = F1/G11;
    D2 = (F2 - G12*D1)/G22;
    D3 = (F3 - G13*D1 - G23*D2)/G33;
    
    dT = D3/G33;
    dF = (D2 - G23*dT)/G22;
    dA = (D1 - G12*dF - G13*dT)/G11;
    
    //       updating the parameters
    
    Anew = Aold - dA;
    Fnew = Fold - dF;
    Tnew = Told - dT;
    
    if (One+Four*Anew*Fnew < epsilon) 
    {
        Xshift += dX;
        Yshift += dY;
        
        H = sqrt(One+Four*Aold*Fold);
        aTemp = -H*cos(Told)/(Aold+Aold) + dX;
        bTemp = -H*sin(Told)/(Aold+Aold) + dY;
        rTemp = One/std::abs(Aold+Aold);
        
        Aold = One/(rTemp + rTemp);
        aabb = aTemp*aTemp + bTemp*bTemp;
        Fold = (aabb - rTemp*rTemp)*Aold;
        Told = acos(-aTemp/sqrt(aabb));
        if (bTemp > 0.) Told = Two*Pi - Told;
        
        lambda *= factorUp;
        inner++;
        goto shiftXY;
    }
    
    H = sqrt(One+Four*Anew*Fnew);
    New.a = -H*cos(Tnew)/(Anew+Anew) - Xshift;
    New.b = -H*sin(Tnew)/(Anew+Anew) - Yshift;
    New.r = One/std::abs(Anew+Anew);
    
    // New.r can never be 0.0 because Anew isn't infinite, so the division always yields a number > 1
    assert(New.r > 0.0);
    New.s = Sigma(begin, end, Circle<float>(New.r, Point<float>(New.a, New.b)));
    
    deltaFit = (std::abs(New.a-Old.a) + std::abs(New.b-Old.b) + std::abs(New.r-Old.r))/(One + Old.r);
    if (deltaFit < epsilon)
    {
        goto enough;
    }
    
    //       check if improvement is gained
    
    if (New.s < Old.s)    //   yes, improvement
    {
        lambda *= factorDown;
        goto NextIteration;
    }
    else                       //   no improvement
    {
        if (++inner > IterMAX) {code = 2;  goto enough;}
        lambda *= factorUp;
        goto try_again;
    }
    
    //       exit
    
enough:
    
    Old.i = iter;
    Old.j = inner;
    
    if(Old.r > 0.0)
    {
        fitCircle = Circle<float>(Old.r, Point<float>(Old.a, Old.b));
    }
    else
    {
        code = 4;
    }
    
    return code;
}
    
} // namespace detail
} // namespace math
} // namespace vulcan

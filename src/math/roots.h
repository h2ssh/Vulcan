/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     roots.h
* \author   Collin Johnson
* 
* Definition of a generic root-finding algorithms:
* 
*   - find_single_root
* 
* and associated implementations of the concepts:
* 
*   - NewtonRaphsonErrorFunc
*/

#ifndef MATH_ROOTS_H
#define MATH_ROOTS_H

#include <iostream>
#include <functional>
#include <type_traits>

namespace vulcan
{
namespace math
{
    
/**
* find_single_root finds the root of an equation containing a single variable, f(x). To find the root, an
* ErrorFunc is provided to find_single_root. This function accepts a value of x and returns some deltaX
* value.
* 
* The root-finding equation is simply:  x' = x - ErrorFunc(x). It runs until the value x' - x < precision.
* 
* single_root is templated on ErrorFunc, which is a class that supports the Callable concept with a single
* argument and returns a value of the same type.
* 
* The initial value of x and the precision are passed in because they aren't general to all roots.
* 
* NOTE: This function does not currently do anything fancy. As a result, it is susceptible to all the problems
*       of robustness that simple root-finders are prone to -- minima, divide by 0, etc.
* 
* NOTE: Only floating point values are currently supported.
* 
* \param    func            ErrorFunc class
* \param    initial         Initial value for the root finder
* \param    precision       How close to 0 before stopping
* \return   The value of x when |f(x)| < precision.
*/
template <typename T, class ErrorFunc>
T find_single_root(ErrorFunc func, T initial, T precision)
{
    if(std::is_floating_point<T>::value)
    {
        T root = initial;
        T delta;
        
        do
        {
            delta = func(root);
            root -= delta;
        } while(std::abs(delta) > precision);
        
        return root;
    }
    else
    {
        std::cout<<"INFO::find_single_root: Unable to find roots with non-floating point values. Returning initial.\n";
        return initial;
    }
}

/**
* NewtonRaphsonErrorFunc implements the ErrorFunc concept. The Newton-Raphson error function is:
* 
*       f(x) / f'(x)
*/
template <typename T = double>
class NewtonRaphsonErrorFunc
{
public:
    
    using Func = std::function<T(T)>;
    
    /**
    * Constructor for NewtonRaphsonErrorFunc.
    * 
    * \param    func            f(x)
    * \param    derivative      f'(x)
    */
    NewtonRaphsonErrorFunc(Func func, Func derivative)
    : func_(func)
    , derivative_(derivative)
    {
    }
    
    // Callable concept
    T operator()(T value)
    {
        std::cout<<"Newton: x:"<<value<<" f(x):"<<func_(value)<<" f'(x):"<<derivative_(value)<<" err:"<<(func_(value) / derivative_(value))<<'\n';
        
        return func_(value) / derivative_(value);
    }
    
private:
    
    Func func_;
    Func derivative_;
};
    
}
}

#endif // MATH_ROOTS_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef MATH_VECTOR_H
#define MATH_VECTOR_H

#include <armadillo>
#include <cereal/cereal.hpp>

namespace vulcan
{

using Vector = arma::Col<double>;
using IntVector = arma::Col<int>;

}   // namespace vulcan

namespace cereal
{

template <class Archive, typename T>
void save(Archive& ar, const arma::Col<T>& c)
{
    uint64_t num = c.n_elem;
    ar(num, binary_data(const_cast<T*>(c.mem), num * sizeof(T)));
}

template <class Archive, typename T>
void load(Archive& ar, arma::Col<T>& c)
{
    uint64_t num;
    T* memory;

    ar(num);
    memory = new T[num];
    ar(binary_data(memory, num * sizeof(T)));

    c = arma::Col<T>(memory, num);

    delete[] memory;
}

}   // namespace cereal

#endif   // MATH_VECTOR_H

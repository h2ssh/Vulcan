/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef MATH_MATRIX_H
#define MATH_MATRIX_H

#include <armadillo>
#include <cassert>
#include <cereal/cereal.hpp>

namespace vulcan
{

using Matrix = arma::Mat<double>;
using IntMatrix = arma::Mat<int>;

}   // namespace vulcan

namespace cereal
{

template <class Archive, typename T>
void save(Archive& ar, const arma::Mat<T>& m)
{
    uint64_t numRows = m.n_rows;
    uint64_t numCols = m.n_cols;
    ar(numRows, numCols, binary_data(const_cast<T*>(m.mem), numRows * numCols * sizeof(T)));
    assert(numRows * numCols == m.n_elem);
}

template <class Archive, typename T>
void load(Archive& ar, arma::Mat<T>& m)
{
    uint64_t rows, cols;
    T* memory;

    ar(rows, cols);

    memory = new T[rows * cols];
    ar(binary_data(memory, rows * cols * sizeof(T)));

    m = arma::Mat<T>(memory, rows, cols);

    delete[] memory;
}

}   // namespace cereal

#endif   // MATH_MATRIX_H

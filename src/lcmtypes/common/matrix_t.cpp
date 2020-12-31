/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "lcmtypes/common/matrix_t.h"
#include <cassert>


void vulcan::lcm::allocate_lcm_matrix(vulcan_lcm_matrix_t& matrix, int numRows, int numColumns)
{
    if (numRows && numColumns) {
        matrix.num_rows = numRows;
        matrix.num_columns = numColumns;

        matrix.matrix = new double*[numRows];

        while (--numRows >= 0) {
            matrix.matrix[numRows] = new double[numColumns];
        }
    } else {
        matrix.num_rows = 0;
        matrix.num_columns = 0;
        matrix.matrix = 0;
    }
}


void vulcan::lcm::free_lcm_matrix(vulcan_lcm_matrix_t& matrix)
{
    if (matrix.matrix) {
        for (int i = matrix.num_rows; --i >= 0;) {
            delete[] matrix.matrix[i];
        }

        delete[] matrix.matrix;
    }
}


void vulcan::lcm::vulcan_matrix_to_lcm_matrix(const Matrix& vulcanMatrix, vulcan_lcm_matrix_t& lcmMatrix)
{
    for (int i = vulcanMatrix.n_rows; --i >= 0;) {
        for (int j = vulcanMatrix.n_cols; --j >= 0;) {
            lcmMatrix.matrix[i][j] = vulcanMatrix(i, j);
        }
    }
}


void vulcan::lcm::lcm_matrix_to_vulcan_matrix(const vulcan_lcm_matrix_t& lcmMatrix, Matrix& vulcanMatrix)
{
    for (int i = vulcanMatrix.n_rows; --i >= 0;) {
        for (int j = vulcanMatrix.n_cols; --j >= 0;) {
            vulcanMatrix(i, j) = lcmMatrix.matrix[i][j];
        }
    }
}

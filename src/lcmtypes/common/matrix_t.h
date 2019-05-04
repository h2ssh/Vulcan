/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LCMTYPES_MATRIX_T_H
#define LCMTYPES_MATRIX_T_H

#include <lcmtypes/vulcan_lcm_matrix_t.h>
#include <core/matrix.h>

namespace vulcan
{
namespace lcm
{

void allocate_lcm_matrix(vulcan_lcm_matrix_t& matrix, int numRows, int numColumns);
void free_lcm_matrix(vulcan_lcm_matrix_t& matrix);

void vulcan_matrix_to_lcm_matrix(const Matrix& vulcanMatrix, vulcan_lcm_matrix_t& lcmMatrix);
void lcm_matrix_to_vulcan_matrix(const vulcan_lcm_matrix_t& lcmMatrix, Matrix& vulcanMatrix);

}
}

#endif // LCMTYPES_MATRIX_T_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <utils/cell_grid.h>

namespace vulcan
{
namespace utils
{

template class CellGrid<uint8_t>;
template class CellGrid<int16_t>;
template class CellGrid<uint16_t>;
    
}
}

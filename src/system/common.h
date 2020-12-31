/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <array>
#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <cereal/access.hpp>
#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/stream.hpp>

#include "system/message_traits.h"

#include <boost/optional.hpp>

#include <armadillo>
//#include "lcmtypes/lcm_types.h"



// #include "core/point.h"
// #include "core/line.h"

// #include "math/geometry/rectangle.h"
// #include "math/geometry/polygon.h"

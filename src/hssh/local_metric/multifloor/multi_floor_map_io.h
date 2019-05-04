/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     multi_floor_map_io.h
* \author   Collin Johnson
* 
* Declaration of MultiFloorMapIO.
*/

#ifndef HSSH_UTILS_MULTI_FLOOR_MAP_IO_H
#define HSSH_UTILS_MULTI_FLOOR_MAP_IO_H

#include <string>

namespace vulcan
{
namespace hssh
{
    
class MultiFloorMap;

/**
* MultiFloorMapIO is used to save/load a MultiFloorMap to/from the hard drive.
* 
* The file format is:
* 
*   <map_info>
*       <id> id of the map </id>
*       <current_floor> id or -1 if map wasn't initialized </current_floor>
*       <current_elevator> id or -1 if not on elevator </current_elevator>
*   </map_info>
* 
*   <floor>
*       <id> id </id>
*       <elevators> id1 id2 id3 ...</elevators>
*       <lpm> lpm filename </lpm>
*   </floor>
* 
*   <elevator>
*       <id> id </id>
*       <current_floor> floor id </current_floor>
*       <floors> id1 id2 ...</floors>
*       <transition> start end height </transition>
*               can be many transitions
*       <boundary> floor bottom_left top_right </boundary>
*               one boundary per floor
*   </elevator>
*/
class MultiFloorMapIO
{
public:
    
    /**
    * save saves the map to a file using the above file format.
    * 
    * \param    map         Map to be saved
    * \param    filename    Name of the file in which to save the map
    * \return   True if the map was saved successfully. False if the file couldn't be opened or some other I/O error occurred.
    */
    static bool save(const MultiFloorMap& map, const std::string& filename);
    
    /**
    * load loads the multi-floor map from a file.
    * 
    * \param    filename    Name of the file with the map
    * \return   The map loaded from the file.
    */
    static MultiFloorMap load(const std::string& filename);
};

}
}

#endif // HSSH_UTILS_MULTI_FLOOR_MAP_IO_H

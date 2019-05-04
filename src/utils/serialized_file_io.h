/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     serialized_file_io.h
* \author   Collin Johnson
* 
* Definition of utilities for reading and writing files that consist of a Serializable types:
* 
*   - save_serializable_to_file
*   - load_serializable_from_file
* 
* 
*/

#ifndef UTILS_SERIALIZED_FILE_IO_H
#define UTILS_SERIALIZED_FILE_IO_H

#include <cereal/types/tuple.hpp>
#include <cereal/archives/binary.hpp>
#include <fstream>

namespace vulcan
{
namespace utils
{

/**
* save_serializable_to_file saves a serializable type to a file.
*/
template <class T>
bool save_serializable_to_file(const std::string& filename, const T& value)
{
    std::ofstream out(filename, std::ios_base::binary);

    if(!out.is_open())
    {
        std::cerr << "ERROR:save_to_serializable_to_file: Failed to open file: " << filename << '\n';
        return false;
    }

    try
    {
        cereal::BinaryOutputArchive ar(out);
        ar(value);
    }
    catch(cereal::Exception& e)
    {
        std::cerr << "ERROR:save_to_serializable_to_file: Failed to serialize elements: " << e.what() << '\n';
        return false;
    }

    return true;
}


/**
* save_serializable_tuple_to_file saves a tuple of serializable types to file. This is useful for creating multi-type
* file formats.
*/
template <typename... Ts>
bool save_serializable_tuple_to_file(const std::string& filename, const std::tuple<Ts...>& elements)
{
    std::ofstream out(filename, std::ios_base::binary);
    
    if(!out.is_open())
    {
        std::cerr << "ERROR:save_to_serializable_to_file: Failed to open file: " << filename << '\n';
        return false;
    }
    
    try
    {
        cereal::BinaryOutputArchive ar(out);
        ar(elements);
    }
    catch(cereal::Exception& e)
    {
        std::cerr << "ERROR:save_to_serializable_to_file: Failed to serialize elements: " << e.what() << '\n';
        return false;
    }
    
    return true;
}


/**
* load_serializable_from_file loads a saved serialized type from a file.
*/
template <class T>
bool load_serializable_from_file(const std::string& filename, T& value)
{
    std::ifstream in(filename, std::ios_base::binary);

    if(!in.is_open())
    {
        std::cerr << "ERROR:load_serializable_from_file: Failed to open file: " << filename << '\n';
        return false;
    }

    try
    {
        cereal::BinaryInputArchive ar(in);
        ar(value);
    }
    catch(cereal::Exception& e)
    {
        std::cerr << "ERROR:load_serializable_from_file: Failed to serialize elements: " << e.what() << '\n';
        return false;
    }

    return true;
}


/**
* load_serializable_tuple_from_file loads a saved tuple from a file.
*/
template <typename... Ts>
bool load_serializable_tuple_from_file(const std::string& filename, std::tuple<Ts...>& elements)
{
    std::ifstream in(filename, std::ios_base::binary);

    if(!in.is_open())
    {
        std::cerr << "ERROR:load_serializable_from_file: Failed to open file: " << filename << '\n';
        return false;
    }

    try
    {
        cereal::BinaryInputArchive ar(in);
        ar(elements);
    }
    catch(cereal::Exception& e)
    {
        std::cerr << "ERROR:load_serializable_from_file: Failed to serialize elements: " << e.what() << '\n';
        return false;
    }

    return true;
}

} // namespace utils
} // namespace vulcan

#endif // UTILS_SERIALIZED_FILE_IO_H

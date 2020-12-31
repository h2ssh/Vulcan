/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     byte_conversion.cpp
 * \author   Collin Johnson
 *
 * Definition of various conversion functions for bytes to/from larger data types.
 */

#include "utils/byte_conversion.h"
#include <cstring>   // for memcpy

namespace vulcan
{
namespace utils
{

float float_from_bytes(const char bytes[4])
{
    uint32_t floatBytes = char_to_uint32_t(bytes[0], bytes[1], bytes[2], bytes[3]);
    float data;

    memcpy(&data, &floatBytes, 4);

    return data;
}

uint16_t char_to_unsigned(unsigned char msb, unsigned char lsb)
{
    return static_cast<uint16_t>((msb << 8) + lsb);
}


int16_t char_to_signed(unsigned char msb, unsigned char lsb)
{
    return static_cast<int16_t>((msb << 8) + lsb);
}


uint32_t char_to_uint32_t(unsigned char byte1, unsigned char byte2, unsigned char byte3, unsigned char byte4)
{
    return static_cast<uint32_t>(byte1 << 24 | byte2 << 16 | byte3 << 8 | byte4);
}


void signed_to_char(int integer, char& msb, char& lsb)
{
    msb = (integer >> 8) & 0xFF;
    lsb = integer & 0xFF;
}


void uint16_to_char(uint16_t integer, char& msb, char& lsb)
{
    msb = (integer >> 8) & 0xFF;
    lsb = integer & 0xFF;
}

}   // namespace utils
}   // namespace vulcan

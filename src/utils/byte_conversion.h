/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     byte_conversion.h
* \author   Collin Johnson
*
* Declaration of functions for converting bytes into other types and vice-versa.
*/

#ifndef UTILS_BYTE_CONVERSION_H
#define UTILS_BYTE_CONVERSION_H

#include <cstdint>

namespace vulcan
{
namespace utils
{

float    float_from_bytes(const char bytes[4]);
uint16_t char_to_unsigned(unsigned char msb, unsigned char lsb);
int16_t  char_to_signed(unsigned char msb, unsigned char lsb);
uint32_t char_to_uint32_t(unsigned char byte1, unsigned char byte2, unsigned char byte3, unsigned char byte4);
void     signed_to_char(int integer, char& msb, char& lsb);
void     uint16_to_char(uint16_t integer, char& msb, char& lsb);

}
}

#endif // UTILS_BYTE_CONVERSION_H

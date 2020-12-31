/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UTILS_CONFIG_FILE_H
#define UTILS_CONFIG_FILE_H

#include "core/matrix.h"
#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace vulcan
{
namespace utils
{

/**
 * ConfigFile is a class that encapsulates the information contained in an .cfg file. The values from
 * the .cfg are loaded as key->value pairs, where both values are strings to allow maximum flexibility.
 *
 * In general, .cfg files should be used for any configuration values that could potentially change.
 * This allows for a design that avoids lots of hardcoded values that may need to be adjusted later
 * and would otherwise require recompilation
 *
 * The format of the .ini file is as follows:
 *
 *           # comment
 *           [headingA]
 *           key = value
 *           key = value
 *           [headingB]
 *           # comment
 *           key = value
 *           . . .
 *
 * Essential to this format is that each piece of the file is on its own line. Within each line, whitespace
 * can be whatever you desire, but only a single key-value pair or heading can appear on a single line.
 * Comments can only exist at the beginning of the line, otherwise the '#' will be included in the value read.
 * A valid .cfg file MUST include at least one heading
 */
class ConfigFile
{
public:
    /** Constructor for ConfigFile. */
    ConfigFile(const std::string& filename);

    /**
     * getFilename retrieves the filename associated with the contents of the ConfigFile.
     *
     * \return   The filename of .ini file.
     */
    std::string getFileName() const { return file; }

    /**
     * validate checks to see that the config file contains some required set of (heading,key) pairs.
     * Each pair that is not in the config will be output to std::cerr. If any are missing, then the
     * return value will be false.
     *
     * \param    required            Set of required (heading,key) pairs
     * \return   Number of missing (heading,key) pairs. 0 means everything is great!
     */
    int validate(const std::vector<std::pair<std::string, std::string>>& required) const;

    /**
     * hasValue checks to see if the given (heading,key) pair exists in the config file.
     */
    bool hasValue(const std::string& heading, const std::string& key) const;

    /** getValueAsXXXX retrieves the value associated with a particular key and heading. */
    std::string getValueAsString(const std::string& heading, const std::string& key) const;

    uint8_t getValueAsUInt8(const std::string& heading, const std::string& key) const;
    uint16_t getValueAsUInt16(const std::string& heading, const std::string& key) const;
    uint32_t getValueAsUInt32(const std::string& heading, const std::string& key) const;

    int8_t getValueAsInt8(const std::string& heading, const std::string& key) const;
    int16_t getValueAsInt16(const std::string& heading, const std::string& key) const;
    int32_t getValueAsInt32(const std::string& heading, const std::string& key) const;

    float getValueAsFloat(const std::string& heading, const std::string& key) const;
    double getValueAsDouble(const std::string& heading, const std::string& key) const;

    bool getValueAsBool(const std::string& heading, const std::string& key) const;

    /**
     * getValueAsMatrix retrieves the specified heading/key pair as a Matrix. Matrices are
     * defined similarly to the MATLAB matrix notation, where columns are separated by a single
     * space and rows are separated by semi-colons. Matrices are enclosed within []. The matrix
     * string can only be a single line.
     *
     * Example: {0 0; 1 1; 3 3;} defines a 3x2 matrix. The semi-colon after the final row is
     *          required.
     *
     * NOTE: The parser isn't meant to be robust, but will handle strings in the exact
     * format described above. Extra spaces WILL break things, so don't add them! If you want to
     * be sloppy, then write a better parser yourself.
     */
    Matrix getValueAsMatrix(const std::string& heading, const std::string& key) const;

private:
    /** Default constructor. Forbid its use because it doesn't make sense to have this given the layout of the class. */
    ConfigFile() { }

    /** parse processes the .ini file. */
    bool parse(std::ifstream& input);

    std::string file;   ///< Filename associated with the ConfigFile
    bool valid;         ///< True if the .ini file format was correct

    std::map<std::string, std::map<std::string, std::string>> contents;   ///< Contents of the .ini file
};

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_CONFIG_FILE_H

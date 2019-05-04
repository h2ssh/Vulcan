/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <cassert>
#include <cctype>
#include <iostream>
#include <cstdlib>
#include <utils/config_file.h>


using vulcan::utils::ConfigFile;


// typedef away to make life that much better
typedef std::map<std::string, std::map<std::string, std::string>> ConfigMap;
typedef std::pair<std::string, std::map<std::string, std::string>> ConfigPair;
typedef std::map<std::string, std::string> HeadingMap;


const char MATRIX_START_INDICATOR  = '{';
const char MATRIX_END_INDICATOR    = '}';
const char MATRIX_ROW_SEPARATOR    = ';';
const char MATRIX_COLUMN_SEPARATOR = ' ';


/** Pull the heading from a line read from the file. */
std::string extractHeading(const std::string& line);

/** Extract key->value pair from a line. */
void extractKeyValuePair(const std::string& line, std::string& key, std::string& value);

/** strip strips whitespace from the ends of a string. */
void strip(std::string& str);

/** parse_matrix parses a Matrix from a string. */
vulcan::Matrix parse_matrix                  (const std::string& matrixString);
std::string          extract_matrix_contents_string(const std::string& matrixString);
std::pair<int, int>  find_matrix_dimensions        (const std::string& matrixContents);
size_t extract_next_matrix_row_string(const std::string& matrixContents, size_t rowStartIndex, std::string& rowString);
void                 parse_matrix_row              (const std::string& matrixRowString, int row, vulcan::Matrix& matrix);


/**
* Constructor for ConfigFile. Attempts to open the specified file and parse its contents.
*
* \param    filename            Name of the .ini file to load
*/
ConfigFile::ConfigFile(const std::string& filename) : file(filename)
{
    // Attempt to open the file, if this fails, then throw an exception
    // Otherwise, parse the file and be done with it

    std::ifstream input(file.c_str());

    // ensure the file opened successfully
    if(!input.is_open())
    {
        std::cerr<<"Unable to open .cfg file: "<<file<<std::endl;
        // TODO: Throw an exception right here!!!!!  --- maybe
    }
    else
    {
        parse(input);
    }

    input.close();
}


int ConfigFile::validate(const std::vector<std::pair<std::string, std::string>>& required) const
{
    int missingValues = 0;

    for(auto pairIt = required.begin(), pairEnd = required.end(); pairIt != pairEnd; ++pairIt)
    {
        if(!hasValue(pairIt->first, pairIt->second))
        {
            std::cerr<<"ERROR: ConfigFile missing value ("<<pairIt->first<<','<<pairIt->second<<")\n";
            ++missingValues;
        }
    }

    return missingValues;
}


bool ConfigFile::hasValue(const std::string& heading, const std::string& key) const
{
    ConfigMap::const_iterator headIt = contents.find(heading);

    // if heading not found, return the empty string
    if(headIt == contents.end())
    {
        return false;
    }

    return headIt->second.find(key) != headIt->second.end();
}


std::string ConfigFile::getValueAsString(const std::string& heading, const std::string& key) const
{
    // First, find the heading, then search within that for the desired key

    ConfigMap::const_iterator headIt = contents.find(heading);

    // if heading not found, return the empty string
    if(headIt == contents.end())
    {
        return "";
    }

    HeadingMap::const_iterator keyIt = headIt->second.find(key);

    // check if the key was found
    if(keyIt == headIt->second.end())
    {
        return "";
    }
    else   // found the value, return it now
    {
        return keyIt->second;
    }
}


// NOTE: For these functions, what would be an appropriate error code? Hard to say.
uint8_t ConfigFile::getValueAsUInt8(const std::string& heading, const std::string& key) const
{
    std::string value = getValueAsString(heading, key);

    if(!value.empty())
    {
        return static_cast<uint8_t>(strtoul(value.c_str(), 0, 10));
    }
    else
    {
        return 0;
    }
}


uint16_t ConfigFile::getValueAsUInt16(const std::string& heading, const std::string& key) const
{
    std::string value = getValueAsString(heading, key);

    if(!value.empty())
    {
        return static_cast<uint16_t>(strtoul(value.c_str(), 0, 10));
    }
    else
    {
        return 0;
    }
}


uint32_t ConfigFile::getValueAsUInt32(const std::string& heading, const std::string& key) const
{
    std::string value = getValueAsString(heading, key);

    if(!value.empty())
    {
        return static_cast<uint32_t>(strtoul(value.c_str(), 0, 10));
    }
    else
    {
        return 0;
    }
}

int8_t ConfigFile::getValueAsInt8(const std::string& heading, const std::string& key) const
{
    std::string value = getValueAsString(heading, key);

    if(!value.empty())
    {
        return static_cast<int8_t>(strtol(value.c_str(), 0, 10));
    }
    else
    {
        return 0;
    }
}


int16_t ConfigFile::getValueAsInt16(const std::string& heading, const std::string& key) const
{
    std::string value = getValueAsString(heading, key);

    if(!value.empty())
    {
        return static_cast<int16_t>(strtol(value.c_str(), 0, 10));
    }
    else
    {
        return 0;
    }
}


int32_t ConfigFile::getValueAsInt32(const std::string& heading, const std::string& key) const
{
    std::string value = getValueAsString(heading, key);

    if(!value.empty())
    {
        return static_cast<int32_t>(strtol(value.c_str(), 0, 10));
    }
    else
    {
        return 0;
    }
}


float ConfigFile::getValueAsFloat(const std::string& heading, const std::string& key) const
{
    std::string value = getValueAsString(heading, key);

    if(!value.empty())
    {
        return strtof(value.c_str(), 0);
    }
    else
    {
        return 0;
    }
}


double ConfigFile::getValueAsDouble(const std::string& heading, const std::string& key) const
{
    std::string value = getValueAsString(heading, key);

    if(!value.empty())
    {
        return strtod(value.c_str(), 0);
    }
    else
    {
        return 0;
    }
}


bool ConfigFile::getValueAsBool(const std::string& heading, const std::string& key) const
{
    std::string value = getValueAsString(heading, key);

    if(!value.empty())
    {
        return strtol(value.c_str(), 0, 10) != 0;
    }
    else
    {
        return 0;
    }
}


vulcan::Matrix ConfigFile::getValueAsMatrix(const std::string& heading, const std::string& key) const
{
    std::string value = getValueAsString(heading, key);

    if(!value.empty())
    {
        return parse_matrix(value);
    }
    else
    {
        return vulcan::Matrix();
    }
}


/**
* parse parses the provided input stream, extracting the headings and key->value pairs. It is assumed
* that the input stream provided is formatted correctly.
*
* \param    input           Stream containing .ini file
* \return   True if the parsing is successful.
*/
bool ConfigFile::parse(std::ifstream& input)
{
    /*
    * To parse, read the file line-by-line and handle each of the three cases:
    * 1) Comment -- just go to next line
    * 2) Heading -- save the previous heading and map to the contents map, then start a new one
    * 3) Key     -- key is left of '=', value to right, create a pair and put in map for current heading
    */

    // make sure there is something to read
    if(!input.is_open())
    {
        std::cerr<<"ERROR: Cannot parse close file."<<std::endl;
        return false;
    }

    std::string nextLine;   // store next line read from stream here
    std::string curHeading("");  // current heading
    std::map<std::string, std::string> keys;   // map holding the key/value pairs

    ConfigPair headingPair;  // pair to insert when heading completes
    std::pair<std::string, std::string> keyPair;    // pair containing the key/value

    while(std::getline(input, nextLine))
    {
        if(nextLine.size() == 0)  // actually data to read?
        {
            continue;
        }

        switch(nextLine[0])
        {
        case '#': // comment line
            continue;
            break;
        case '[':  // new heading
            if(curHeading != "")  // if already processing a heading, save it to the contents map
            {
                headingPair.first  = curHeading;
                headingPair.second = keys;
                contents.insert(headingPair);
            }

            // set the new heading and empty out the values associated with the old heading
            curHeading = extractHeading(nextLine);
            if(curHeading == "")  // if heading extraction fails, then the contents were not valid
            {
                valid = false;
            }

            keys.clear();
            break;
        default:  // must be a key->value pair
            if(curHeading != "")  // if no heading, don't do anything
            {
                extractKeyValuePair(nextLine, keyPair.first, keyPair.second);

                if((keyPair.first == "") || (keyPair.second == ""))
                {
                    valid = false;
                }
                else
                {
                    keys.insert(keyPair);
                }
            }
        }
    }

    // at the end, if there is a current heading, it will not have been written to the contents, so add it
    if(curHeading != "")
    {
        headingPair.first  = curHeading;
        headingPair.second = keys;
        contents.insert(headingPair);
    }

    return true;
}


/**
* extractHeading extracts the heading from a line. The heading is the string between the first and
* last [] on the line. Whitespace is stripped from the edges of the extracted value.
*
* \param    line            Line containing the heading
* \return   The heading contained in the file as described above. If the line does not contain a heading
*           an empty string is returned.
*/
std::string extractHeading(const std::string& line)
{
    // Hunt down the location of the last ], then take the substring and strip whitespace from the edges

    size_t pos = line.rfind(']');
    if(pos == std::string::npos)   // if the ']' was not found, then invalid
    {
        std::cerr<<"ERROR: Line contains no heading: "<<line<<std::endl;
        return "";
    }

    std::string retVal = line.substr(1, pos - 1);
    strip(retVal);
    return retVal;
}


/**
* extractKeyValuePair extracts the key=value data from a line. The key is all data to the left of the
* '=' and the value is all data to the right of the '='. For both data, the whitespace is stripped from
* the ends of the string. If there is no '=', i.e. this is not a valid pair, then both key and value
* will be set to "", an empty string.
*
* \param    line             Line containing the pair
* \param    key              Store the key here
* \param    value            Store the value here
*/
void extractKeyValuePair(const std::string& line, std::string& key, std::string& value)
{
    // Search for the '=', then just grab the left and right portions and be happy

    size_t pos = line.find('=');
    if((pos == std::string::npos) || (pos == 0))  // ensure there is actually a pair
    {
        std::cerr<<"ERROR: No key->value pair on the line: "<<line<<std::endl;
        key = "";
        value = "";
        return;
    }

    // extract data as described above
    key = line.substr(0, pos);
    strip(key);
    value = line.substr(pos + 1);
    strip(value);
}


/**
* strip accepts a string and removes whitespace from the edges of the string. Whitespace is any
* character that returns true when passed to the isspace() function defined in <cctype>.
*
* \param    str         String to be stripped
*/
void strip(std::string& str)
{
    // First, find the first non-whitespace character. Then, find last non-whitespace character.
    // set str to be the string in between these two positions.

    int firstPos = 0;
    int lastPos = str.length();

    // get the first non-whitespace
    for(firstPos = 0; firstPos < lastPos; ++firstPos)
    {
        if(!isspace(str[firstPos]))
        {
            break;
        }
    }

    // if x == len, then str contains only whitespace, and str == "", then bailout early
    if(firstPos == lastPos)
    {
        str = "";
        return;
    }

    // find last non-whitespace
    while(--lastPos >= 0)
    {
        if(!isspace(str[lastPos]))
        {
            break;
        }
    }

    str = str.substr(firstPos, lastPos - firstPos + 1);
}


/** parse_matrix parses a Matrix from a string. */
vulcan::Matrix parse_matrix(const std::string& matrixString)
{
    std::string matrixContents  = extract_matrix_contents_string(matrixString);
    std::string matrixRowString;

    std::pair<int, int>  dimensions = find_matrix_dimensions(matrixContents);

    vulcan::Matrix matrix(dimensions.first, dimensions.second);

    size_t nextRowStart = extract_next_matrix_row_string(matrixContents, 0, matrixRowString);

    int row = 0;

    while(!matrixRowString.empty())
    {
        parse_matrix_row(matrixRowString, row++, matrix);

        nextRowStart = extract_next_matrix_row_string(matrixContents, nextRowStart, matrixRowString);
    }

    return matrix;
}


std::string extract_matrix_contents_string(const std::string& matrixString)
{
    size_t startIndex = matrixString.find(MATRIX_START_INDICATOR, 0);
    size_t endIndex   = matrixString.find(MATRIX_END_INDICATOR, startIndex);

    std::string contents;

    if((startIndex != std::string::npos) && (endIndex != std::string::npos))
    {
        contents = matrixString.substr(startIndex+1, endIndex-startIndex-1);

        strip(contents);
    }

    return contents;
}


std::pair<int, int> find_matrix_dimensions(const std::string& matrixContents)
{
    // Number of rows is the number of MATRIX_ROW_SEPARATORS
    // Number of columns is the number of MATRIX_COLUMN_SEPARATORS until the first MATRIX_ROW_SEPARATOR plus one

    std::pair<int, int> dimensions(0, 0);

    for(size_t i = 0; i < matrixContents.length(); ++i)
    {
        if(matrixContents[i] == MATRIX_COLUMN_SEPARATOR)
        {
            ++dimensions.second;
        }
        // There isn't a space after the last column value, so the row separator defines where it ends
        else if(matrixContents[i] == MATRIX_ROW_SEPARATOR)
        {
            ++dimensions.second;
            break;
        }
    }

    for(size_t i = 0; i < matrixContents.length(); ++i)
    {
        if(matrixContents[i] == MATRIX_ROW_SEPARATOR)
        {
            ++dimensions.first;
        }
    }

    assert(dimensions.first != 0 && dimensions.second != 0);

    return dimensions;
}


size_t extract_next_matrix_row_string(const std::string& matrixContents, size_t rowStartIndex, std::string& rowString)
{
    size_t endIndex = matrixContents.find(MATRIX_ROW_SEPARATOR, rowStartIndex);

    rowString.clear();

    if(endIndex != std::string::npos)
    {
        rowString = matrixContents.substr(rowStartIndex, endIndex-rowStartIndex);
        strip(rowString);

        ++endIndex;
    }

    return endIndex;
}


void parse_matrix_row(const std::string& matrixRowString, int row, vulcan::Matrix& matrix)
{
    unsigned int column = 0;

    size_t valueStart = 0;
    size_t valueEnd   = 0;

    std::string valueString;

    do
    {
        valueEnd    = matrixRowString.find(MATRIX_COLUMN_SEPARATOR, valueStart);
        valueString = matrixRowString.substr(valueStart, valueEnd-valueStart);
        valueStart  = valueEnd+1;

        matrix(row, column) = strtof(valueString.c_str(), 0);

        ++column;
    } while(column < matrix.n_cols);
}

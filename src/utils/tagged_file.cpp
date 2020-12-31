/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     tagged_file.cpp
* \author   Collin Johnson
*
* Definition of TaggedFile.
*/

#include <fstream>
#include "utils/tagged_file.h"

namespace vulcan
{
namespace utils
{

void extract_tags(const std::string& tagged, TaggedMap& out);


TaggedFile::TaggedFile(const std::string& filename)
{
    parseFile(filename);
}


std::vector<TaggedMap> TaggedFile::getNestedContents(const std::string& tag) const
{
    std::vector<TaggedMap> contents;

    std::pair<ParsedMap::const_iterator, ParsedMap::const_iterator> tagIts = tags.equal_range(tag);

    for(auto startIt = tagIts.first, endIt = tagIts.second; startIt != endIt; ++startIt)
    {
        contents.push_back(startIt->second);
    }

    return contents;
}


std::vector<std::string> TaggedFile::getTagValues(const TaggedMap& taggedValues, const std::string& valueTag)
{
    std::vector<std::string> values;
    
    auto range = taggedValues.equal_range(valueTag);
    
    while(range.first != range.second)
    {
        values.push_back(range.first->second);
        ++range.first;
    }
    
    return values;
}


void TaggedFile::parseFile(const std::string& filename)
{
    // To parse a stream, load the contents into a string and pass it on to the parse_tagged_string function
    std::ifstream in(filename.c_str());
    
    if(!in.is_open())
    {
        return;
    }
    
    // This probably isn't the fastest method for doing this, but it works
    std::string temp;
    std::string contents("");
    
    while(!in.eof())
    {
        std::getline(in, temp);
        
        // Only add it in there if it isn't a commented line
        if(temp.size() > 0 && temp[0] != '#')
        {
            contents += temp;
        }
    }
    
    parseString(contents);
}


void TaggedFile::parseString(const std::string& contents)
{
    /*
    * To parse the string, just recursively call the extract_tags function. First, call it on the highest
    * level to get the main tags, then for each of these tags, call it again and store the results in
    * out.
    */
    
    // get the tags for the first level of values and the associated strings for the second level
    TaggedMap firstTier;
    extract_tags(contents, firstTier);
    
    // Iterate through the first tier and parse all the second tier tags
    TaggedMap secondTemp;
    for(auto tagIt = firstTier.begin(), endIt = firstTier.end(); tagIt != endIt; ++tagIt)
    {
        secondTemp.clear();
        extract_tags(tagIt->second, secondTemp);
        tags.insert(std::make_pair(tagIt->first, secondTemp));
    }
}


void extract_tags(const std::string& tagged, TaggedMap& out)
{
    /*
     * To extract the tags, first search for the first tag -- the first statement between <>. From this,
     * find the accompanying closing tag (within the <>) and place all value in-between in the value
     * portion of the map tag as the key. Repeat this until there end of the string is reached.
     */
    std::string tag;
    std::string endTag;
    std::string::size_type tagStart = 0;
    std::string::size_type tagEnd = 0;
    
    for(std::string::size_type strLen = tagged.length(); tagStart < strLen;)
    {
        // Get the tag
        tagStart = tagged.find("<", tagEnd);
        tagEnd = tagged.find(">", tagStart);
        tag = tagged.substr(tagStart + 1, tagEnd - tagStart - 1);
        
        endTag = "</" + tag + ">";
        tagStart = tagged.find(endTag, tagEnd);
        
        if(tagStart == std::string::npos)
        {
            break;
        }
        
        out.insert(std::make_pair(tag, tagged.substr(tagEnd + 1, tagStart - tagEnd - 1)));
        
        tagEnd = tagStart + endTag.length();
    }
}

} // namespace utils
} // namespace vulcan

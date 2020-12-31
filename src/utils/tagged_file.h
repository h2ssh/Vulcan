/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     tagged_file.h
 * \author   Collin Johnson
 *
 * Declaration of TaggedFile, a simple abstraction of a singly-nested tagged file.
 */

#ifndef UTILS_TAGGED_FILE_H
#define UTILS_TAGGED_FILE_H

#include <map>
#include <string>
#include <vector>

namespace vulcan
{
namespace utils
{

typedef std::multimap<std::string, std::string> TaggedMap;

/**
 * TaggedFile is a simple abstraction of a file with tagged values, a la HTML or XML. The
 * tags must be nested one deep.
 *
 * The format of the tagged contents must be that of tagged data in an XML-type format where
 * the nesting is only two levels deep. The following is a clearer explanation:
 *
 * <tagA>                    <----- parent tag
 *       <tagB>blah</tagB>   <----- child tag
 *       <tagC>blah</tagC>
 *       <tagD>foo</tagD>
 * </tagA>
 *    .
 *    .
 *    .
 *
 * Tags can be repeated as long as the format remains exactly as above.
 *
 * Whitespace will adversely affect the results generated, so make sure that it is consistent. The
 * best way to ensure this is to keep the format exactly as described above.
 *
 * Also, the parser will not work correctly if, within the value of the outer tag is a tag of the
 * same name, i.e. <tagA><tagA>asdf</tagA></tagA>. This is a simple parser so this case is not
 * processed correctly. Sorry.
 *
 * TODO: Make this file much much much better. Gets the job done, but isn't particularly automated at the moment.
 */
class TaggedFile
{
public:
    /**
     * Constructor for TaggedFile.
     *
     * \param    filename            Filename from which the contents should be loaded and parsed
     */
    TaggedFile(const std::string& filename);

    /**
     * getTagContents retrieves the set of all nested tags for the parent tag.
     *
     * \param    tag         Tag for which to retrieve the internal contents
     * \return   TaggedMaps with the contents of the nested tags.
     */
    std::vector<TaggedMap> getNestedContents(const std::string& tag) const;

    /**
     * getTagValues retrieves the set of values associated a tag within a tagged map.
     *
     * \param    taggedValues    A map of tagged values
     * \param    valueTag        Tag identifying the specific data
     * \return   A sequence of the values associated with the innerTag. Each value represents one instance of
     *           the value tag.
     */
    static std::vector<std::string> getTagValues(const TaggedMap& taggedValues, const std::string& valueTag);

private:
    typedef std::multimap<std::string, TaggedMap> ParsedMap;

    void parseFile(const std::string& filename);
    void parseString(const std::string& contents);

    ParsedMap tags;
};

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_TAGGED_FILE_H

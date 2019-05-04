/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     topo_training_data.h
 * \author   Collin Johnson
 *
 * Declaration of TopoTrainingData.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_TRAINING_TOPO_TRAINING_DATA_H
#define HSSH_LOCAL_TOPOLOGICAL_TRAINING_TOPO_TRAINING_DATA_H

#include <algorithm>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <cassert>

namespace vulcan
{
namespace hssh
{

/**
* TopoTrainingData contains a set of labeled areas and their features for one or more completed maps. The data is
* organized both on a per-map and a complete basis. The data consists of a sequence of LabeledData, each of which
* contains a set of features and a hand-assigned label.
*
* Concept:
*
*   LabeledData:
*       .features.version() exists
*       operator==
*       operator!=
*       operator>>
*       operator<<
*/
template <class LabeledData>
class TopoTrainingData
{
public:

    using FeatureIter = typename std::vector<LabeledData>::const_iterator;
    using MapIter     = std::vector<std::string>::const_iterator;

    // All default constructors are supported
    TopoTrainingData(void) = default;
    TopoTrainingData(const TopoTrainingData& rhs) = default;
    TopoTrainingData(TopoTrainingData&& rhs) = default;
    TopoTrainingData& operator=(const TopoTrainingData& rhs) = default;
    TopoTrainingData& operator=(TopoTrainingData&& rhs) = default;

    /**
    * addExample adds a single example to the collection of data.
    *
    * \param    mapName         Name of the map associated with the example
    * \param    example         Example to add.
    */
    void addExample(const std::string& mapName, const LabeledData& example);

    /**
    * addExamples adds the examples from a map to the stored data. If data associated with the map already exists, then
    * the examples will be appended to that data.
    *
    * This method invalidates all iterators.
    *
    * Any spaces in mapName will be converted to underscores.
    *
    * \param    mapName         Name of the map in which these examples were captured
    * \param    beginExamples   Start of the range of examples
    * \param    endExamples     End of the range of examples
    */
    void addExamples(std::string mapName, FeatureIter beginExamples, FeatureIter endExamples);

    /**
    * addExamples adds all examples from an existing collection of labeled areas to this collection of labeled areas.
    *
    * This method invalidates all iterators.
    *
    * \param    data            Data to be added to the collection
    */
    void addExamples(const TopoTrainingData& data);

    /**
    * version returns the version number of the features contained in the labeled data.
    */
    int version(void) const;

    // Iterator support for LabeledData
    std::size_t size(void) const { return examples_.size(); }
    bool empty(void) const { return examples_.empty(); }
    FeatureIter begin(void) const { return examples_.begin(); }
    FeatureIter end(void) const { return examples_.end(); }
    const LabeledData& operator[](int n) const { return examples_[n]; }
    void clear(void)
    {
        examples_ = std::vector<LabeledData>();
        maps_ = std::vector<std::string>();
        mapRanges_ = std::vector<MapExampleRange>();
    }

    // Iterator support for the maps contained in the data
    std::size_t sizeMaps(void) const { return maps_.size(); }
    MapIter beginMaps(void) const { return maps_.begin(); }
    MapIter endMaps(void) const { return maps_.end(); }

    // Iterator support for the examples associated with a particular map
    // Creating these iterators is more expensive than other iterators, so the results should be cached for a given
    // use period
    std::size_t sizeMapExamples(const std::string& mapName) const;
    FeatureIter beginMapExamples(const std::string& mapName) const;
    FeatureIter endMapExamples(const std::string& mapName) const;

    /**
    * findMapExamples retrieves all examples associated with the specified map.
    *
    * The examples are returned as a new instance of TopoTrainingData with sizeMaps() <= 1. If no map with the given
    * name is found, size() == 0 for the returned instance.
    *
    * \param    mapName         Name of the map for which to find examples
    * \return   An instance of TopoTrainingData containing all examples associated with the specified map name.
    */
    TopoTrainingData findMapExamples(const std::string& mapName) const;

    /**
    * removeMapExamples removes all examples associated with the specified map name.
    *
    * \param    mapName         Name of the map for which to remove all examples
    * \return   Number of examples removed.
    */
    int removeMapExamples(const std::string& mapName);

    // I/O operators for TopoTrainingData
    template <class T>
    friend std::ostream& operator<<(std::ostream& out, const TopoTrainingData<T>& data);

    template <class T>
    friend std::istream& operator>>(std::istream& in, TopoTrainingData<T>& data);

private:

    // .first = first index, .second = number of entries
    using MapExampleRange = std::pair<int, int>;

    std::vector<LabeledData> examples_;
    std::vector<std::string> maps_;
    std::vector<MapExampleRange> mapRanges_;

    // INVARIANT: maps_.size() == mapRanges_.size()

    int mapIndex(const std::string& mapName) const; // -1 if not found
};

// Operators
/**
* Equality operator. Two sets of data are equal if they contain the same maps and same examples assigned to those maps.
*
* FYI: This is a costly operation to check.
*/
template <class LabeledData>
bool operator==(const TopoTrainingData<LabeledData>& lhs, const TopoTrainingData<LabeledData>& rhs);

template <class LabeledData>
bool operator!=(const TopoTrainingData<LabeledData>& lhs, const TopoTrainingData<LabeledData>& rhs);


/*
 * The format for TopoTrainingData<LabeledData> is:
 *
 *   num_examples
 *   map_name example
 *   map_name example
 *   map_name example
 *       .
 *       .
 *       .
 */
template <class LabeledData>
std::ostream& operator<<(std::ostream& out, const TopoTrainingData<LabeledData>& data)
{
    assert(data.maps_.size() == data.mapRanges_.size());

    if(data.empty())
    {
        return out;
    }

    out << data.examples_.size() << ' ' << data.begin()->features.version() << '\n';

    for(std::size_t mapIndex = 0; mapIndex < data.maps_.size(); ++mapIndex)
    {
        int rangeStart = data.mapRanges_[mapIndex].first;

        // For each example in the map's range, add it to the stream
        for(int n = 0; n < data.mapRanges_[mapIndex].second; ++n)
        {
            assert(rangeStart + n < static_cast<int>(data.examples_.size()));

            out << data.maps_[mapIndex] << ' ' << data.examples_[rangeStart + n] << '\n';
        }
    }

    return out;
}


template <class LabeledData>
std::istream& operator>>(std::istream& in, TopoTrainingData<LabeledData>& data)
{
    std::map<std::string, std::vector<LabeledData>> mapToExamples;

    int numExamples;
    int featuresVersion;
    std::string mapName;
    LabeledData example;

    in >> numExamples >> featuresVersion;

    // Associate each example in the file with the appropriate map
    for(int n = 0; (n < numExamples) && in.good(); ++n)
    {
        // Ensure there's actually data in the file
        assert(in.good());

        in >> mapName >> example;

        // Only add examples if the feature version matches
        if(example.features.version() == featuresVersion)
        {
            mapToExamples[mapName].push_back(example);
        }
        // Else exit immediately because we're using stale feature data
        else
        {
            break;
        }
    }

    // Add each map's examples to the data
    for(auto& examples : mapToExamples)
    {
        data.addExamples(examples.first, examples.second.begin(), examples.second.end());
    }

    return in;
}


template <class LabeledData>
bool operator==(const TopoTrainingData<LabeledData>& lhs, const TopoTrainingData<LabeledData>& rhs)
{
    // Both sets of data must have the same number of elements and maps contained in them
    if((lhs.size() != rhs.size()) || (lhs.sizeMaps() != rhs.sizeMaps()))
    {
        return false;
    }

    // If both sides have a single map, then just directly compare all the examples to see if they are the same
    if((lhs.sizeMaps() == 1) && (rhs.sizeMaps() == 1))
    {
        // Make sure all examples are equal
        for(std::size_t n = 0; n < lhs.size(); ++n)
        {
            if(*(lhs.begin() + n) != *(rhs.begin() + n))
            {
                return false;
            }
        }

        return true;
    }

    // Otherwise, for each map name in lhs, recursively compare via == to see that they are the same by extracting the
    // examples for each map name and comparing just those

    // Make sure map names are equal
    for(std::size_t n = 0; n < lhs.sizeMaps(); ++n)
    {
        if(lhs.findMapExamples(*(lhs.beginMaps() + n)) != rhs.findMapExamples(*(lhs.beginMaps() + n)))
        {
            return false;
        }
    }

    return true;
}


template <class LabeledData>
bool operator!=(const TopoTrainingData<LabeledData>& lhs, const TopoTrainingData<LabeledData>& rhs)
{
    return !(lhs == rhs);
}


///////////////// TopoTrainingData<LabeledData> //////////////////////

template <class LabeledData>
void TopoTrainingData<LabeledData>::addExample(const std::string& mapName, const LabeledData& example)
{
    // Use the vector addExamples because it has all the internal checks.
    std::vector<LabeledData> e = { example };
    addExamples(mapName, e.begin(), e.end());
}


template <class LabeledData>
void TopoTrainingData<LabeledData>::addExamples(std::string mapName, FeatureIter beginExamples, FeatureIter endExamples)
{
    std::replace(mapName.begin(), mapName.end(), ' ', '_');

    auto mapIt = std::find(maps_.begin(), maps_.end(), mapName);
    int numValidExamples = std::distance(beginExamples, endExamples);

    // If the map exists in the list, then these examples need to be appended to the end of the map's range.
    // All subsequence ranges will then need to have their start index increased by the number of examples added
    if(mapIt != maps_.end())
    {
        std::size_t index = std::distance(maps_.begin(), mapIt);
        int endOfRange = mapRanges_[index].first + mapRanges_[index].second;

        examples_.insert(examples_.begin() + endOfRange, beginExamples, endExamples);

        mapRanges_[index].second += numValidExamples;

        for(std::size_t n = index + 1; n < mapRanges_.size(); ++n)
        {
            mapRanges_[n].first += numValidExamples;
        }
    }
    // Append the new examples to the back of the maps
    else
    {
        int examplesStart = examples_.size();
        mapRanges_.emplace_back(std::make_pair(examplesStart, numValidExamples));
        maps_.push_back(mapName);
        examples_.insert(examples_.end(), beginExamples, endExamples);
    }
}


template <class LabeledData>
void TopoTrainingData<LabeledData>::addExamples(const TopoTrainingData<LabeledData>& data)
{
    // For each map in data, add all examples for that map via the addExamples(map, begin, end) method
    for(std::size_t n = 0; n < data.maps_.size(); ++n)
    {
        addExamples(data.maps_[n],
                    data.examples_.begin() + data.mapRanges_[n].first,
                    data.examples_.begin() + data.mapRanges_[n].first + data.mapRanges_[n].second);
    }
}


template <class LabeledData>
int TopoTrainingData<LabeledData>::version(void) const
{
    return examples_.empty() ? -1 : examples_.front().features.version();
}


template <class LabeledData>
std::size_t TopoTrainingData<LabeledData>::sizeMapExamples(const std::string& mapName) const
{
    int index = mapIndex(mapName);
    return (index >= 0) ? mapRanges_[index].second : 0;
}


template <class LabeledData>
typename TopoTrainingData<LabeledData>::FeatureIter
TopoTrainingData<LabeledData>::beginMapExamples(const std::string& mapName) const
{
    int index = mapIndex(mapName);
    return (index >= 0) ? examples_.begin() + mapRanges_[index].first : examples_.end();
}


template <class LabeledData>
typename TopoTrainingData<LabeledData>::FeatureIter
TopoTrainingData<LabeledData>::endMapExamples(const std::string& mapName) const
{
    int index = mapIndex(mapName);
    return (index >= 0) ? examples_.begin() + mapRanges_[index].first + mapRanges_[index].second : examples_.end();
}


template <class LabeledData>
TopoTrainingData<LabeledData> TopoTrainingData<LabeledData>::findMapExamples(const std::string& mapName) const
{
    TopoTrainingData<LabeledData> mapData;
    int index = mapIndex(mapName);

    if(index >= 0)
    {
        mapData.addExamples(maps_[index],
                            examples_.begin() + mapRanges_[index].first,
                            examples_.begin() + mapRanges_[index].first + mapRanges_[index].second);
    }

    return mapData;
}


template <class LabeledData>
int TopoTrainingData<LabeledData>::removeMapExamples(const std::string& mapName)
{
    int index = mapIndex(mapName);

    // If the map exists in the list, then these examples need to be erased. to the end of the map's range.
    // All subsequence ranges will then need to have their start index decreased by the number of examples removed
    if(index >= 0)
    {
        // Erase the examples associated with the map
        int endOfRange = mapRanges_[index].first + mapRanges_[index].second;
        examples_.erase(examples_.begin() + mapRanges_[index].first, examples_.begin() + endOfRange);

        int numExamplesRemoved = mapRanges_[index].second;

        for(std::size_t n = index + 1; n < mapRanges_.size(); ++n)
        {
            mapRanges_[n].first -= numExamplesRemoved;
        }

        // Erase evidence of the map
        maps_.erase(maps_.begin() + index);
        mapRanges_.erase(mapRanges_.begin() + index);

        return numExamplesRemoved;
    }

    return 0;
}

template <class LabeledData>
int TopoTrainingData<LabeledData>::mapIndex(const std::string& mapName) const
{
    // Find the map name in the collection of maps
    auto mapIt = std::find(maps_.begin(), maps_.end(), mapName);
    // If the name is found, the distance to the iterator is what's needed
    return (mapIt != maps_.end()) ? std::distance(maps_.begin(), mapIt) : -1;
}


} // namespace hssh
} // namepace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_TRAINING_TOPO_TRAINING_DATA_H

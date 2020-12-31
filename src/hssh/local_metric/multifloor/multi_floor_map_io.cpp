/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     multi_floor_map_io.cpp
* \author   Collin Johnson
*
* Definition of MultiFloorMapIO::save and MultiFloorMapIO::load.
*/

#include "hssh/local_metric/multifloor/multi_floor_map_io.h"
#include "hssh/local_metric/lpm_io.h"
#include "hssh/local_metric/multi_floor_map.h"
#include "utils/config_file_utils.h"
#include "utils/tagged_file.h"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cassert>

namespace vulcan
{
namespace hssh
{

const std::string MAP_INFO_TAG         ("map_info");
const std::string MAP_INFO_ID_TAG      ("id");
const std::string MAP_INFO_FLOOR_TAG   ("current_floor");
const std::string MAP_INFO_ELEVATOR_TAG("current_elevator");

const std::string FLOOR_TAG          ("floor");
const std::string FLOOR_ID_TAG       ("id");
const std::string FLOOR_ELEVATORS_TAG("elevators");
const std::string FLOOR_LPM_TAG      ("lpm");

const std::string ELEVATOR_TAG           ("elevator");
const std::string ELEVATOR_ID_TAG        ("id");
const std::string ELEVATOR_FLOORS_TAG    ("floors");
const std::string ELEVATOR_CURRENT_TAG   ("current_floor");
const std::string ELEVATOR_TRANSITION_TAG("transition");
const std::string ELEVATOR_BOUNDARY_TAG  ("boundary");


void save_map_info(const MultiFloorMap&             map,      std::ofstream& out);
void save_elevator(const std::shared_ptr<Elevator>& elevator, std::ofstream& out);
void save_floor   (const std::shared_ptr<Floor>&    floor,    std::ofstream& out);

std::tuple<int, int, int> load_map_info(const utils::TaggedMap& tags);
std::shared_ptr<Elevator> load_elevator(const utils::TaggedMap& tags);
std::shared_ptr<Floor>    load_floor   (const utils::TaggedMap& tags, const std::vector<std::shared_ptr<Elevator>>& elevators);


bool MultiFloorMapIO::save(const MultiFloorMap& map, const std::string& filename)
{
    std::ofstream file(filename);

    if(!file.is_open())
    {
        return false;
    }

    save_map_info(map, file);

    for(const auto& elevator : map.getElevators())
    {
        save_elevator(elevator, file);
    }

    for(const auto& floor : map.getFloors())
    {
        save_floor(floor, file);
    }

    return true;
}


MultiFloorMap MultiFloorMapIO::load(const std::string& filename)
{
    utils::TaggedFile file(filename);

    auto mapInfoTags  = file.getNestedContents(MAP_INFO_TAG);
    auto elevatorTags = file.getNestedContents(ELEVATOR_TAG);
    auto floorTags    = file.getNestedContents(FLOOR_TAG);

    if(mapInfoTags.empty())
    {
        return MultiFloorMap();
    }

    assert(mapInfoTags.size() == 1);
    // Don't actually need floors or elevators if the map is empty

    std::vector<std::shared_ptr<Elevator>> elevators;
    std::vector<std::shared_ptr<Floor>>    floors;

    auto mapInfo = load_map_info(mapInfoTags.front());

    for(const auto& tags : elevatorTags)
    {
        elevators.push_back(load_elevator(tags));
    }

    for(const auto& tags : floorTags)
    {
        floors.push_back(load_floor(tags, elevators));
    }

    return MultiFloorMap(std::get<1>(mapInfo), std::get<2>(mapInfo), floors, elevators);
}


void save_map_info(const MultiFloorMap& map, std::ofstream& out)
{
    out<< '<' << MAP_INFO_TAG << ">\n"
       << "\t<" << MAP_INFO_ID_TAG       << '>' << 0                        << "</" << MAP_INFO_ID_TAG       << ">\n"
       << "\t<" << MAP_INFO_FLOOR_TAG    << '>' << map.getCurrentFloor()    << "</" << MAP_INFO_FLOOR_TAG    <<">\n"
       << "\t<" << MAP_INFO_ELEVATOR_TAG << '>' << map.getCurrentElevator() << "</" << MAP_INFO_ELEVATOR_TAG <<">\n"
       << "</" << MAP_INFO_TAG << ">\n";
}


void save_elevator(const std::shared_ptr<Elevator>& elevator, std::ofstream& out)
{
    std::vector<int> floorIds;

    out << '<' << ELEVATOR_TAG << ">\n"
        << "\t<" << ELEVATOR_ID_TAG      << ">" << elevator->getId()           << "</" << ELEVATOR_ID_TAG      << ">\n"
        << "\t<" << ELEVATOR_CURRENT_TAG << ">" << elevator->getCurrentFloor() << "</" << ELEVATOR_CURRENT_TAG << ">\n";

    // boundary = int->rectangle
    for(const auto& boundary : elevator->getBoundaries())
    {
        floorIds.push_back(boundary.first);

        out << "\t<" << ELEVATOR_BOUNDARY_TAG << ">" << boundary.first << ' '
                                                     << boundary.second.bottomLeft.x << ' ' << boundary.second.bottomLeft.y << ' '
                                                     << boundary.second.topRight.x << ' ' << boundary.second.topRight.y << ' '
            << "</" << ELEVATOR_BOUNDARY_TAG << ">\n";
    }

    // transitions = int -> vec<floor_transition_t>
    for(const auto& floor : elevator->getTransitions())
    {
        for(const auto& transition : floor.second)
        {
            out << "\t<" << ELEVATOR_TRANSITION_TAG << ">" << floor.first << ' '
                                                           << transition.endFloor << ' '
                                                           << transition.height
                << "</" << ELEVATOR_TRANSITION_TAG << ">\n";
        }
    }

    out << "\t<" << ELEVATOR_FLOORS_TAG << '>';

    for(auto id : floorIds)
    {
        out << id << ' ';
    }

    out << "</" << ELEVATOR_FLOORS_TAG << ">\n"
        << "</" << ELEVATOR_TAG << ">\n";
}


void save_floor(const std::shared_ptr<Floor>& floor, std::ofstream& out)
{
    out << '<' << FLOOR_TAG << ">\n"
        << "\t<" << FLOOR_ID_TAG  << ">" << floor->getId()      << "</" << FLOOR_ID_TAG      << ">\n"
        << "\t<" << FLOOR_LPM_TAG << ">" << floor->getLPMName() << "</" << FLOOR_LPM_TAG << ">\n";

    out << "\t<" << FLOOR_ELEVATORS_TAG << '>';

    for(const auto& elevator : floor->getElevators())
    {
        out << elevator->getId() << ' ';
    }

    out << "</" << FLOOR_ELEVATORS_TAG << ">\n"
        << "</" << FLOOR_TAG << ">\n";

    save_lpm_1_0(floor->getLPM(), floor->getLPMName());
}


std::tuple<int, int, int> load_map_info(const utils::TaggedMap& tags)
{
    auto idStr       = utils::TaggedFile::getTagValues(tags, MAP_INFO_ID_TAG);
    auto floorStr    = utils::TaggedFile::getTagValues(tags, MAP_INFO_FLOOR_TAG);
    auto elevatorStr = utils::TaggedFile::getTagValues(tags, MAP_INFO_ELEVATOR_TAG);

    assert(!idStr.empty());
    assert(!floorStr.empty());
    assert(!elevatorStr.empty());

    int id       = atoi(idStr.front().c_str());
    int floor    = atoi(floorStr.front().c_str());
    int elevator = atoi(elevatorStr.front().c_str());

    return std::make_tuple(id, floor, elevator);
}


std::shared_ptr<Elevator> load_elevator(const utils::TaggedMap& tags)
{
    auto idStr         = utils::TaggedFile::getTagValues(tags, ELEVATOR_ID_TAG);
    auto currentStr    = utils::TaggedFile::getTagValues(tags, ELEVATOR_CURRENT_TAG);
    auto floorsStr     = utils::TaggedFile::getTagValues(tags, ELEVATOR_FLOORS_TAG);
    auto boundaryStr   = utils::TaggedFile::getTagValues(tags, ELEVATOR_BOUNDARY_TAG);
    auto transitionStr = utils::TaggedFile::getTagValues(tags, ELEVATOR_TRANSITION_TAG);

    assert(!idStr.empty());
    assert(!currentStr.empty());
    assert(!floorsStr.empty());
    assert(!boundaryStr.empty());
    // Don't necessarily have a transition if the elevator exists. Could have saved before moving on the elevator

    std::vector<int>                               floors;
    std::map<int, math::Rectangle<float>>          boundaries;
    std::map<int, std::vector<floor_transition_t>> transitions;

    int id      = atoi(idStr.front().c_str());
    int current = atoi(currentStr.front().c_str());

    std::vector<std::string> strings = utils::split_into_strings(floorsStr.front(), ' ');
    for(auto str : strings)
    {
        if(str.length())
        {
            floors.push_back(std::stoi(str));
        }
    }

    for(const auto& boundaryValue : boundaryStr)
    {
        std::istringstream boundaryIn(boundaryValue);
        int                id;
        Point<float> bottomLeft;
        Point<float> topRight;

        boundaryIn >> id >> bottomLeft.x >> bottomLeft.y >> topRight.x >> topRight.y;
        boundaries.insert(std::make_pair(id, math::Rectangle<float>(bottomLeft, topRight)));
    }

    for(const auto& transitionValue : transitionStr)
    {
        std::istringstream transitionIn(transitionValue);
        int    start;
        int    end;
        double height;

        transitionIn >> start >> end >> height;

        auto transIt = transitions.find(start);
        if(transIt != transitions.end())
        {
            transIt->second.push_back({start, end, height});
        }
        else
        {
            decltype(transIt->second) newFloorTransitions;
            newFloorTransitions.push_back({start, end, height});
            transitions.insert(std::make_pair(start, newFloorTransitions));
        }
    }

    return std::shared_ptr<Elevator>(new Elevator(id, current, boundaries, transitions));
}


std::shared_ptr<Floor> load_floor(const utils::TaggedMap& tags, const std::vector<std::shared_ptr<Elevator>>& elevators)
{
    auto idStr        = utils::TaggedFile::getTagValues(tags, FLOOR_ID_TAG);
    auto elevatorsStr = utils::TaggedFile::getTagValues(tags, FLOOR_ELEVATORS_TAG);
    auto lpmStr       = utils::TaggedFile::getTagValues(tags, FLOOR_LPM_TAG);

    assert(!idStr.empty());
    assert(!lpmStr.empty());
    // A floor can exist without any elevators

    int                id      = atoi(idStr.front().c_str());
    std::string        lpmName = lpmStr.front();
    LocalPerceptualMap lpm;

    load_lpm_1_0(lpmName, lpm);

    std::shared_ptr<Floor> floor(new Floor(id, lpm, lpmName));

    std::vector<std::string> strings = utils::split_into_strings(elevatorsStr.front(), ' ');
    for(auto str : strings)
    {
        if(str.empty())
        {
            continue;
        }

        int elevatorId = std::stoi(str);

        auto elevatorComp = [elevatorId](const std::shared_ptr<Elevator>& elevator) { return elevator->getId() == elevatorId; };

        auto eleIt = std::find_if(elevators.begin(), elevators.end(), elevatorComp);

        assert(eleIt != elevators.end());

        floor->addElevator(*eleIt);
    }

    return floor;
}

} // namespace hssh
} // namespace vulcan

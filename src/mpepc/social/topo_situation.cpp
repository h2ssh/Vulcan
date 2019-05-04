/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topo_situation.cpp
* \author   Collin Johnson
*
* Definition of PathSituation, PlaceSituation, and TopoSituationResponse.
*/

#include <mpepc/social/topo_situation.h>
#include <mpepc/social/social_norm_utils.h>
#include <hssh/local_topological/local_topo_map.h>

namespace vulcan
{
namespace mpepc
{

const double kMaxStationaryVel = 0.25;  // maximum velocity of an agent for it to be considered stationary


struct path_situation_matcher_t : public boost::static_visitor<bool>
{
    const PathSituation* situation;

    // Match a path if they have the same configuration of lateral bins as represented by the situation
    bool operator()(const PathSituation& path)
    {
        return path.situation() == situation->situation();
    }

    // Never match a place
    bool operator()(const PlaceSituation& /*place*/)
    {
        return false;
    }
};

struct place_situation_matcher_t : public boost::static_visitor<bool>
{
    const PlaceSituation* situation;

    // Never match a path
    bool operator()(const PathSituation& /*path*/)
    {
        return false;
    }

    // Match a place if there's the same number of incoming agents
    bool operator()(const PlaceSituation& place)
    {
        return place.situation() == situation->situation();
    }
};

struct situation_printer_t : public boost::static_visitor<>
{
    std::ostream& out;
    situation_printer_t(std::ostream& out)
    : out(out)
    {
    }

    void operator()(const PathSituation& path)
    {
        out << "path " << path;
    }

    void operator()(const PlaceSituation& place)
    {
        out << "place " << place;
    }
};

// See if the agent is in the direction self is moving
bool is_agent_of_interest(const topo_agent_t& agent, const topo_agent_t& self);

/////////////////////// PathSituation definition //////////////////////

PathSituation::PathSituation(const topo_agent_t& agent,
                             const std::vector<topo_agent_t>& others,
                             int numLateralBins,
                             const hssh::LocalTopoMap& topoMap)
: areaId_(agent.areaId)
, gatewayId_(agent.gatewayId)
, situation_(numLateralBins, empty)
{
    numLateralBins = std::max(numLateralBins, 1);   // there must be at least one bin

    // Store the nearest distance for each bin to know if that value should change or not
    // when another agent is detected in that bin
    std::vector<double> distInBin(numLateralBins, HUGE_VAL);

    for(auto& ag : others)
    {
        if(!is_agent_of_interest(ag, agent))
        {
            continue;
        }

        double pathPosition = normalized_position_path(ag, topoMap);

        // If the goals aren't the same, then they are moving in opposite directions, so the position
        // of the other agent is the opposite of its internal normalized position since the left side of
        // the path segment is opposite
        if(ag.gatewayId != agent.gatewayId)
        {
            pathPosition = 1.0 - pathPosition;
        }

        int bin = std::floor(pathPosition * numLateralBins);
        bin = std::max(std::min(bin, numLateralBins - 1), 0);    // make sure we end up in the proper range

        double distToAgent = distance_between_points(agent.state.x, agent.state.y, ag.state.x, ag.state.y);
        // Ignore agents that are further than the closest agent for the bin
        if(distToAgent > distInBin[bin])
        {
            continue;
        }

        distInBin[bin] = distToAgent;

        double vel = std::sqrt(std::pow(ag.state.xVel, 2.0) + std::pow(ag.state.yVel, 2.0));
        // If heading to the same gateway, then moving in the same direction. Otherwise opposite directions
        if(vel < kMaxStationaryVel)
        {
            situation_[bin] = stationary;
        }
        else
        {
            situation_[bin] = (ag.gatewayId == agent.gatewayId) ? away : toward;
        }
    }
}


std::ostream& operator<<(std::ostream& out, const PathSituation& situation)
{
    out << situation.situation_.size() << ' ';

    for(auto& bin : situation.situation_)
    {
        switch(bin)
        {
        case PathSituation::empty:
            out << "empty ";
            break;

        case PathSituation::toward:
            out << "toward ";
            break;

        case PathSituation::away:
            out << "away ";
            break;

        case PathSituation::stationary:
            out << "stationary ";
            break;
        }
    }

    return out;
}


std::istream& operator>>(std::istream& in, PathSituation& situation)
{
    int numBins = 0;
    in >> numBins;

    situation.situation_.resize(numBins);

    std::string binName;
    for(int n = 0; n < numBins; ++n)
    {
        in >> binName;

        if(binName == "empty")
        {
            situation.situation_[n] = PathSituation::empty;
        }
        else if(binName == "toward")
        {
            situation.situation_[n] = PathSituation::toward;
        }
        else if(binName == "away")
        {
            situation.situation_[n] = PathSituation::away;
        }
        else if(binName == "stationary")
        {
            situation.situation_[n] = PathSituation::stationary;
        }
        else
        {
            std::cerr << "ERROR: PathSituation: Unknown bin state: " << binName << '\n';
            assert(false);
        }
    }

    return in;
}

/////////////////////// PlaceSituation definition //////////////////////

PlaceSituation::PlaceSituation(const topo_agent_t& agent,
                               const std::vector<topo_agent_t>& others,
                               const hssh::LocalTopoMap& topoMap)
: areaId_(agent.areaId)
, gatewayId_(agent.gatewayId)
, agentCount_(0)
{
    // Count agents with the same gateway goal, but a different current area
    for(auto& ag : others)
    {
        if((ag.gatewayId == agent.gatewayId) && (ag.areaId != agent.areaId))
        {
            ++agentCount_;
        }
    }
}


std::ostream& operator<<(std::ostream& out, const PlaceSituation& situation)
{
    out << situation.situation();
    return out;
}


std::istream& operator>>(std::istream& in, PlaceSituation& situation)
{
    in >> situation.agentCount_;
    return in;
}

/////////////////////// TopoSituationResponse definition //////////////////////

TopoSituationResponse::TopoSituationResponse(const std::vector<double>& distribution, SituationType type)
: dist_(distribution)
{
    assert(!dist_.empty());

    binWidth_ = 1.0 / dist_.size();
    normalizeDistribution();

    switch(type)
    {
    case place:
        situation_ = PlaceSituation();
        break;
    case path:
        situation_ = PathSituation();
        break;
    default:
        std::cerr << "Unknown response type " << type << " Using default.\n";
    }
}


TopoSituationResponse::TopoSituationResponse(const PathSituation& situation, const std::vector<double>& distribution)
: TopoSituationResponse(distribution, path)
{
    situation_ = situation;
}


TopoSituationResponse::TopoSituationResponse(const PlaceSituation& situation, const std::vector<double>& distribution)
: TopoSituationResponse(distribution, place)
{
    situation_ = situation;
}


bool TopoSituationResponse::isResponseForSituation(const PathSituation& situation) const
{
    path_situation_matcher_t matcher;
    matcher.situation = &situation;

    return situation_.apply_visitor(matcher);
}


bool TopoSituationResponse::isResponseForSituation(const PlaceSituation& situation) const
{
    place_situation_matcher_t matcher;
    matcher.situation = &situation;

    return situation_.apply_visitor(matcher);
}


double TopoSituationResponse::distanceCost(double distance) const
{
    // Cost of being in a particular spot is probability that something was not in that location when observed.
    // The stored distribution is the probability of being in a location at a given time, so the inverse dist gives
    // the desired cost value. The inverse dist is cached for efficiency.
    if(distance < 0.0)
    {
        return invDist_.front();
    }
    else if(distance > 1.0)
    {
        return invDist_.back();
    }

    std::size_t bin = distance * dist_.size();

    if(bin < dist_.size())
    {
        return invDist_[bin];
    }

    // Might hit the upper edge of the last bin, then just return the final one
    return invDist_.back();
}


void TopoSituationResponse::addExample(double normDistance)
{
    if((normDistance < 0.0) || (normDistance > 1.0))
    {
        std::cerr << "WARN: TopoSituationResponse: Example was not a normalized distance: " << normDistance << '\n';
        return;
    }

    std::size_t bin = std::lrint(normDistance * dist_.size());

    if(bin < dist_.size())
    {
        dist_[bin] += 1.0;
    }
    else
    {
        dist_.back() += 1.0;
    }

    examples_.push_back(normDistance);
}


void TopoSituationResponse::normalizeDistribution(void)
{
    double totalExamples = std::accumulate(dist_.begin(), dist_.end(), 0.0);

    if(totalExamples > 0)
    {
        std::transform(dist_.begin(), dist_.end(), dist_.begin(), [totalExamples](double val) {
            return val / totalExamples;
        });

        invDist_.resize(dist_.size());
        std::transform(dist_.begin(), dist_.end(), invDist_.begin(), [](double prob) {
            return prob > 0.0 ? 1.0 / prob : 1e8;
        });
    }
}


std::ostream& operator<<(std::ostream& out, const TopoSituationResponse& response)
{
    situation_printer_t printer(out);
    response.situation_.apply_visitor(printer);
    out << " -> ";
    std::copy(response.dist_.begin(), response.dist_.end(), std::ostream_iterator<double>(out, " "));
    return out;
}


void TopoSituationResponse::saveExamples(std::ostream& out) const
{
    situation_printer_t printer(out);
    situation_.apply_visitor(printer);
    out << ' ' << examples_.size() << ' ';
    std::copy(examples_.begin(), examples_.end(), std::ostream_iterator<double>(out, " "));
    out << '\n';
}


bool TopoSituationResponse::loadExamples(std::istream& in)
{
    bool success = false;
    std::string type;
    in >> type;

    if(type == "path")
    {
        PathSituation situation;
        in >> situation;
        situation_ = situation;
        success = true;
    }
    else if(type == "place")
    {
        PlaceSituation situation;
        in >> situation;
        situation_ = situation;
        success = true;
    }
    else
    {
        std::cerr << "ERROR: TopoSituationResponse: Unknown situation type: " << type << '\n';
        success = false;
    }

    // If good so far, then load the examples
    if(success)
    {
        int numExamples = 0;
        in >> numExamples;
        examples_.resize(numExamples);

        std::copy_n(std::istream_iterator<double>(in), numExamples, examples_.begin());
    }

    return success;
}

/////////////////////// Helper functions //////////////////////

bool is_agent_of_interest(const topo_agent_t& other, const topo_agent_t& self)
{
    // Don't care about agents on other paths
    if(other.areaId != self.areaId)
    {
        return false;
    }

    double heading = std::atan2(self.state.yVel, self.state.xVel);
    double otherHeading = angle_to_point(Point<float>(self.state.x, self.state.y),
                                               Point<float>(other.state.x, other.state.y));

    // The other agent must be in front, which means the difference in heading angles is less than pi/2
    return angle_diff_abs(otherHeading, heading) < 2.0 * M_PI / 3.0;
}

} // namespace mpepc
} // namespace vulcan

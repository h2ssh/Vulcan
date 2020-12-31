/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     topo_situation.h
 * \author   Collin Johnson
 *
 * Declaration of PathSituation, PlaceSituation, and TopoResponseSituation.
 */

#ifndef MPEPC_COSTS_TOPO_SITUATION_H
#define MPEPC_COSTS_TOPO_SITUATION_H

#include "mpepc/social/topo_agent.h"
#include <cereal/access.hpp>
#include <cereal/types/boost_variant.hpp>
#include <cereal/types/vector.hpp>
#include <iosfwd>

namespace vulcan
{
namespace hssh
{
class LocalTopoMap;
}
namespace mpepc
{

/**
 * PathSituation describes the symbolic situation for an agent traveling along a path segment. The situation
 * for a path segment divides the path segment laterally into N bins. Each bin has one of three states:
 *
 *   - empty      : no one is occupying that bin in the direction of motion along the path
 *   - toward     : someone is coming toward the agent in the bin
 *   - away       : someone is moving away from the agent
 *   - stationary : someone is standing still (vel < 0.2m/s)
 *
 * When assigning a direction to the bin, we consider only the closest other agent that occupies that bin.
 *
 * The situation considers only other agents that occupy the same path segment as the agent. It doesn't consider
 * agents that might end up moving into this area.
 */
class PathSituation
{
public:
    enum BinState
    {
        empty,        ///< No one occupies the bin of interest
        toward,       ///< Someone is moving towards the agent
        away,         ///< Someone is moving away from the agent
        stationary,   ///< Someone is standing still (vel < 0.2m/s)
    };

    PathSituation(void) = default;

    /**
     * Constructor for PathSituation.
     *
     * \param    agent               State of the agent in the situation
     * \param    others              Other agents in the environment
     * \param    numLateralBins      Number of lateral bins to create across the path segment
     * \param    topoMap             Topological map of the environment
     * \pre numLateralBins > 0
     */
    PathSituation(const topo_agent_t& agent,
                  const std::vector<topo_agent_t>& others,
                  int numLateralBins,
                  const hssh::LocalTopoMap& topoMap);

    /**
     * situation retrieves the symbolic representation of the situation as described above.
     */
    std::vector<BinState> situation(void) const { return situation_; }

    /**
     * areaId retrieves the id of the path segment for which this situation was estimated.
     */
    int areaId(void) const { return areaId_; }

    /**
     * Retrieve the goal gateway for the agent in the situation.
     */
    int gatewayId(void) const { return gatewayId_; }

    // Output operator:  bin_states
    friend std::ostream& operator<<(std::ostream& out, const PathSituation& situation);

    // Input operator: load the bin states -- applies to any area
    friend std::istream& operator>>(std::istream& in, PathSituation& situation);

private:
    int areaId_;                        ///< Id of the path associated with the situation
    int gatewayId_;                     ///< Id of goal gateway
    std::vector<BinState> situation_;   ///< Estimate state of the situation

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar(areaId_, gatewayId_, situation_);
    }
};

/**
 * PlaceSituation is a symbolic representation for navigation through a place. The situation representation
 * considers the number of agents that are transitioning through the gateway in the opposite direction.
 *
 * If entering a place, it's the number of people exiting through the gateway. If exiting a place, it's the
 * number of people entering through the gateway.
 *
 * Because entering and exiting an area both have a situation, the area associated with a PlaceSituation is
 * not necessarily a place if the situation occurs when exiting a path segment and entering a place.
 */
class PlaceSituation
{
public:
    PlaceSituation(void) = default;

    /**
     * Constructor for PlaceSituation.
     *
     * \param    agent           State of the agent in the situation
     * \param    others          Other agents in the environment
     * \param    topoMap         Topological map of the environment
     */
    PlaceSituation(const topo_agent_t& agent,
                   const std::vector<topo_agent_t>& others,
                   const hssh::LocalTopoMap& topoMap);

    /**
     * situation retrieves the symbolic information on the particular transition situation as described above.
     */
    int situation(void) const { return agentCount_; }

    /**
     * areaId retrieves the id of the area associated with the situation.
     */
    int areaId(void) const { return areaId_; }

    /**
     * gatewayId retrieves the id of the gateway through which the agent is transitioning.
     */
    int gatewayId(void) const { return gatewayId_; }

    // Output operator:  incoming count
    friend std::ostream& operator<<(std::ostream& out, const PlaceSituation& situation);

    // Input operator: read incoming count
    friend std::istream& operator>>(std::istream& in, PlaceSituation& situation);

private:
    int areaId_;       ///< Id of the place associated with the situation
    int gatewayId_;    ///< Id of the gateway for the situation
    int agentCount_;   ///< Count of the number of agents transitioning through the gateway

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar(areaId_, gatewayId_, agentCount_);
    }
};

/**
 * TopoSituationResponse encodes the learned response to a particular topological situation that occurs. The encoded
 * response is a discrete probability distribution across the normalized distance within the topological area. The
 * distance is evenly divided into bins. Each bin holds some probability of the agent being there. By using a
 * normalized distance, behaviors can be easily transferred from environment to environment.
 */
class TopoSituationResponse
{
public:
    enum SituationType
    {
        path,
        place,
    };

    TopoSituationResponse(void) = default;

    /**
     * Constructor TopoSituationResponse.
     *
     * Create a generic response that can apply to anything.
     *
     * When learning, distribution is a prior on the learned model.
     *
     * \param    distribution        Distribution for the response
     * \param    type                Type of situation for the response (path or place)
     */
    TopoSituationResponse(const std::vector<double>& distribution, SituationType type);

    /**
     * Constructor for TopoSituationResponse.
     *
     * Create a response for a PathSituation.
     *
     * When learning, distribution is a prior on the learned model.
     *
     * \param    situation           Path situation that occurred
     * \param    distribution        Discretized probability distribution representing the encoded response
     */
    TopoSituationResponse(const PathSituation& situation, const std::vector<double>& distribution);

    /**
     * Constructor for TopoSituationResponse.
     *
     * Create a response for a PlaceSituation.
     *
     * When learning, distribution is a prior on the learned model.
     *
     * \param    situation           Place situation that occurred
     * \param    distribution        Discretized probability distribution representing the encoded response
     */
    TopoSituationResponse(const PlaceSituation& situation, const std::vector<double>& distribution);

    // Observers

    /**
     * isResponseForSituation checks if this response applies to the identified situation.
     *
     * \param    situation           PathSituation the agent is navigating
     * \return   True if the situation is the same as this learned response.
     */
    bool isResponseForSituation(const PathSituation& situation) const;
    bool isResponseForSituation(const PlaceSituation& situation) const;

    /**
     * distribution retrieves the learned distribution the encodes the learned response for the particular situation.
     *
     * \return   Discretized probability distribution for the learned norm. Uniform width bins from [0, 1].
     */
    const std::vector<double>& distribution(void) const { return dist_; }

    /**
     * distanceCost retrieves the cost of being a particular normalized distance from the wall of interest.
     *
     * The distance is clamped to the range [0, 1], so values too high or too low snap to the nearest bin.
     *
     * \param    distance            Normalized distance from the wall
     * \return   Learned cost of being that distance.
     */
    double distanceCost(double distance) const;

    // Mutators -- for learning a response

    /**
     * addExample adds a new example of the response to this particular situation. During learning, the distribution
     * will not be normalized until requested.
     *
     * \param    normDistance        Observed normalized distance of an agent in this situation
     */
    void addExample(double normDistance);

    /**
     * normalizeDistribution normalizes the learned distribution to be a true probability distribution across
     * the distance bins.
     */
    void normalizeDistribution(void);

    // Output operator::   situation  ->  bins
    friend std::ostream& operator<<(std::ostream& out, const TopoSituationResponse& response);

    // I/O operations for results evaluation dealing with all examples made of particular responses

    /**
     * Save all stored examples for this TopoSituation added via addExample to the provided stream. The format
     * is:
     *
     *   situation_description N_ex ex0 ex1 . . . exN
     *
     * \param    out         Stream in which to save the examples
     */
    void saveExamples(std::ostream& out) const;

    /**
     * Load all stored examples and the situation from the provided stream. The situation will be changed to the
     * situation described in the stream.
     *
     * \param    in          Input stream with the examples
     * \return   True if the examples and situation were loaded successfully.
     */
    bool loadExamples(std::istream& in);

    /**
     * Retrieve all examples for this particular situation.
     */
    std::vector<double> examples(void) const { return examples_; }

private:
    boost::variant<PathSituation, PlaceSituation> situation_;   ///< Situation this response applies to
    std::vector<double> dist_;                                  ///< Learned distribution for the situation
    std::vector<double> invDist_;                               ///< Distribution of the inverse distribution
    double binWidth_;                                           ///< Width of each normalized dist bin

    std::vector<double> examples_;   ///< All examples created during the learning phase for the situation

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar(situation_, dist_, binWidth_);

        normalizeDistribution();
    }
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_COSTS_TOPO_SITUATION_H

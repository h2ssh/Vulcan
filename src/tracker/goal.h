/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal.h
* \author   Collin Johnson
*
* Definition of ObjectDestination, ObjectGoal, and ObjectGoalDistribution.
*/

#ifndef TRACKER_GOAL_H
#define TRACKER_GOAL_H

#include <core/angle_functions.h>
#include <core/line.h>
#include <cereal/access.hpp>
#include <cereal/types/boost_variant.hpp>
#include <numeric>
#include <vector>
#include <cassert>

namespace vulcan
{
namespace tracker
{

/**
* ObjectDestination defines the possible types of destinations that an object wishes to reach. Currently, they are
* either a line boundary or a point position.
*/
using ObjectDestination = boost::variant<Line<double>, Point<double>>;


/**
* ObjectGoal describes a possible goal for an object in the environment. The goal is defined as some boundary to be
* crossed along with the desired heading of the object when crossing that boundary.
*/
class ObjectGoal
{
public:

    /**
    * Default constructor for ObjectGoal.
    */
    ObjectGoal(void) = default;

    /**
    * Constructor for ObjectGoal.
    *
    * Create a goal with an assigned destination.
    *
    * \param    destination         Destination for the goal
    * \param    heading             Heading for the goal
    * \param    logProbability      Log-probability of this particular goal
    * \pre  logProbability <= 0.0
    */
    ObjectGoal(const ObjectDestination& destination, double heading, double logProbability)
    : destination_(destination)
    , heading_(wrap_to_pi(heading))
    , logProbability_(logProbability)
    {
        assert(logProbability_ <= 1e-6);
    }

    /**
    * destination retrieves the destination of the object.
    */
    ObjectDestination destination(void) const { return destination_; }

    /**
    * heading retrieves the heading for the object when reaching the goal.
    */
    double heading(void) const { return heading_; }

    /**
    * probability retrieves the probability of this goal being the one for the object.
    */
    double probability(void) const { return std::exp(logProbability_); }

    /**
    * logProbability retrieves the log-probability of this goal being the one for the object.
    */
    double logProbability(void) const { return logProbability_; }

private:

    ObjectDestination destination_;     ///< Boundary defining the goal
    double heading_ = 0.0;              ///< Heading in which the object will cross the boundary
    double logProbability_ = -INFINITY; ///< Probability of this particular goal being reached by the object

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( destination_,
            heading_,
            logProbability_
        );
    }
};

/////   Operators for ObjectGoal   /////
inline bool operator<(const ObjectGoal& lhs, const ObjectGoal& rhs)
{
    return lhs.logProbability() < rhs.logProbability();
}

inline bool operator>(const ObjectGoal& lhs, const ObjectGoal& rhs)
{
    return lhs.logProbability() > rhs.logProbability();
}

inline bool operator==(const ObjectGoal& lhs, const ObjectGoal& rhs)
{
    return lhs.destination() == rhs.destination();
}

/**
* ObjectGoalDistribution maintains the complete distribution across all possible goals in the area.
*/
class ObjectGoalDistribution
{
public:

    using const_iterator = std::vector<ObjectGoal>::const_iterator;

    ObjectGoalDistribution(void) = default;

    /**
    * Constructor for ObjectGoalDistribution.
    *
    * Create a normalized distribution for the provided goals.
    *
    * \param    goals       Goals to construct the distribution from
    */
    ObjectGoalDistribution(const std::vector<ObjectGoal>& goals)
    {
        if(!goals.empty())
        {
            // Use the shift by max, then exponentiate trick for normalizing the distribution
            double maxLogProb = std::max_element(goals.begin(), goals.end())->logProbability();
            double normalizer = std::accumulate(goals.begin(), goals.end(), 0.0, [&](double sum, const ObjectGoal& g) {
                return sum + std::exp(g.logProbability() - maxLogProb);
            });

            for(auto& g : goals)
            {
                // If all goals are equally unlikely, then assume a uniform distribution across all goals
                double posterior = (normalizer > 0.0)
                    ? std::exp(g.logProbability() - maxLogProb) / normalizer : 1.0 / goals.size();
                goals_.emplace_back(g.destination(), g.heading(), std::log(posterior));
            }
        }
    }

    ObjectGoalDistribution(std::initializer_list<ObjectGoal> goals)
    : ObjectGoalDistribution(std::vector<ObjectGoal>(goals))
    {
    }

    /**
    * bestGoal retrieves the best goal amongst the distribution. If the distribution is empty, then the default
    * null goal is returned.
    */
    ObjectGoal bestGoal(void) const
    {
        auto maxIt = std::max_element(goals_.begin(), goals_.end());
        return (maxIt != goals_.end()) ? *maxIt : ObjectGoal{};
    }

    // Iteration support
    std::size_t size(void) const { return goals_.size(); }
    bool empty(void) const { return goals_.empty(); }
    const_iterator begin(void) const { return goals_.begin(); }
    const_iterator end(void) const { return goals_.end(); }
    ObjectGoal operator[](int index) const { return goals_[index]; }

private:

    std::vector<ObjectGoal> goals_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(goals_);
    }
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_GOAL_H

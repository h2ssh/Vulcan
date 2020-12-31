/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     univariate_distribution.h
 * \author   Collin Johnson
 *
 * Definition of UnivariateDistribution interface and create_univariate_distribution factory.
 */

#ifndef MATH_UNIVARIATE_DISTRIBUTION_H
#define MATH_UNIVARIATE_DISTRIBUTION_H

#include <iosfwd>
#include <memory>
#include <string>

namespace vulcan
{
namespace math
{

class UnivariateDistribution;

/**
 * create_univariate_distribution creates a new UnivariateDistribution of the specified type.
 *
 * The type string specifies a concrete type for the distribution. These type strings can be found
 * in header files for implementations of the UnivariateDistribution interface.
 *
 * \param    type            Type of distribution to be created.
 */
std::unique_ptr<UnivariateDistribution> create_univariate_distribution(const std::string& type);


/**
 * UnivariateDistribution is an interface describing a simple univariate probability distribution. A valid distribution
 * supports both sampling a value from the distribution and calculating the likelihood of a value using the PDF for
 * the distribution.
 *
 * Additionally, a distribution needs to support saving itself to a file so it can be created at a later time. This is
 * accomplished via the save method. The loadParams method loads the parameters at a later time.
 */
class UnivariateDistribution
{
public:
    virtual ~UnivariateDistribution(void) { }

    /**
     * name retrieves the name of the particular distribution.
     */
    virtual std::string name(void) const = 0;

    /**
     * sample draws a sample from the distribution.
     *
     * \return   A sample drawn from the probability distribution.
     */
    virtual double sample(void) const = 0;

    /**
     * likelihood calculates the likelihood of a value in the distribution.
     *
     * \param    value           Value for which to calculate P(x)
     * \return   P(value).
     */
    virtual double likelihood(double value) const = 0;

    /**
     * save saves the distribution to a stream. The format for the stream is:
     *
     *   param1 param2 ... paramN
     *
     * \param    out         Stream to write to
     * \return   True if the distribution was saved successfully.
     */
    virtual bool save(std::ostream& out) const = 0;

    /**
     * loadParams loads the parameters for the distribution from an input stream.
     * The format for the stream will be:
     *
     *   param1 param2 ... paramN
     *
     * which is the same format as save.
     *
     * \param    in          Input stream to load from
     * \return   True if the parameters were loaded successfully.
     */
    virtual bool load(std::istream& in) = 0;
};

}   // namespace math
}   // namespace vulcan

#endif   // MATH_UNIVARIATE_DISTRIBUTION_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     smoothed_derivative_sequence.h
* \author   Collin Johnson
* 
* Definition of SmoothedDerivativeSequence.
*/

#ifndef MATH_SMOOTHED_DERIVATIVE_SEQUENCE_H
#define MATH_SMOOTHED_DERIVATIVE_SEQUENCE_H

#include <math/statistics.h>
#include <boost/optional.hpp>
#include <deque>
#include <iomanip>
#include <iostream>
#include <type_traits>
#include <cassert>

namespace vulcan
{
namespace math
{
    
/**
* ExtremumType defines the possible types of extrema, which are minimum and maximum.
*/
enum class ExtremumType
{
    minimum,
    maximum
};

/**
* sequence_extremum_t
*/
template <typename T>
struct sequence_extremum_t
{
    ExtremumType type;
    int          begin;
    int          index;
    int          end;
    T            value;
};

/**
* SmoothedDerivativeSequence is a sequence of noisy values in which the extrema (maxima and minima) are to be found.
* 
* T must be an arithmetic type.
*/
template <typename T>
class SmoothedDerivativeSequence
{
public:
    
    static_assert(std::is_arithmetic<T>::value, "T in SmoothedDerivativeSequence must be arithmetic");
    
    using ConstIter  = typename std::deque<T>::const_iterator;
    using value_type = T;
    
    /**
    * Constructor for SmoothedDerivativeSequence.
    *
    * \pre smoothingRadius >= 0
    * \pre extremumWidth > 0
    *
    * \param    smoothingRadius         Radius of the rectangular smoothing filter to apply (optional, default = 3)
    * \param    extremumWidth           Minimum width of an extremum (optional, default = 3)
    */
    explicit SmoothedDerivativeSequence(int smoothingRadius = 3, int extremumWidth = 3)
    : beginIndexOffset_(0)
    , kFilterRadius_(smoothingRadius)
    , kExtremumWidth_(extremumWidth)
    {
        assert(kFilterRadius_ >= 0);
        assert(kExtremumWidth_ > 0);
    };
    
    /**
    * push_back adds a new value to the end of the sequence.
    */
    void push_back(T value);

    /**
    * erase erases all values before the specified index. erase should be used to erase values from the sequence
    * that have already been processed. An easy way is to pass erase the index in a sequence_extremum_t that results
    * from a call to extremumAfter.
    *
    * Values are erased in the interval [begin, begin+index).
    *
    * \param    index       Index for which to erase values
    */
    void erase(int index);
    
    /**
    * hasEnoughDataForExtremum checks if enough values have been added to the sequence for a reasonable extremum to
    * exist. The amount of data needed is related to the smoothing radius of the sequence.
    * 
    * If there isn't enough data, then calls to xxxmumAfter will always return boost::none, rather than maybe return
    * boost::none.
    */
    bool hasEnoughDataForExtremum(void) const { return static_cast<int>(values_.size()) > kExtremumWidth_ + kFilterRadius_; }
    
    /**
    * extremumAfter finds the next extrema in the sequence if one exists.
    * 
    * \param    begin           Index from which the extrema search should start
    * \return   A sequence_extrema_t if an extrema exists in the range [begin, end)
    */
    boost::optional<sequence_extremum_t<T>> extremumAfter(int begin) const;
    
    /**
    * minimumAfter finds the next minimum in the sequence if one exists.
    * 
    * \param    begin           Index from which the minimum search should start
    * \return   A sequence_extremum_t corresponding to the first minimum in the range [begin, end).
    */
    boost::optional<sequence_extremum_t<T>> minimumAfter(int begin) const;
    
    /**
    * maximumAfter finds the next maximum in the sequence if one exists.
    * 
    * \param    begin           Index from which the maximum search should start
    * \return   A sequence_extremum_t corresponding to the first maximum in the range [begin, end).
    */
    boost::optional<sequence_extremum_t<T>> maximumAfter(int begin) const;
    
    // Iteration support for the values
    bool        empty(void) const { return values_.empty(); }
    std::size_t size(void) const { return values_.size() - beginIndexOffset_; }
    ConstIter   begin(void) const { return values_.cbegin() + beginIndexOffset_; }
    ConstIter   end(void) const { return values_.cend(); }
    const T&    operator[](int index) const { return values_[index + beginIndexOffset_]; }
    const T&    front(void) const { return values_[beginIndexOffset_]; }
    const T&    back(void) const { return values_.back(); }

private:

    std::deque<T> values_;
    std::deque<T> derivs_;
    std::deque<T> firstSmoothedDerivs_;
    std::deque<T> secondSmoothedDerivs_;

    int beginIndexOffset_;
    
    const int kFilterRadius_;
    const int kExtremumWidth_;
    
    void calculateDerivatives(void);
    void smoothDerivatives(const std::deque<T>& in, std::deque<T>& out);
    boost::optional<sequence_extremum_t<T>> extremumOfTypeAfter(int begin, ExtremumType type) const;
};


template <typename T>
void SmoothedDerivativeSequence<T>::push_back(T value)
{
    values_.push_back(value);
    calculateDerivatives();
}


template <typename T>
void SmoothedDerivativeSequence<T>::erase(int index)
{
    if(index <= 0)
    {
        return;
    }
    
    const int kMinPreviousData = kFilterRadius_ * 10;

    // Need to keep around [index - radius*2, end) values in order to ensure that accurate derivatives are always stored
    // Otherwise erasing will cause the derivatives of the sequence to change.
    // Two smoothing passes mean derivatives are needed up to twice the radius
    index += beginIndexOffset_;
    int eraseIndex = std::max(0, index - kMinPreviousData);

    // If the index is less than the full size, erase part of the data
    if(eraseIndex < static_cast<int>(values_.size()))
    {
        values_.erase(values_.begin(), values_.begin() + eraseIndex);
        derivs_.erase(derivs_.begin(), derivs_.begin() + eraseIndex);
    }
    // Otherwise clear everything out
    else
    {
        values_.clear();
        derivs_.clear();
    }

    // Erasing effectively sets values_.front() == values_[index]. If there's not actually filter radius values in
    // values_, then nothing is erased but the offset is moved. Once enough data is in values_, then the offset always
    // remains equal to the filter offset
    beginIndexOffset_ = std::min(kMinPreviousData, index);
}


template <typename T>
boost::optional<sequence_extremum_t<T>> SmoothedDerivativeSequence<T>::extremumAfter(int begin) const
{
    if(!hasEnoughDataForExtremum() || (begin >= static_cast<int>(values_.size())))
    {
        return boost::none;
    }
    
    assert(static_cast<int>(values_.size()) > kFilterRadius_);

    // Derivatives start at the first index
    begin = std::max(beginIndexOffset_, begin + beginIndexOffset_);

//     std::cout << "Value:    Deriv:\n";
//     for(std::size_t n = 0; n < values_.size(); ++n)
//     {
//         std::cout << std::setprecision(4) << std::setw(9) << values_[n] << ' ' << secondSmoothedDerivs_[n] << '\n';
//     }
    
    bool initialPositive = secondSmoothedDerivs_[begin] > 0.0;
    int signFlipIndex    = begin;

//     for(int n = begin, end = static_cast<int>(secondSmoothedDerivs_.size())-kFilterRadius_; n < end; ++n)
        for(int n = begin, end = static_cast<int>(secondSmoothedDerivs_.size()); n < end; ++n)
    {
        // If the sign has flipped from pos/neg or neg/pos, then store that index
        if(secondSmoothedDerivs_[n] * secondSmoothedDerivs_[n+1] < 0.0)
        {
            signFlipIndex = n;
        }
        // If going in the opposite direction from the start of the search and the sign has flipped and are beyond the
        // extremum width, then
        else if(((initialPositive && secondSmoothedDerivs_[n] < 0.0) ||
            (!initialPositive && secondSmoothedDerivs_[n] > 0.0)) &&
            (n - signFlipIndex >= kExtremumWidth_))
        {
            sequence_extremum_t<T> extremum;
            extremum.type  = initialPositive ? ExtremumType::maximum : ExtremumType::minimum;
            extremum.begin = begin;
            extremum.index = std::distance(values_.begin(), 
                                           std::max_element(values_.begin()+begin, values_.begin() + n)) 
                - beginIndexOffset_; //; signFlipIndex - beginIndexOffset_; // the index offset remains hidden to callers
            extremum.end   = n;
            extremum.value = values_[extremum.index + beginIndexOffset_];
            
            return extremum;
        }
    }

    // Nothing was found
    return boost::none;
}


template <typename T>
boost::optional<sequence_extremum_t<T>> SmoothedDerivativeSequence<T>::minimumAfter(int begin) const
{
    return extremumOfTypeAfter(begin, ExtremumType::minimum);
}


template <typename T>
boost::optional<sequence_extremum_t<T>> SmoothedDerivativeSequence<T>::maximumAfter(int begin) const
{
    return extremumOfTypeAfter(begin, ExtremumType::maximum);
}


template <typename T>
void SmoothedDerivativeSequence<T>::calculateDerivatives(void)
{
    // Only calculate if the dervis and values aren't the same size, otherwise known that the derivatives have already
    // been calculated
    assert(!values_.empty());
    if(values_.size() == 1)
    {
        derivs_.push_back(0.0);
        return;
    }
    
    // Add the new derivative
    derivs_.push_back(values_.back() - values_[values_.size()-2]);

    assert(derivs_.size() == values_.size());

    // Perform two smoothing passes on the derivatives.
    smoothDerivatives(derivs_, firstSmoothedDerivs_);
    smoothDerivatives(firstSmoothedDerivs_, secondSmoothedDerivs_);
//     secondSmoothedDerivs_ = firstSmoothedDerivs_;
}


template <typename T>
void SmoothedDerivativeSequence<T>::smoothDerivatives(const std::deque<T>& in, std::deque<T>& out)
{
    // Use intermediate storage for the smoothed derivatives, then copy them back into the derivatives
    out.resize(in.size());
    
    for(int n = 0, end = static_cast<int>(in.size()); n < end; ++n)
    {
        int startIndex = std::max(0, n - kFilterRadius_);
//         int endIndex   = std::min(end, n + kFilterRadius_ + 1);
        int endIndex = n + 1;

        out[n] = math::mean(in.begin() + startIndex, in.begin() + endIndex);
    }
}


template <typename T>
boost::optional<sequence_extremum_t<T>> SmoothedDerivativeSequence<T>::extremumOfTypeAfter(int begin,
                                                                                           ExtremumType type) const
{
    int extremumSearchStart = begin;
    
    while(true)
    {
        auto extremum = extremumAfter(extremumSearchStart);
        
        // If no extremum found, then jump out of the loop
        if(!extremum)
        {
            break;
        }
        // If an extremum is found of the correct type, return it
        else if(extremum->type == type)
        {
            return extremum;
        }
        // Otherwise, start the search from where the last extremum was found
        else
        {
            extremumSearchStart = extremum->index + 1;
        }
    }
    
    // No suitable extremum was found
    return boost::none;
}


} // namespace math
} // namespace vulcan

#endif // MATH_SMOOTHED_DERIVATIVE_SEQUENCE_H

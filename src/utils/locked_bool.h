/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     locked_bool.h
* \author   Collin Johnson
*
* Definition of LockedBool.
*/

#ifndef UTILS_LOCKED_BOOL_H
#define UTILS_LOCKED_BOOL_H

#include <atomic>

namespace vulcan
{
namespace utils
{

/**
* LockedBool provides a thread-safe implementation of the bool primitive-type.
*
* A LockedBool is useful for applications where two threads share a variable that
* determines if a condition continues to run, i.e. a loop condition, while(var)
* for example. In this case, the value of the LockedBool can be used without
* concern for any sort of race condition and without having to explicitly lock
* the value before using it.
*
* A LockedBool can be used as a normal bool can, with the exception of getting
* the value. The value of the LockedBool -- either true or false -- can be received
* using getValue() accessor.
*
* The LockedBool has been updated to C++11 and is now simply a wrapper around std::atomic<bool>.
*/
class LockedBool
{
public:

    /**
    * Constructor for LockedBool.
    *
    * \param    value           Initial value to assign (false by default)
    */
    LockedBool(bool value = false)
        : value(value)
    {
    }

    /**
    * Copy constructor for LockedBool.
    *
    * \param    lb              LockedBool to be copied
    */
    LockedBool(const LockedBool& lb)
        : value(lb.value.load())
    {
    }

    // Nothing crazy needs to happen in the destructor, nothing is allocated, so the default should be fine

    /**
    * getValue atomically retrieves the value of the LockedBool.
    *
    * \return   Value of the LockedBool.
    */
    bool getValue() const
    {
        return value.load();
    }


    /**
    * setValue atomically sets the value of the LockedBool
    *
    * \param    value               New value for the LockedBool
    */
    void setValue(bool value)
    {
        this->value = value;
    }

    /**
    * operator== compares the LockedBool to a bool.
    *
    * \param    comp        Value to compare to
    * \return   True if the values are the same, that is, this->getValue() == comp
    */
    bool operator==(bool comp) const { return (getValue() == comp); }

    /**
    * operator== compares the LockedBool to another LockedBool.
    *
    * \param    comp         LockedBool to compare to
    * \return   True if (comp.getValue() == this->getValue())
    */
    bool operator==(const LockedBool& comp) const { return (getValue() == comp.getValue()); }

    /**
    * operator= assigns the value the of the LockedBool to be that of newVal.
    *
    * \param    newVal         New value to assign to the LockedBool
    * \return   LockedBool with getValue() == newVal
    */
    LockedBool& operator=(bool newVal) { setValue(newVal); return *this; }

    /**
    * operator= assigns the value of the LockedBool to be that of newVal.getValue().
    *
    * \param    newVal          New value to assign the LockedBool
    * \return   LockedBool with getValue() == newVal.getValue
    */
    LockedBool& operator=(const LockedBool& newVal) { setValue(newVal.getValue()); return *this; }

    /**
    * operator bool() is a conversion operator to allow a LockedBool to be treated the same way as a normal bool
    * in conditional operations.
    */
    operator bool() const { return value.load(); }

private:
    std::atomic<bool> value;
};

}
}

#endif // UTILS_LOCKED_BOOL_H

/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UTILS_OBJECT_POOL_H
#define UTILS_OBJECT_POOL_H

#include <vector>

namespace vulcan
{
namespace utils
{

/**
 * ObjectPool maintains a pool of dynamically allocated objects. The objects are intended to be
 * PODs that will get reused frequently during normal operation of a program. The ObjectPool
 * maintains the memory for the objects, so there is no fear of a leak, though crashes due to
 * double-deleting are certainly a problem.
 *
 * To use the object pool, only one method is needed, newObject(), which returns a pointer to
 * an object of the desired type.
 *
 * Currently, constructors are not supported, so a default constructor needs to exist for any
 * objects managed by an object pool.
 *
 * If the pool needs to be reset because no previous objects will be used again, call reset().
 * If the pool needs to be emptied, do so with the deleteObjects() method.
 */
template <typename T>
class ObjectPool
{
public:
    ObjectPool(void) : objectsUsed(0) { }

    ~ObjectPool(void) { deleteObjects(); }

    /**
     * newObject retrieves a new object from the pool.
     */
    T* newObject(void)
    {
        T* object = 0;

        // Only allocate a new object when there isn't one available already
        if (objectsUsed < pool.size()) {
            object = pool[objectsUsed];
        } else {
            object = new T();
            pool.push_back(object);
        }

        ++objectsUsed;

        return object;
    }

    /**
     * reset resets the object counter in the pool to 0 so newObject() will begin
     * returning previously created objects before creating new objects.
     */
    void reset(void) { objectsUsed = 0; }

    /**
     * deleteObjects erases all currently maintained objects and free the
     * associated memory.
     */
    void deleteObjects(void)
    {
        for (int n = pool.size(); --n >= 0;) {
            delete pool[n];
        }

        pool.clear();
        objectsUsed = 0;
    }

private:
    // No copying or assignment supported!
    ObjectPool(const ObjectPool<T>& op) { }

    void operator=(const ObjectPool<T>& op) { }

    std::vector<T*> pool;
    unsigned int objectsUsed;
};

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_OBJECT_POOL_H

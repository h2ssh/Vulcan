/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     binary_heap.h
 * \author   Collin Johnson
 *
 * Definition of a simple BinaryHeap data structure.
 */

#ifndef UTILS_BINARY_HEAP_H
#define UTILS_BINARY_HEAP_H

#include <cassert>
#include <cstddef>
#include <vector>

namespace vulcan
{
namespace utils
{

/**
 * BinaryHeap is an implementation of a standard binary heap data structure. The binary heap serves
 * as an efficient backend for a priority queue.
 *
 * A BinaryHeap is a data structure where each node has a parent and a left and right child. Each node
 * satisfies the following constraint:
 *
 *       priority(A) >= priority(left(A)) && priority(right(A))  (for a max heap)
 *       priority(A) <= priority(left(A)) && priority(right(A))  (for a min heap)
 *
 * Due to its construction, a binary heap is a nearly complete binary tree. As a result of this, a binary heap
 * has some nice performance characteristics. They are:
 *
 * 1) The maximum/minimum can be found is O(1) -- it's just the top of the heap.
 * 2) Insertion in the heap is O(log N).
 * 3) Extracting the max/min value is O(log N).
 * 4) Modifying the priority of a node is O(log N).
 *
 * For more about heaps, consult Chapter 6 of CLRS.
 */
template <class HeapNode>
class BinaryHeap
{
public:
    /**
     * Constructor for BinaryHeap.
     *
     * \param    comparator      Function to use for comparing node
     * \param    initialSize     Initial size of the heap (default = 0)
     */
    BinaryHeap(bool (*comparator)(const HeapNode& a, const HeapNode& b), size_t initialSize = 0);

    /**
     * Destructor for BinaryHeap.
     */
    ~BinaryHeap();

    /**
     * insert inserts a new node into the heap.
     *
     * NOTE: The provided node is not copied, so the caller still needs to handle the memory of the
     *       provided node and ensure that it is not deleted or destroyed while it is still in the
     *       heap.
     *
     * \param    newNode         Node to be placed in the heap
     */
    void insert(const HeapNode& newNode);

    /**
     * remove removes a node from the heap.
     *
     * NOTE: This is currently a SLOW operation O(N).
     *
     * \param    node                Node to be removed
     */
    void remove(const HeapNode& node);

    /**
     * top takes a peek at the top of the heap. The returned value is the current max/min of the heap.
     * The value is not removed from the heap, so repeated calls to top() without any intermediate
     * calls will return the same node.
     *
     * \return   A point to the node at the top of the heap. 0 if the heap is empty.
     */
    const HeapNode& top(void) const;

    /**
     * extract removes the top element from the heap. It is the responsibility of the caller to handle
     * the memory of the returned HeapNode, as no local copies are made by BinaryHeap.
     *
     * The heap is modified by a call to extract(), so successive calls will return different values.
     *
     * \return   The top node in the heap.
     */
    HeapNode extract(void);

    /**
     * fix moves through the heap and ensures that the heap property is satisfied by every node.
     * If any node fails to satisfy, it will be re-heapified so that it again satisfies the heap property.
     *
     * Fix needs to be called if the priority of any nodes in the heap changes.
     */
    void fix(void);

    /** clear erases all values currently stored in the heap. */
    void clear(void);

    /**
     * size retrieves the current number of elements in the heap.
     *
     * \return   The number of elements in the heap.
     */
    int size(void) const { return numNodes; }

private:
    void initialize(void);

    /** heapify moves a node to the correct position within the heap. */
    void heapify(size_t i);

    /** buildHeap constructs a new heap from existing elements. */
    void buildHeap(const std::vector<HeapNode>& initial);

    /** moveNode moves the node at the specified position into the correct position in the heap. */
    void moveNode(int i);

    /** expandHeap makes the heap bigger. */
    void expandHeap(int minSize);

    // Helpers to calculate the position of various locations in the heap relative to some node
    size_t parent(size_t i) { return i >> 1; }
    size_t left(size_t i) { return i << 1; }
    size_t right(size_t i) { return (i << 1) + 1; }

    // Fields for storing the actual heap
    // Invariant for the class requires that at all times: numNodes <= heapSize.

    HeapNode* heap;    ///< The actual heap -- implemented as an array
    size_t heapSize;   ///< Current size of the heap
    size_t numNodes;   ///< Number of nodes actually in the heap. numNodes <= heapSize

    bool (*comp)(const HeapNode& a,
                 const HeapNode& b);   ///< Comparator function for determining the position of a node in the heap
};


template <class HeapNode>
BinaryHeap<HeapNode>::BinaryHeap(bool (*comparator)(const HeapNode& a, const HeapNode& b), size_t initialSize)
: heapSize(initialSize)
, numNodes(0)
, comp(comparator)
{
    initialize();
}


template <class HeapNode>
BinaryHeap<HeapNode>::~BinaryHeap(void)
{
    // Delete the slab of HeapNodes. Individual heap node memory is not maintained by BinaryHeap,
    // so only the array needs to be deleted
    delete[] heap;
}


template <class HeapNode>
void BinaryHeap<HeapNode>::insert(const HeapNode& newNode)
{
    /*
     * To insert a node into the heap, perform the following steps:
     *
     * 1) Increase the number of nodes by 1.
     * 2) Expand the heap if numNodes > heapSize
     * 3) Place the new node at the end of the heap, delinated by numNodes - 1.
     * 4) Perform a moveNode on the newly inserted node.
     */

    ++numNodes;   // This potentially breaks the invariant, but as long as it holds at the end of the method, everything
                  // is fine

    if (numNodes > heapSize) {
        expandHeap(numNodes);
    }

    heap[numNodes - 1] = newNode;

    moveNode(numNodes - 1);

    assert(numNodes <= heapSize);
}


template <class HeapNode>
void BinaryHeap<HeapNode>::remove(const HeapNode& node)
{
    // For now this is going to be SLOW!

    // Do a linear search to find the node, copy the last node in place, then run a fix operation
    int x = numNodes;
    while (heap[--x] != node && x >= 0)
        ;

    if (x >= 0) {
        heap[x] = heap[--numNodes];
        fix();
    }
}


template <class HeapNode>
const HeapNode& BinaryHeap<HeapNode>::top(void) const
{
    // The top of the heap, is heap[0]

    assert(numNodes <= heapSize);

    if (numNodes == 0)   // empty heap
    {
        return 0;
    } else {
        return heap[0];
    }
}


template <class HeapNode>
HeapNode BinaryHeap<HeapNode>::extract(void)
{
    /*
     * To extract the top, do the following:
     *
     * 1) Save the top node in the heap.
     * 2) Move the last node to the top.
     * 3) Heapify the new top.
     */

    assert(numNodes <= heapSize);

    HeapNode top = heap[0];

    heap[0] = heap[--numNodes];

    heapify(0);

    return top;
}


template <class HeapNode>
void BinaryHeap<HeapNode>::fix(void)
{
    /*
     * To fix, go through the first (size / 2) nodes and heapify them, which will ensure that they are in the proper
     * position.
     */

    for (int x = numNodes / 2; --x >= 0;) {
        heapify(x);
    }
}


template <class HeapNode>
void BinaryHeap<HeapNode>::clear(void)
{
    numNodes = 0;
}


template <class HeapNode>
void BinaryHeap<HeapNode>::initialize(void)
{
    if (heapSize > 0) {
        heap = new HeapNode[heapSize];
    } else {
        heap = 0;
    }
}


template <class HeapNode>
void BinaryHeap<HeapNode>::heapify(size_t i)
{
    /*
     * To heapify, a node moved down the heap until is satisfies the heap property. To this, follow these steps:
     *
     * 1) Get the left and right children of the node.
     * 2) Find the value with the highest/lowest priority amongst the node and its children.
     * 3) If the largest value is not the parent, swap the two.
     * 4) Repeat this process on the original node until it satisfies the heap property.
     */

    assert(numNodes <= heapSize);

    // Can't heapify an empty heap
    if ((numNodes <= 0) || (i > numNodes)) {
        return;
    }

    size_t leftNode = 0;
    size_t rightNode = 0;
    size_t parentNode = 0;

    while (i < numNodes) {
        leftNode = left(i);
        rightNode = right(i);
        parentNode = i;

        // See if the left node should be the parent of i
        if ((leftNode < numNodes) && !comp(heap[i], heap[leftNode])) {
            parentNode = leftNode;
        }

        // See if right node should be the parent of parent
        if ((rightNode < numNodes) && !comp(heap[parentNode], heap[rightNode])) {
            parentNode = rightNode;
        }

        // If the parent is still i, then bail out, otherwise, reassign some variables and have another go at things
        if (parentNode == i) {
            break;
        } else   // need to swap parent and i, then set parent to be i and go through the process again
        {
            std::swap(heap[i], heap[parentNode]);
            i = parentNode;
        }
    }
}


template <class HeapNode>
void BinaryHeap<HeapNode>::buildHeap(const std::vector<HeapNode>& initial)
{
    /*
     * To build the heap:
     *
     * 1) Copy all values into the heap.
     * 2) For the first initial.size() / 2, call heapify.
     */

    // If doing the initial adding, numNodes MUST be 0.
    assert(numNodes == 0);

    // Make sure the heap is large enough for our purposes
    if (heapSize < initial.size()) {
        expandHeap(initial.size());
    }

    for (auto iniIt = initial.begin(), endIt = initial.end(); iniIt != endIt; ++iniIt) {
        heap[numNodes++] = *iniIt;
    }

    // Need to start in the middle, so the assumption that a node's children a heaps can be satisfied
    for (int x = initial.size() / 2; --x >= 0;) {
        heapify(x);
    }
}


template <class HeapNode>
void BinaryHeap<HeapNode>::moveNode(int i)
{
    /*
     * This is often called increase/decreaseKey.
     * Move node adjusts takes a node and a position and slides it up through the heap
     * until it sits in the correct position. This is the opposite of heapify(), which
     * moves elements down the heap.
     *
     * 1) Until the node's position satisfies the heap property, swap it with its parent.
     */

    while (i > 0 && !comp(heap[parent(i)], heap[i])) {
        std::swap(heap[parent(i)], heap[i]);
        i = parent(i);
    }
}


template <class HeapNode>
void BinaryHeap<HeapNode>::expandHeap(int minSize)
{
    // To expand, make the new heap size twice the min size. This will amortize away the cost of insertion to O(1).
    size_t newSize = minSize * 2;

    if (newSize < heapSize) {
        return;
    }

    HeapNode* newHeap = new HeapNode[newSize];

    // Now copy over all the values
    if (heap != 0) {
        for (int x = heapSize; --x >= 0;) {
            newHeap[x] = heap[x];
        }
    }

    delete[] heap;

    heap = newHeap;
    heapSize = newSize;
}

}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_BINARY_HEAP_H

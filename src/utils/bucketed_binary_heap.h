/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     bucketed_binary_heap.h
 * \author   Collin Johnson
 *
 * Definition of BucketedBinaryHeap.
 */

#ifndef UTILS_BUCKETED_BINARY_HEAP_H
#define UTILS_BUCKETED_BINARY_HEAP_H

#include <cassert>
#include <cstddef>
#include <deque>
#include <unordered_map>

namespace vulcan
{
namespace utils
{


/**
 * BucketedBinaryHeap is similar to a standard binary heap, but nodes with the same priority are stored in
 * an unordered list. If an application will have many nodes with identical values and the ordering of those nodes
 * does not matter, the BucketedBinaryHeap will be considerably faster.
 *
 * A BucketedBinaryHeap is a data structure where each node has a parent and a left and right child. Each node
 * satisfies the following constraint:
 *
 *       priority(A) > priority(left(A)) && priority(right(A))  (for a max heap)
 *       priority(A) < priority(left(A)) && priority(right(A))  (for a min heap)
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
 *
 * Comparator must be default-constructible.
 */
template <typename HeapNode, typename priority_t, class Comparator = std::less<HeapNode>>
class BucketedBinaryHeap
{
public:
    /**
     * Constructor for BucketedBinaryHeap.
     *
     * BucketedBinaryHeap uses operator< for comparing nodes by default.
     *
     * \param    initialSize     Initial size of the heap (default = 0)
     */
    BucketedBinaryHeap(std::size_t initialSize = 0);

    /**
     * Destructor for BucketedBinaryHeap.
     */
    ~BucketedBinaryHeap(void);

    /**
     * insert inserts a new node into the heap.
     *
     * \param    newNode         Node to be placed in the heap
     */
    void insert(const HeapNode& newNode);

    /**
     * push pushes a new node into the heap.
     *
     * \param    newNode         Node to be placed in the heap
     */
    void push(const HeapNode& newNode) { insert(newNode); }

    /**
     * top takes a peek at the top of the heap. The returned value is the current max/min of the heap.
     * The value is not removed from the heap, so repeated calls to top() without any intermediate
     * calls will return the same node.
     *
     * \return   The node at the top of the heap. HeapNode() is returned if the heap is empty.
     */
    const HeapNode& top(void) const;

    /**
     * pop pops the top node off the queue.
     */
    void pop(void) { extract(); }

    /**
     * extract removes the top element from the heap. It is the responsibility of the caller to handle
     * the memory of the returned HeapNode, as no local copies are made by BinaryHeap.
     *
     * The heap is modified by a call to extract(), so successive calls will return different values.
     *
     * \return   The top node in the heap. HeapNode() is returned if the heap is empty.
     */
    HeapNode extract(void);

    /** clear erases all values currently stored in the heap. */
    void clear(void);

    /**
     * size retrieves the current number of elements in the heap.
     *
     * \return   The number of elements in the heap.
     */
    int size(void) const { return numNodes; }

    /**
     * empty checks if the queue is empty.
     */
    bool empty(void) const { return numNodes == 0; }

private:
    struct heap_bucket_t
    {
        priority_t priority;   ///< Priority of nodes in this bucket
        std::deque<HeapNode> values;

        heap_bucket_t(const HeapNode& firstNode) : priority(firstNode.getPriority()) { values.push_back(firstNode); }
    };

    // Helpers to calculate the position of various locations in the heap relative to some node
    inline size_t parent(size_t i) { return i >> 1; }
    inline size_t left(size_t i) { return i << 1; }
    inline size_t right(size_t i) { return (i << 1) + 1; }

    /** heapify moves a node to the correct position within the heap. */
    void heapify(size_t i);

    /** moveNode moves the node at the specified position into the correct position in the heap. */
    void moveNode(int i, heap_bucket_t* newBucket);

    /** expandHeap makes the heap bigger. */
    void expandHeap(int minSize);

    // Fields for storing the actual heap

    // Invariant for the class requires that at all times: numNodes <= heapSize.

    heap_bucket_t** heap;   ///< The actual heap -- implemented as an array
    size_t heapSize;        ///< Current size of the heap
    size_t numNodes;        ///< Number of nodes actually in the heap. numNodes <= heapSize

    std::unordered_map<priority_t, int> priorityIndices;   ///< Keep track of which priority is at which index to avoid
                                                           ///< a linear search when inserting a new node

    Comparator comp;
};


template <typename HeapNode, typename priority_t, class Comparator>
BucketedBinaryHeap<HeapNode, priority_t, Comparator>::BucketedBinaryHeap(std::size_t initialSize)
: heap(0)
, heapSize(initialSize)
, numNodes(0)
{
    if (heapSize > 0) {
        heap = new heap_bucket_t*[heapSize];
    }
}


template <typename HeapNode, typename priority_t, class Comparator>
BucketedBinaryHeap<HeapNode, priority_t, Comparator>::~BucketedBinaryHeap(void)
{
    // Delete the slab of HeapNodes. Individual heap node memory is not maintained by BinaryHeap,
    // so only the array needs to be deleted
    for (size_t n = 0; n < heapSize; ++n) {
        delete heap[n];
    }

    delete[] heap;
}


template <typename HeapNode, typename priority_t, class Comparator>
void BucketedBinaryHeap<HeapNode, priority_t, Comparator>::insert(const HeapNode& newNode)
{
    /*
     * To insert a node into the heap, perform the following steps:
     *
     * 1) Increase the number of nodes by 1.
     * 2) Expand the heap if numNodes > heapSize
     * 3) Place the new node at the end of the heap, delinated by numNodes - 1.
     * 4) Perform a moveNode on the newly inserted node.
     */

    auto index = priorityIndices.find(newNode.getPriority());

    if (index != priorityIndices.end()) {
        heap[index->second]->values.push_back(newNode);
    } else {
        ++numNodes;   // This potentially breaks the invariant, but as long as it holds at the end of the method,
                      // everything is fine

        // See if the node falls in a current bin
        if (numNodes > heapSize) {
            expandHeap(numNodes);
        }

        heap[numNodes - 1] = new heap_bucket_t(newNode);
        priorityIndices[heap[numNodes - 1]->priority] = numNodes - 1;

        moveNode(numNodes - 1, heap[numNodes - 1]);

        assert(numNodes <= heapSize);
    }
}


template <typename HeapNode, typename priority_t, class Comparator>
const HeapNode& BucketedBinaryHeap<HeapNode, priority_t, Comparator>::top(void) const
{
    // The top of the heap, is heap[0]

    assert(numNodes <= heapSize);
    assert(numNodes > 0);
    return heap[0]->values.front();
}


template <typename HeapNode, typename priority_t, class Comparator>
HeapNode BucketedBinaryHeap<HeapNode, priority_t, Comparator>::extract(void)
{
    /*
     * To extract the top, do the following:
     *
     * 1) Save the top node in the heap.
     * 2) Move the last node to the top.
     * 3) Heapify the new top.
     */

    assert(numNodes <= heapSize);

    // Check that there are actually values in the queue
    if (numNodes == 0) {
        return HeapNode();
    }

    assert(!heap[0]->values.empty());

    HeapNode top = heap[0]->values.front();

    heap[0]->values.pop_front();

    assert(priorityIndices[heap[0]->priority] == 0);

    if (heap[0]->values.empty()) {
        priorityIndices.erase(heap[0]->priority);
        delete heap[0];

        heap[0] = heap[--numNodes];
        heap[numNodes] = 0;   // indicate the old position to now be empty

        if (numNodes > 0) {
            priorityIndices[heap[0]->priority] = 0;

            heapify(0);
        }
    }

    return top;
}


template <typename HeapNode, typename priority_t, class Comparator>
void BucketedBinaryHeap<HeapNode, priority_t, Comparator>::clear(void)
{
    // To clear, go through the heap and set all the pointers to 0.
    for (int x = heapSize; --x >= 0;) {
        delete heap[x];
        heap[x] = 0;
    }

    numNodes = 0;
}


template <typename HeapNode, typename priority_t, class Comparator>
void BucketedBinaryHeap<HeapNode, priority_t, Comparator>::heapify(size_t i)
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
        if ((leftNode < numNodes) && !comp(heap[i]->values.front(), heap[leftNode]->values.front())) {
            parentNode = leftNode;
        }

        // See if right node should be the parent of parent
        if ((rightNode < numNodes) && !comp(heap[parentNode]->values.front(), heap[rightNode]->values.front())) {
            parentNode = rightNode;
        }

        // If the parent is still i, then bail out, otherwise, reassign some variables and have another go at things
        if (parentNode == i) {
            break;
        } else   // need to swap parent and i, then set parent to be i and go through the process again
        {
            heap_bucket_t* temp = heap[i];

            heap[i] = heap[parentNode];
            heap[parentNode] = temp;

            priorityIndices[heap[parentNode]->priority] = parentNode;
            priorityIndices[heap[i]->priority] = i;

            i = parentNode;
        }
    }

    // Fall off the bottom, et fini
}


template <typename HeapNode, typename priority_t, class Comparator>
void BucketedBinaryHeap<HeapNode, priority_t, Comparator>::moveNode(int i, heap_bucket_t* newNode)
{
    /*
     * Move node adjusts takes a node and a position and slides it up through the heap
     * until it sits in the correct position. This is the opposite of heapify(), which
     * moves elements down the heap.
     *
     * 1) Until the node's position satisfies the heap property, swap it with its parent.
     */

    heap_bucket_t* swapper = 0;

    while (i > 0 && !comp(heap[parent(i)]->values.front(), heap[i]->values.front())) {
        swapper = heap[parent(i)];
        heap[parent(i)] = heap[i];
        heap[i] = swapper;

        priorityIndices[heap[parent(i)]->priority] = parent(i);
        priorityIndices[heap[i]->priority] = i;

        i = parent(i);
    }
}


template <typename HeapNode, typename priority_t, class Comparator>
void BucketedBinaryHeap<HeapNode, priority_t, Comparator>::expandHeap(int minSize)
{
    // To expand, make the new heap size twice the min size. This will amortize away the cost of insertion to O(1).

    size_t newSize = minSize * 2;

    // If the new size is less than the current heap size, all done here
    if (newSize < heapSize) {
        return;
    }

    heap_bucket_t** newHeap = new heap_bucket_t*[newSize];

    // Now copy over all the values
    if (heap != 0) {
        for (int x = heapSize; --x >= 0;) {
            newHeap[x] = heap[x];
        }
    }

    // Zero out the other values
    for (size_t x = heapSize; x < newSize; ++x) {
        newHeap[x] = 0;
    }

    // Switch over the heaps, copy the sizes, and delete the old heap
    delete[] heap;

    heap = newHeap;
    heapSize = newSize;
}


}   // namespace utils
}   // namespace vulcan

#endif   // UTILS_BUCKETED_BINARY_HEAP_H

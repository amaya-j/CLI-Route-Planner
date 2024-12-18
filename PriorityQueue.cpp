//========================================================
// PriorityQueue.cpp
// Trinity Meckel & Amaya Joshi
// December 18, 2024
//
// Description:
// This file implements the MinPriorityQueue class.
//========================================================
#include "PriorityQueue.hpp"

//==============================================================
// MinPriorityQueue
// Default constructor for the MinPriorityQueue class. Initializes 
// an empty priority queue with no elements.
// PARAMETERS:
// - none
// RETURN VALUE:
// - none
//==============================================================
template <typename T>
MinPriorityQueue<T>::MinPriorityQueue() {}

//==============================================================
// MinPriorityQueue
// Copy Constructor for the MinPriorityQueue class. Copies the 
// contents of another MinPriorityQueue object. This ensures 
// that both the heap and position map are copied correctly.
// PARAMETERS:
// - other: const MinPriorityQueue<T>& -> The queue to copy.
// RETURN VALUE:
// - none
//==============================================================
template <typename T>
MinPriorityQueue<T>::MinPriorityQueue(const MinPriorityQueue<T>& other) 
{
    // Copy the heap and position map from the other queue
    heap = other.heap;
    position = other.position;
}

//==============================================================
// ~MinPriorityQueue
// Destructor for the MinPriorityQueue class. Cleans up resources.
// PARAMETERS:
// - none
// RETURN VALUE:
// - none
//==============================================================
template <typename T>
MinPriorityQueue<T>::~MinPriorityQueue() {}

//==============================================================
// operator=
// Assignment operator for the MinPriorityQueue. Copies the heap 
// and position map from the other queue.
// PARAMETERS:
// - other: const MinPriorityQueue<T>& -> The queue to assign.
// RETURN VALUE:
// - *this -> The current object after the assignment.
//==============================================================
template <typename T>
MinPriorityQueue<T>& MinPriorityQueue<T>::operator=(const MinPriorityQueue<T>& other) {
    if (this != &other)  // Check for self-assignment
    {  
        heap = other.heap;
        position = other.position;
    }
    return *this;
}

//==============================================================
// heapifyUp
// Helper function to restore the min-heap property by moving an 
// element up the heap when its priority is decreased.
// PARAMETERS:
// - index: size_t -> The index of the element to heapify up.
// RETURN VALUE:
// - none
//==============================================================
template <typename T>
void MinPriorityQueue<T>::heapifyUp(size_t index) 
{
    while (index > 0) {
        size_t parentIndex = (index - 1) / 2;
        if (heap[index].second >= heap[parentIndex].second) 
        {
            break; // Heap property satisfied
        }
        // Swap current node with parent
        swap(heap[index], heap[parentIndex]);
        // Update position map
        position[heap[index].first] = index;
        position[heap[parentIndex].first] = parentIndex;
        // Move up the tree
        index = parentIndex;
    }
}

//==============================================================
// heapifyDown
// Helper function to restore the min-heap property by moving an 
// element down the heap after removal or modification.
// PARAMETERS:
// - index: size_t -> The index of the element to heapify down.
// RETURN VALUE:
// - none
//==============================================================
template <typename T>
void MinPriorityQueue<T>::heapifyDown(size_t index) 
{
    size_t size = heap.size();
    while (true) {
        size_t leftChild = 2 * index + 1;
        size_t rightChild = 2 * index + 2;
        size_t smallest = index;

        // Find the smallest among parent and children
        if (leftChild < size && heap[leftChild].second < heap[smallest].second) 
        {
            smallest = leftChild;
        }
        if (rightChild < size && heap[rightChild].second < heap[smallest].second) 
        {
            smallest = rightChild;
        }

        if (smallest == index) 
        {
            break; // Heap property satisfied
        }

        // Swap current node with smallest child
        swap(heap[index], heap[smallest]);
        // Update position map
        position[heap[index].first] = index;
        position[heap[smallest].first] = smallest;
        // Move down the tree
        index = smallest;
    }
}

//==============================================================
// insert
// Inserts a new element into the priority queue with a given priority.
// PARAMETERS:
// - value: T -> The value to be inserted.
// - priority: double -> The priority associated with the value.
// RETURN VALUE:
// - none
// EXCEPTIONS:
// - Throws invalid_argument if the element already exists in the queue.
//==============================================================
template <typename T>
void MinPriorityQueue<T>::insert(T value, double priority) 
{
    if (position.find(value) != position.end()) 
    {
        throw invalid_argument("Element already exists in the priority queue");
    }

    // Add the new element to the end of the heap
    heap.push_back({value, priority});
    size_t index = heap.size() - 1;
    position[value] = index;

    // Restore heap property
    heapifyUp(index);
}

//==============================================================
// extractMin
// Removes and returns the element with the minimum priority from 
// the priority queue.
// PARAMETERS:
// - none
// RETURN VALUE:
// - pair<T, double> -> A pair containing the value and its priority.
// EXCEPTIONS:
// - Throws underflow_error if the priority queue is empty.
//==============================================================
template <typename T>
pair<T, double> MinPriorityQueue<T>::extractMin() 
{
    if (isEmpty()) 
    {
        throw underflow_error("Priority queue is empty");
    }

    // The root of the heap is the minimum element
    pair<T, double> minElement = heap[0];

    // Replace root with the last element
    heap[0] = heap.back();
    position[heap[0].first] = 0;
    heap.pop_back();
    position.erase(minElement.first);

    // Restore heap property
    if (!heap.empty()) 
    {
        heapifyDown(0);
    }

    return minElement;
}

//==============================================================
// decreaseKey
// Decreases the priority of an existing element in the priority queue.
// PARAMETERS:
// - value: T -> The value whose priority is to be decreased.
// - newPriority: double -> The new priority to assign to the value.
// RETURN VALUE:
// - none
// EXCEPTIONS:
// - Throws invalid_argument if the element is not in the queue or 
//   if the new priority is not smaller than the current priority.
//==============================================================
template <typename T>
void MinPriorityQueue<T>::decreaseKey(T value, double newPriority) 
{
    if (position.find(value) == position.end()) 
    {
        throw invalid_argument("Element not found in the priority queue");
    }

    size_t index = position[value];
    if (newPriority >= heap[index].second) 
    {
        throw invalid_argument("New priority must be smaller than the current priority");
    }

    // Update the priority
    heap[index].second = newPriority;

    // Restore heap property
    heapifyUp(index);
}

//==============================================================
// isEmpty
// Checks if the priority queue is empty.
// PARAMETERS:
// - none
// RETURN VALUE:
// - bool -> True if the priority queue is empty, otherwise false.
//==============================================================
template <typename T>
bool MinPriorityQueue<T>::isEmpty() const 
{
    return heap.empty();
}


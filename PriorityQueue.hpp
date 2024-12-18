//========================================================
// PriorityQueue.hpp
// Trinity Meckel & Amaya Joshi
// December 18, 2024
//
// Description:
// This header file defines the MinPriorityQueue class template.
// The class represents a minimum priority queue implemented using a binary heap.
// It supports operations like inserting an element with a given priority, 
// extracting the element with the minimum priority, decreasing the priority of 
// an existing element, and checking if the queue is empty. The queue uses a 
// vector to store the elements and a hash map (unordered_map) to track the 
// positions of elements in the heap for efficient priority updates.
//========================================================
#ifndef PRIORITY_QUEUE_HPP
#define PRIORITY_QUEUE_HPP

#include <vector>
#include <unordered_map>
#include <stdexcept>
#include <utility> 

using namespace std; 

template <typename T>
class MinPriorityQueue 
{
private:
    // A vector to represent the binary heap
    vector<pair<T, double>> heap; // (value, priority)

    // A map to store the index of each element in the heap
    unordered_map<T, size_t> position;

    // Helper methods for heap operations
    void heapifyUp(size_t index);
    void heapifyDown(size_t index);

public:
    MinPriorityQueue();
    MinPriorityQueue(const MinPriorityQueue<T>& other);
    MinPriorityQueue<T>& operator=(const MinPriorityQueue<T>& other);
    ~MinPriorityQueue();

    // Core methods
    void insert(T value, double priority);       
    pair<T, double> extractMin();               
    void decreaseKey(T value, double newPriority); 
    bool isEmpty() const;                       
};

#endif


//========================================================
// Graph.hpp
// Trinity Meckel & Amaya Joshi
// December 18, 2024
//
// Description:
// This header file defines the Graph class, which represents a weighted
// graph using an adjacency list. The class provides methods for adding
// vertices, adding weighted edges, and performing graph traversals like
// DFS and BFS. The DFS implementation supports tracking discovery and
// finish times for each vertex, enabling operations like topological sorting.
//========================================================
#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "customexceptions.hpp"
#include <list>
#include <tuple>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <limits>
#include <algorithm>

#include <iostream>
using namespace std;

class Graph
{
private:
    // Adjacency list now stores pairs (neighbor, weight)
    unordered_map<int, vector<pair<int, double>>> Adj;
    vector<int> sorted;
    vector<int> order;

    bool check_vertex(int u) const;
    bool check_edge(int u, int v) const;
    bool is_empty() const;


public:
    Graph();
    Graph(const Graph &G);
    ~Graph();
    Graph &operator=(const Graph &G);

    // Methods for manipulating the graph
    void addEdge(int u, int v, double weight); // Adds a weighted edge
    void removeEdge(int u, int v);
    bool edgeIn(int u, int v);
    

    void addVertex(int u);
    void removeVertex(int u);

    // Accessors
    vector<int> getVertices() const;
    vector<pair<int, double>> getEdges(int v) const;
    
    // Utility methods
    void printGraph() const;
    
    // Traversals and other graph methods
    unordered_map<int, pair<int, int>> breadthFirstSearch(int s);
    unordered_map<int, tuple<int, int, int>> depthFirstSearch(bool sort = false);
    void DFS_visit(int u, unordered_map<int, tuple<int, int, int>> &DFS, int &time, bool sort);
    vector<int> getOrdering();

    // Static method to construct a graph from stdin
    static Graph readFromSTDIN();

    // Dijkstra's algorithm
    unordered_map<int, pair<int, double>> dijkstra(int start);
};

#endif



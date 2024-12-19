//========================================================
// Graph.cpp
// Trinity Meckel & Amaya Joshi
// December 18, 2024
//
// Description:
// This source file implements the methods of the Graph class as
// defined in Graph.hpp.
//========================================================
#include "Graph.hpp"
#include "PriorityQueue.cpp"

using namespace std;

//==============================================================
// Graph
// Default constructor for the Graph class. Initializes an empty
// graph with no vertices or edges.
// PARAMETERS:
// - none
// RETURN VALUE:
// - none
//==============================================================
Graph::Graph() {};

//==============================================================
// Graph
// Copy constructor for the Graph class. Creates a deep copy of
// the given graph, duplicating its adjacency list and sorted order.
// PARAMETERS:
// - G: A reference to the Graph object to copy.
// RETURN VALUE:
// - none
//==============================================================
Graph::Graph(const Graph &G)
{
    Adj = G.Adj;
    sorted = G.sorted;
};

//==============================================================
// ~Graph
// Destructor for the Graph class. Frees any dynamically allocated
// resources (if applicable).
// PARAMETERS:
// - none
// RETURN VALUE:
// - none
//==============================================================
Graph::~Graph() {};

//==============================================================
// operator=
// Assignment operator for the Graph class. Copies the adjacency
// list and sorted order from the given graph.
// PARAMETERS:
// - G: A reference to the Graph object to copy.
// RETURN VALUE:
// - A reference to the updated Graph object.
//==============================================================
Graph &Graph::operator=(const Graph &G)
{
    Adj = G.Adj;
    sorted = G.sorted;
    return *this;
};

//==============================================================
// check_vertex
// Checks if a vertex exists in the graph.
// PARAMETERS:
// - v: The vertex to check.
//==============================================================
bool Graph::check_vertex(int v) const
{
    return Adj.find(v) != Adj.end();
}

//==============================================================
// check_edge
// Checks if a directed edge exists between two vertices.
// PARAMETERS:
// - u: The source vertex.
// - v: The destination vertex.
// RETURN VALUE:
// - true if the edge exists, false otherwise.
//==============================================================
bool Graph::check_edge(int u, int v) const
{
    if (!check_vertex(u)) return false;
    for (const auto &edge : Adj.at(u))
    {
        if (edge.first == v)
        {
            return true;
        }
    }
    return false;
}

//==============================================================
// is_empty
// Checks if the graph is empty.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the graph is empty, false otherwise.
//==============================================================
bool Graph::is_empty() const
{
    return sorted.empty();
}

//==============================================================
// addEdge
// Adds a weighted edge between two vertices.
// PARAMETERS:
// - u: The source vertex.
// - v: The destination vertex.
// - weight: The weight of the edge.
//==============================================================
void Graph::addEdge(int u, int v, double weight)
{
    if (!check_vertex(u)) addVertex(u);
    if (!check_vertex(v)) addVertex(v);

    if (!edgeIn(u, v))
    {
        Adj[u].emplace_back(v, weight);
    }
}

//==============================================================
// removeEdge
// Removes an edge between two vertices.
// Throws an edge_exception if the edge does not exist.
// PARAMETERS:
// - u: The source vertex.
// - v: The destination vertex.
//==============================================================
void Graph::removeEdge(int u, int v)
{
    if (!check_vertex(u) || !check_vertex(v) || !edgeIn(u, v)) throw edge_exception();

    auto &edges = Adj[u];
    edges.erase(remove_if(edges.begin(), edges.end(),
                          [v](const pair<int, double> &edge) { return edge.first == v; }),
                edges.end());
}

//==============================================================
// edgeIn
// Checks if an edge exists between two vertices.
// PARAMETERS:
// - u: The source vertex.
// - v: The destination vertex.
//==============================================================
bool Graph::edgeIn(int u, int v)
{
    return check_edge(u, v); // Reuse the existing function
}

//==============================================================
// addVertex
// Adds a vertex to the graph.
// PARAMETERS:
// - v: The vertex to add.
//==============================================================
void Graph::addVertex(int v)
{
    if (!check_vertex(v))
    {
        Adj[v] = vector<pair<int, double>>();
    }
}

//==============================================================
// removeVertex
// Removes a vertex and all its associated edges from the graph.
// Throws a vertex_exception if the vertex does not exist.
// PARAMETERS:
// - v: The vertex to remove.
//==============================================================
void Graph::removeVertex(int v)
{
    if (!check_vertex(v)) throw vertex_exception();

    // Remove all edges to this vertex
    for (auto &entry : Adj)
    {
        auto &edges = entry.second;
        edges.erase(remove_if(edges.begin(), edges.end(),
                              [v](const pair<int, double> &edge) { return edge.first == v; }),
                    edges.end());
    }

    // Remove the vertex itself
    Adj.erase(v);
}

//==============================================================
// getVertices
// Returns a vector of all vertices in the graph.
//==============================================================
vector<int> Graph::getVertices() const
{
    vector<int> vertices;
    for (const auto &entry : Adj)
    {
        vertices.push_back(entry.first);
    }
    return vertices;
}

//==============================================================
// getEdges
// Returns a vector of all edges from a given vertex in the form
// of pairs (destination, weight). Throws a vertex_exception if the
// vertex does not exist.
// PARAMETERS:
// - v: The source vertex.
//==============================================================
vector<pair<int, double>> Graph::getEdges(int v) const
{
    if (!check_vertex(v)) throw vertex_exception();
    return Adj.at(v);
}

//==============================================================
// printGraph
// Prints the adjacency list of the graph.
//==============================================================
void Graph::printGraph() const
{
    for (const auto &entry : Adj)
    {
        cout << entry.first << ": ";
        for (const auto &edge : entry.second)
        {
            cout << "(" << edge.first << ", " << edge.second << ") ";
        }
        cout << endl;
    }
}

//==============================================================
// breadthFirstSearch
// Performs BFS starting from a given vertex. Returns a map of vertices
// with their distances from the start vertex.
// PARAMETERS:
// - start: The starting vertex.
//==============================================================
unordered_map<int, pair<int, int>> Graph::breadthFirstSearch(int start)
{
    // Check if the start vertex exists
    if (!check_vertex(start))
        throw vertex_exception();

    // Data structures for BFS
    unordered_map<int, pair<int, int>> bfsResult; // vertex -> (parent, distance)
    unordered_set<int> visited;
    queue<int> q;

    // Initialize BFS
    visited.insert(start);
    bfsResult[start] = {-1, 0}; // No parent, distance 0
    q.push(start);

    // BFS loop
    while (!q.empty())
    {
        int curr = q.front();
        q.pop();

        // Ensure curr is in adjacency list
        if (Adj.find(curr) == Adj.end()) continue;

        for (const auto &edge : Adj.at(curr))
        {
            int neighbor = edge.first;
            if (visited.find(neighbor) == visited.end())
            {
                visited.insert(neighbor);
                bfsResult[neighbor] = {curr, bfsResult[curr].second + 1};
                q.push(neighbor);
            }
        }
    }

    return bfsResult;
}

//==============================================================
// DFS_visit
// Recursive helper function for DFS.
// PARAMETERS:
// - v: The current vertex.
// - DFS: DFS result map (vertex -> (parent, discovery time, finish time)).
// - time: Current time.
// - sort: Whether to sort adjacency lists for consistent ordering.
//==============================================================
void Graph::DFS_visit(int u, unordered_map<int, tuple<int, int, int>> &DFS, int &time, bool sortFlag)
{
    time++;
    get<1>(DFS[u]) = time; // Discovery time

    if (sortFlag) // sort the adjacency list conditionally for consistent ordering
    {
        sort(Adj[u].begin(), Adj[u].end());
    }

    for (const auto &edge : Adj[u])
    {
        int v = edge.first;
        if (DFS.find(v) == DFS.end())
        {
            DFS[v] = {u, 0, 0}; // Initialize parent
            DFS_visit(v, DFS, time, sortFlag);
        }
    }

    time++;
    get<2>(DFS[u]) = time; // Finish time
}

//==============================================================
// depthFirstSearch
// Performs DFS and returns a map of vertices with their parent,
// discovery time, and finish time.
// PARAMETERS:
// - sort: Whether to sort adjacency lists for consistent ordering.
//==============================================================
unordered_map<int, tuple<int, int, int>> Graph::depthFirstSearch(bool sort)
{
    unordered_map<int, tuple<int, int, int>> DFS;
    int time = 0;

    for (const auto &entry : Adj)
    {
        int u = entry.first;
        if (DFS.find(u) == DFS.end())
        {
            DFS[u] = {-1, 0, 0}; // No parent
            DFS_visit(u, DFS, time, sort);
        }
    }

    return DFS;
}

//==============================================================
// getOrdering
// Returns the topological ordering of the vertices if the graph is a DAG.
//==============================================================
vector<int> Graph::getOrdering()
{
    if (sorted.empty())
    {
        depthFirstSearch(true);
        reverse(order.begin(), order.end());
    }
    return order;
}

//==============================================================
// readFromSTDIN
// Reads a graph's vertices and weighted edges from standard input.
// Expects the number of vertices (n) and edges (m) followed by m
// triples (u, v, w) representing directed edges and their weights.
// PARAMETERS:
// - none
// RETURN VALUE:
// - A Graph object initialized with the input data.
//==============================================================
Graph Graph::readFromSTDIN()
{
    Graph G;
    int n, m;
    cin >> n >> m;

    for (int i = 1; i <= n; i++)
    {
        G.addVertex(i);
    }

    for (int i = 0; i < m; ++i)
    {
        int u, v;
        double w;
        cin >> u >> v >> w;
        G.addEdge(u, v, w);
    }
    return G;
}

//==============================================================
// Dijkstra
// Computes the shortest paths from a start vertex to all other vertices.
// PARAMETERS:
// - start: The starting vertex.
// RETURNS:
// - A map of vertices to their parent and shortest distance (vertex -> (parent, distance)).
//==============================================================
unordered_map<int, pair<int, double>> Graph::dijkstra(int start) 
{
    unordered_map<int, double> distances;    // Map to store shortest distances to each vertex
    unordered_map<int, int> predecessors;   // Map to store the predecessor of each vertex
    MinPriorityQueue<int> pq;               // Priority queue for vertices

    // Initialize distances and insert all vertices into the priority queue
    for (int vertex : getVertices()) 
    {
        distances[vertex] = numeric_limits<double>::infinity(); // Initialize distances to infinity
        pq.insert(vertex, distances[vertex]);                  // Insert with initial priority
    }
    distances[start] = 0.0; // Distance to the start vertex is 0
    pq.decreaseKey(start, 0.0); // Update the priority of the start vertex

    while (!pq.isEmpty()) {
        auto [currentVertex, currentDistance] = pq.extractMin(); // Get vertex with minimum distance

        // Relax edges from the current vertex
        for (auto [neighbor, weight] : getEdges(currentVertex)) 
        {
            double newDistance = currentDistance + weight;
            if (newDistance < distances[neighbor]) 
            {
                distances[neighbor] = newDistance; // Update shortest distance
                predecessors[neighbor] = currentVertex; // Update predecessor
                pq.decreaseKey(neighbor, newDistance); // Update priority in the queue
            }
        }
    }

    // Construct the result map with distances and predecessors
    unordered_map<int, pair<int, double>> result;
    for (auto &vertex : distances) 
    {
        result[vertex.first] = {predecessors.count(vertex.first) ? predecessors[vertex.first] : -1, vertex.second};
    }
    return result;
}

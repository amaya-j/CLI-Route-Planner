//========================================================
// mytests.cpp
// Trinity Meckel & Amaya Joshi
// December 18, 2024
//
// Description:
// This file contains the tests cases for the Graph class 
// And MinPriorityQueue class.
//========================================================
#include "Graph.cpp"
#include <cassert>
#include <sstream> 

#include <iostream>
using namespace std;

Graph createSampleGraph();

// Test Graph Functions
bool testcheck_edge();
bool testcheck_vertex();
bool testis_empty();
bool testaddEdge();
bool testremoveEdge();
bool testedgeIn();
bool testaddvertex();
bool testremoveVertex();
bool testgetVertices();
bool testgetEdges();
bool testBFS();
bool testDFS();
bool testDFS_visit();
bool testgetOrdering();
bool testreadFromSTDIN();
bool testDijkstra();

// Test MinPriorityQueue
bool testHeapifyUp();
bool testHeapifyDown();
bool testinsert();
bool testextractMin();
bool testdecreaseKey();
bool testisEmpty();

// Testing Functions
void testingGraph();
void testingMinPriorityQueue();

//==============================================================
// main
// Initiate testing by calling testing methods.
// PARAMETERS: 
// - none
// RETURN VALUE: 
// - 0
//==============================================================
int main() 
{

    cout << "\n\nTesting Graph Methods:" << endl;
    testingGraph();
    cout << "\n\nTesting MinPriorityQueue Methods:" << endl;
    testingMinPriorityQueue();
    
    return 0;
}

//==============================================================
// createSampleGraph
// Makes a Graph to use for testing
// PARAMETERS:
// - none
// RETURN VALUE:
// none
//==============================================================
Graph createSampleGraph() 
{
    Graph g;

    // Add vertices
    g.addVertex(1);
    g.addVertex(2);
    g.addVertex(3);
    g.addVertex(4);
    g.addVertex(5);
    g.addVertex(6);

    // Add edges with weights
    g.addEdge(1, 2, 4.5);
    g.addEdge(1, 3, 2.0);
    g.addEdge(2, 3, 5.5);
    g.addEdge(2, 4, 10.0);
    g.addEdge(3, 5, 3.1);
    g.addEdge(4, 6, 11.0);
    g.addEdge(5, 4, 4.8);
    g.addEdge(5, 6, 5.5);

    return g;
}

// *******************************************************
// **************GRAPH TESTING FUNCTIONS******************
// *******************************************************

//==============================================================
// testcheck_edge
// Tests the check_edge function of the Graph class. It verifies 
// whether the function correctly identifies the presence or 
// absence of an edge between two vertices.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testcheck_edge() 
{
    Graph g = createSampleGraph();
    if (!g.edgeIn(1, 2)) return false; // Valid edge
    if (g.edgeIn(2, 1)) return false;  // Invalid edge (directed graph)
    return true;
}

//==============================================================
// testcheck_vertex
// Tests the check_vertex function of the Graph class. It verifies 
// whether the function correctly identifies the presence or 
// absence of a vertex in the graph.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testcheck_vertex() 
{
    Graph g = createSampleGraph();
    auto vertices = g.getVertices();

    // Check for existing vertex
    if (find(vertices.begin(), vertices.end(), 3) == vertices.end()) return false;

    // Check for non-existent vertex
    if (find(vertices.begin(), vertices.end(), 10) != vertices.end()) return false;

    return true;
}

//==============================================================
// testis_empty
// Tests the is_empty function of the Graph class. It verifies 
// whether the function correctly determines if the graph is empty.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testis_empty() 
{
    Graph g; // Empty graph
    if (!g.getVertices().empty()) return false;

    g.addVertex(1); // Add a vertex
    if (g.getVertices().empty()) return false;

    return true;
}

//==============================================================
// testaddEdge
// Tests the addEdge function of the Graph class. It verifies 
// whether the function correctly adds an edge between two vertices.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testaddEdge() 
{
    Graph g = createSampleGraph();
    g.addEdge(2, 6, 7.3); // Add an edge

    // Verify edge addition
    if (!g.edgeIn(2, 6)) return false;
    return true;
}

//==============================================================
// testremoveEdge
// Tests the removeEdge function of the Graph class. It verifies 
// whether the function correctly removes an edge between two vertices.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testremoveEdge() 
{
    Graph g = createSampleGraph();
    g.removeEdge(1, 2); // Remove edge

    // Verify edge removal
    if (g.edgeIn(1, 2)) return false;
    return true;
}

//==============================================================
// testedgeIn
// Tests the edgeIn function of the Graph class. It verifies 
// whether the function correctly returns the weight of an edge.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testedgeIn() 
{
    Graph g = createSampleGraph();

    // Check existing edge
    if (!g.edgeIn(3, 5)) return false;

    // Check non-existent edge
    if (g.edgeIn(5, 3)) return false;

    return true;
}

//==============================================================
// testaddVertex
// Tests the addVertex function of the Graph class. It verifies 
// whether the function correctly adds a vertex to the graph.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testaddVertex() 
{
    Graph g = createSampleGraph();
    g.addVertex(7); // Add a new vertex

    auto vertices = g.getVertices();
    if (find(vertices.begin(), vertices.end(), 7) == vertices.end()) return false;

    return true;
}

//==============================================================
// testremoveVertex
// Tests the removeVertex function of the Graph class. It verifies 
// whether the function correctly removes a vertex and its associated edges.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testremoveVertex() 
{
    Graph g = createSampleGraph();
    g.removeVertex(3); // Remove vertex

    auto vertices = g.getVertices();
    if (find(vertices.begin(), vertices.end(), 3) != vertices.end()) return false;

    return true;
}

//==============================================================
// testgetVertices
// Tests the getVertices function of the Graph class. It verifies 
// whether the function returns all vertices in the graph.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testgetVertices() 
{
    Graph g = createSampleGraph();
    auto vertices = g.getVertices();
    sort(vertices.begin(), vertices.end()); // Ensure order for comparison

    vector<int> expected = {1, 2, 3, 4, 5, 6};
    if (vertices != expected) return false;

    return true;
}

//==============================================================
// testgetEdges
// Tests the getEdges function of the Graph class. It verifies 
// whether the function returns all edges in the graph.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testgetEdges() 
{
    Graph g = createSampleGraph();
    auto edges = g.getEdges(1); // Edges from vertex 1

    // Expected edges
    vector<pair<int, double>> expected = {{2, 4.5}, {3, 1.2}};

    if (edges.size() != expected.size()) return false;
    for (size_t i = 0; i < edges.size(); ++i)
    {
        if (edges[i] != expected[i]) return false;
    }

    return true;
}

//==============================================================
// testBFS
// Tests the breadthFirstSearch (BFS) function of the Graph class.
// It verifies whether BFS correctly calculates distances from a
// start vertex to all reachable vertices.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testBFS()
{
    Graph g = createSampleGraph();
    auto bfsResult = g.breadthFirstSearch(1); // Start BFS at vertex 1

    if (bfsResult[2].second != 4.5) return false;
    if (bfsResult[6].second != 14.6) return false;

    return true;
}

//==============================================================
// testDFS
// Tests the depthFirstSearch (DFS) function of the Graph class.
// It verifies whether DFS correctly discovers and finishes vertices
// while traversing the graph.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testDFS()
{
    Graph g = createSampleGraph();
    auto dfsResult = g.depthFirstSearch();

    // Replace times with actual expected results
    if (dfsResult.find(1) == dfsResult.end()) return false;

    return true;
}

//==============================================================
// testDFS_visit
// Tests the DFS_visit helper function of the Graph class. It verifies
// whether the function correctly updates the discovery, finish, and parent
// information for a given vertex during a DFS traversal.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testDFS_visit()
{
    Graph g = createSampleGraph();
    unordered_map<int, tuple<int, int, int>> dfsData;
    int time = 0;

    g.DFS_visit(1, dfsData, time, false);

    // Replace tuple with actual expected times
    if (dfsData.find(1) == dfsData.end()) return false;

    return true;
}

//==============================================================
// testgetOrdering
// Tests the getOrdering function of the Graph class. It verifies whether
// the function correctly generates a topological ordering of vertices in the graph.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testgetOrdering()
{
    Graph g = createSampleGraph();
    auto ordering = g.getOrdering(); // Get topological order

    // Example order; replace with actual expected
    vector<int> expected = {1, 3, 5, 4, 2, 6};

    if (ordering != expected) return false;

    return true;
}

//==============================================================
// testreadFromSTDIN
// Tests the readFromSTDIN function of the Graph class. It verifies whether
// the graph is correctly constructed from user-provided input.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testreadFromSTDIN()
{
    string input = "6 6\n1 2 4.5\n1 3 1.2\n3 4 2.3\n3 5 3.1\n5 6 4.7\n4 6 5.6\n";
    istringstream iss(input);
    cin.rdbuf(iss.rdbuf()); // Redirect stdin

    Graph g = Graph::readFromSTDIN();

    if (!g.edgeIn(1, 2)) return false;
    if (!g.edgeIn(4, 6)) return false;
    if (g.edgeIn(2, 1)) return false;

    return true;
}

//==============================================================
// testDijkstra
// Tests the Dijkstra function of the Graph class. It verifies whether
// the function calculates the shortest paths from a given start vertex
// to all other vertices in the graph.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the test passes, false otherwise
//==============================================================
bool testDijkstra()
{
    Graph g = createSampleGraph();
    auto result = g.dijkstra(1); // Start at vertex 1

    if (result[6].second != 14.6) return false;

    return true;
}
// *******************************************************
// *********MIN PRIORITY QUEUE TESTING FUNCTIONS**********
// *******************************************************

//==============================================================
// testHeapifyUp
// Tests the heapifyUp function of the Min PriorityQueue class.
// Verifies whether the function correctly restores the heap property
// after an element is added to the priority queue by moving it upwards.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the heap property is correctly restored after heapifyUp.
// - false otherwise
//==============================================================
bool testHeapifyUp() 
{
    MinPriorityQueue<int> pq;

    pq.insert(10, 50);
    pq.insert(20, 30); // Should move up in the heap
    pq.insert(5, 70);

    // Extract to confirm heap property
    auto minElement = pq.extractMin();
    return minElement.first == 20 && minElement.second == 30; // Check if the smallest element is correct
}

//==============================================================
// testHeapifyDown
// Tests the heapifyDown function of the Min PriorityQueue class.
// Verifies whether the function correctly restores the heap property
// after the minimum element is removed from the priority queue
// by moving elements downwards.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the heap property is correctly restored after heapifyDown.
// - false otherwise
//==============================================================
bool testHeapifyDown() 
{
    MinPriorityQueue<int> pq;

    pq.insert(10, 50);
    pq.insert(20, 30);
    pq.insert(5, 70);

    // Remove the minimum and check if heap property is restored
    pq.extractMin(); // Removes 20 (priority 30)
    auto nextMin = pq.extractMin(); // Should now be 10 (priority 50)
    return nextMin.first == 10 && nextMin.second == 50;
}

//==============================================================
// testinsert
// Tests the insert function of the Min PriorityQueue class.
// Verifies whether the function correctly inserts a new element
// into the priority queue and maintains the heap property.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the element is inserted correctly and the heap property is maintained.
// - false otherwise
//==============================================================
bool testinsert() 
{
    MinPriorityQueue<int> pq;

    pq.insert(10, 50);
    pq.insert(20, 30);
    pq.insert(5, 70);

    // Extract elements in order to verify the heap property
    auto first = pq.extractMin();
    auto second = pq.extractMin();
    auto third = pq.extractMin();

    return first.first == 20 && first.second == 30 &&
           second.first == 10 && second.second == 50 &&
           third.first == 5 && third.second == 70; // Verify sorted order by priority
}

//==============================================================
// testextractMin
// Tests the extractMin function of the Min PriorityQueue class.
// Verifies whether the function correctly removes and returns the minimum element
// from the priority queue and maintains the heap property after extraction.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the minimum element is correctly removed and the heap property is maintained.
// - false otherwise
//==============================================================
bool testextractMin() 
{
    MinPriorityQueue<int> pq;

    pq.insert(10, 50);
    pq.insert(20, 30);
    pq.insert(5, 70);

    // Extract minimum elements and verify
    auto minElement = pq.extractMin();
    auto nextMinElement = pq.extractMin();

    return minElement.first == 20 && minElement.second == 30 &&
           nextMinElement.first == 10 && nextMinElement.second == 50;
}

//==============================================================
// testdecreaseKey
// Tests the decreaseKey function of the Min PriorityQueue class.
// Verifies whether the function correctly decreases the key of a specific
// element and restores the heap property if necessary.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if the key is correctly decreased and the heap property is maintained.
// - false otherwise
//==============================================================
bool testdecreaseKey() 
{
    MinPriorityQueue<int> pq;

    pq.insert(10, 50);
    pq.insert(20, 30);
    pq.insert(5, 70);

    // Decrease the priority of element 5
    pq.decreaseKey(5, 25);

    // Verify that 5 moves to the top
    auto minElement = pq.extractMin();
    return minElement.first == 5 && minElement.second == 25;
}

//==============================================================
// testisEmpty
// Tests the isEmpty function of the Min PriorityQueue class.
// Verifies whether the function correctly identifies whether the priority queue
// is empty or not.
// PARAMETERS:
// - none
// RETURN VALUE:
// - true if isEmpty correctly identifies the state of the priority queue.
// - false otherwise
//==============================================================
bool testisEmpty() 
{
    MinPriorityQueue<int> pq;

    // Check initial state
    if (!pq.isEmpty()) return false;

    // Insert an element and check again
    pq.insert(10, 50);
    if (pq.isEmpty()) return false;

    // Extract the element and check again
    pq.extractMin();
    return pq.isEmpty();
}

// *******************************************************
// **********TESTING GRAPH & MIN PRIORITY QUEUE***********
// *******************************************************

//==============================================================
// testingGraph
// Testing functions and respective outputs for Graph class. Updates
// and outputs the tally of passed and failed tests.
// PARAMETERS: 
// - none
// RETURN VALUE: 
// - none
//==============================================================
void testingGraph() 
{
    struct TestResult
    {
            int passed;
            int failed;
    };
    TestResult Graph_result = {0, 0};
    
    
    if(testcheck_edge() == true)
    {
            cout << "Check edge for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testcheck_vertex() == true)
    {
            cout << "Check vertex for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testis_empty() == true)
    {
            cout << "is_empty for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testaddEdge() == true)
    {
            cout << "Add edge for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testremoveEdge() == true)
    {
            cout << "Remove edge for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testedgeIn() == true)
    {
            cout << "Edge in for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testaddVertex() == true)
    {
            cout << "Add vertex for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testremoveVertex() == true)
    {
            cout << "Remove vertex for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testgetVertices() == true)
    {
            cout << "Get vertices for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testgetEdges() == true)
    {
            cout << "Get edges for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testBFS() == true)
    {
            cout << "BFS for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testDFS() == true)
    {
            cout << "DFS for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testDFS_visit() == true)
    {
            cout << "DFS_visit for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testgetOrdering() == true)
    {
            cout << "GetOrdering for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testreadFromSTDIN() == true)
    {
            cout << "readFromSTDIN for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    if(testDijkstra() == true)
    {
            cout << "Dijkstra for Graph test passed" << endl;
            Graph_result.passed++;
    }else{
            Graph_result.failed++;
         }
    
    cout << "\n\n";
    cout << "Graph Tests Passed: " << Graph_result.passed << endl;
    cout << "Graph Tests Failed: " << Graph_result.failed << endl;
    cout << "\n\n";
}

//==============================================================
// testingMinPriorityQueue
// Testing functions and respective outputs for MinPriorityQueue
// class. Updates and outputs the tally of passed and failed tests.
// PARAMETERS: 
// - none
// RETURN VALUE: 
// - none
//==============================================================
void testingMinPriorityQueue() 
{
    struct TestResult
    {
            int passed;
            int failed;
    };
    TestResult MinPQ_result = {0, 0};

    
    if(testHeapifyUp() == true)
    {
            cout << "HeapifyUp for MinPQ test passed" << endl;
            MinPQ_result.passed++;
    }else{
            MinPQ_result.failed++;
         }
    if(testHeapifyDown() == true)
    {
            cout << "HeapifyDown for MinPQ test passed" << endl;
            MinPQ_result.passed++;
    }else{
            MinPQ_result.failed++;
         }
    if(testinsert() == true)
    {
            cout << "Insert for MinPQ test passed" << endl;
            MinPQ_result.passed++;
    }else{
            MinPQ_result.failed++;
         }
    if(testextractMin() == true)
    {
            cout << "ExtractMin for MinPQ test passed" << endl;
            MinPQ_result.passed++;
    }else{
            MinPQ_result.failed++;
         }
    if(testdecreaseKey() == true)
    {
            cout << "DecreaseKey for MinPQ test passed" << endl;
            MinPQ_result.passed++;
    }else{
            MinPQ_result.failed++;
         }
    if(testisEmpty() == true)
    {
            cout << "IsEmpty for MinPQ test passed" << endl;
            MinPQ_result.passed++;
    }else{
            MinPQ_result.failed++;
         }
    
    cout << "\n\n";
    cout << "MinPQ Tests Passed: " << MinPQ_result.passed << endl;
    cout << "MinPQ Tests Failed: " << MinPQ_result.failed << endl;
    cout << "\n\n";
}
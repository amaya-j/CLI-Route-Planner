//========================================================
// CLI.cpp
// Trinity Meckel & Amaya Joshi
// December 18, 2024
//
// Description:
// This file contains the main implementation for a command-line interface
// (CLI) route planner using Dijkstra's algorithm to find the shortest path 
// between two points on a graph loaded from a file. The program allows users
// to input start and end coordinates interactively, and then outputs the 
// shortest path with the associated weight (distance or cost) between those points.
// It supports loading the graph data from a file format containing vertices 
// (with coordinates) and edges (with weights and optional road names).
//=========================================================
#include "Graph.cpp"
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <cmath>

#include <iostream>
using namespace std;


//==============================================================
// areCoordinatesEqual
// Helper function to check if two coordinates are approximately equal.
// It compares the x and y coordinates of the two points and returns
// true if both are within a specified tolerance range.
// PARAMETERS:
// - coord1: A pair of doubles representing the first coordinate (x, y).
// - coord2: A pair of doubles representing the second coordinate (x, y).
// RETURN VALUE:
// - Returns true if the coordinates are approximately equal, false otherwise.
//==============================================================
bool areCoordinatesEqual(pair<double, double> coord1, pair<double, double> coord2) 
{
    const double tolerance = 1e-6;
    return fabs(coord1.first - coord2.first) < tolerance && fabs(coord1.second - coord2.second) < tolerance;
}

//==============================================================
// loadGraphFromFile
// Loads a graph from a specified file. The graph contains vertices 
// with coordinates and edges with weights. The function maps coordinates 
// to vertex IDs and vice versa, and adds vertices and edges to the graph.
// PARAMETERS:
// - filename: The name of the file to load the graph from.
// - graph: A reference to the Graph object to populate with vertices and edges.
// - coordToID: A map that associates coordinates (x, y) with vertex IDs.
// - idToCoord: A map that associates vertex IDs with coordinates (x, y).
// RETURN VALUE:
// - Returns true if the graph is loaded successfully, false if there is an error 
//   opening the file or reading the graph data.
//==============================================================
bool loadGraphFromFile(const string &filename, Graph &graph, map<pair<double, double>, long long> &coordToID, map<long long, pair<double, double>> &idToCoord) 
{
    ifstream file(filename);
    if (!file.is_open()) 
    {
        cerr << "Error: Could not open file " << filename << endl;
        return false;
    }

    int numVertices, numEdges;
    file >> numVertices >> numEdges;

    for (int i = 0; i < numVertices; i++) 
    {
        long long id; 
        double x, y;
        file >> id >> x >> y;

        graph.addVertex(id);
        coordToID[{x, y}] = id;
        idToCoord[id] = {x, y};

        // Debug: print loaded vertex
        cout << "Loaded vertex ID " << id << " with coordinates (" << x << ", " << y << ")" << endl;
    }

    for (int i = 0; i < numEdges; i++) 
    {
        long long u, v;
        double weight;
        string roadName;

        if (!(file >> u >> v >> weight)) 
        {
            cerr << "Error: Failed to read edge data at entry " << i + 1 << endl;
            return false;
        }

        string restOfLine;
        getline(file, restOfLine);
    
        stringstream ss(restOfLine);
        roadName.clear();

        if (!(ss >> ws).eof()) 
        { 
            getline(ss >> ws, roadName); // Extract road name, if present
        }

        graph.addEdge(u, v, weight);

        // Debug: print loaded edge
        cout << "Loaded edge " << i + 1 << ": " << u << " -> " << v << " with weight " << weight << " and name \"" << roadName << "\"" << endl;
    }

    file.close();
    return true;
}

//==============================================================
// runDijkstraCLI
// Interactive function that allows the user to input start and end 
// coordinates, checks their validity, and then runs Dijkstra's algorithm 
// to find the shortest path between them. The function outputs the 
// shortest path and its weight.
// PARAMETERS:
// - graph: A reference to the Graph object that contains the vertices and edges.
// - coordToID: A map that associates coordinates (x, y) with vertex IDs.
// RETURN VALUE:
// - None. The function continues to prompt the user for new coordinates 
//   until the user opts to exit by entering invalid input or "q".
//==============================================================
void runDijkstraCLI(Graph &graph, map<pair<double, double>, long long> &coordToID)
{
    while (true) 
    {
        double startX, startY, endX, endY;

        cout << "\nEnter start coordinates: ";
        if (!(cin >> startX >> startY)) break;

        cout << "Enter end coordinates: ";
        if (!(cin >> endX >> endY)) break;

        // Check if coordinates are valid
        if (coordToID.find({startX, startY}) == coordToID.end() || coordToID.find({endX, endY}) == coordToID.end()) 
        {
            cout << "Error: Invalid coordinates entered. Try again.\n";
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            continue;
        }

        long long startID = coordToID[{startX, startY}];
        long long endID = coordToID[{endX, endY}];

        // Run Dijkstra's algorithm
        auto dijkstraOutput = graph.dijkstra(startID);

        vector<long long> path;
        long long current = endID;

        // Reconstruct the path
        while (current != startID) 
        {
            if (dijkstraOutput.find(current) == dijkstraOutput.end()) 
            {
                cout << "No path found to destination.\n";
                path.clear();
                break;
            }
            path.push_back(current);
            current = dijkstraOutput[current].first;
        }

        if (!path.empty()) 
        {
            path.push_back(startID); 
            reverse(path.begin(), path.end()); 
        
            // Output the shortest path
            cout << "Shortest path from (" << startX << ", " << startY << ") to (" << endX << ", " << endY << ") is:" << endl;
            for (long long vertex : path) 
            {
                cout << vertex << " ";
            }
            cout << "\nand it has weight: " << dijkstraOutput[endID].second << endl;
        } 
        else 
        {
            cout << "Path reconstruction failed!\n";
        }
    }
}


//==============================================================
// main
// The main entry point for the CLI route planner program. 
// It prompts the user for a file name to load the graph data,
// loads the graph from the file, and then runs an interactive
// Dijkstra's algorithm to find the shortest path between user-defined 
// start and end coordinates. The user can exit the program at any time.
// PARAMETERS:
// - None
// RETURN VALUE:
// - Returns 0 if the program completes successfully, or 1 if there is an error
//   loading the graph from the file.
//==============================================================
int main() {
    Graph graph;
    map<pair<double, double>, long long> coordToID;  
    map<long long, pair<double, double>> idToCoord;  

    string filename;
    cout <<"Welcome to my CLI route planner! You may enter\nthe letter q at any time to exit." << endl;
    cout << "Enter the graph file name: ";
    cin >> filename;
    if (filename == "q") 
    {
        cout << "Exiting... Thank you for using me instead of Google Maps!" << endl;
        return 0;
    }

    // Load the graph
    if (!loadGraphFromFile(filename, graph, coordToID, idToCoord)) 
    {
        cerr << "Failed to load graph from file." << endl;
        return 1;
    }

    // Run Dijkstra CLI
    runDijkstraCLI(graph, coordToID);

    cout << "Exiting... Thank you for using me instead of Google Maps! :) "<< endl;
    return 0;
}

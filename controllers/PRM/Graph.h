//
// Created by linor on 11/12/2021.
//

#include <map>
#include <cstdio>
#include <iostream>
#include <list>
#include <vector>
#include "Kdtree.h"
using namespace std;

class Graph
{
    int V; // No. of vertices
    // Pointer to an array containing adjacency
    // lists
    list<int> *adj;

public:
    Graph(int V); // Constructor
    // function to add an edge to graph
    void addEdge(int v, int w);
    // prints BFS traversal from a given source s
    bool BFS(int src, int dest, int v, int pred[], int dist[]);
    vector<int> shortest_path_using_BFS(int s, int dest);
};

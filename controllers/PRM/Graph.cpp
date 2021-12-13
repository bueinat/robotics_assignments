//
// Created by linor on 11/12/2021.
//

#include "Graph.h"
#include "Kdtree.h"
// #include "main.h"
// #include "krembot.ino.h"

void print_nodes(const KdNodeVector &nodes) {
    size_t i,j;
    for (i = 0; i < nodes.size(); ++i) {
        if (i > 0)
            cout << " ";
        cout << "(";
        for (j = 0; j < nodes[i].point.size(); j++) {
            if (j > 0)
                cout << ",";
            cout << nodes[i].point[j];
        }
        cout << ")";
    }
    cout << endl;
}

void print_float_vector(vector<float> const &vec){
    for (int i = 0; i < vec.size(); i++) {
        std::cout << vec.at(i) << ' ';
    }
}

Graph::Graph(int V)
{
    this->V = V;
    adj = new list<int>[V];
}

void Graph::addEdge(int v, int w)
{
    adj[v].push_back(w); // Add w to v’s list.
}
// todo to add a destination node
void Graph::BFS(map<int, vector<float>> mymap, int s,int d)
{
    //vector<KdNode> bfsPoints;
    KdNodeVector bfsPoints;
    // Mark all the vertices as not visited
    bool *visited = new bool[V];
    for(int i = 0; i < V; i++)
        visited[i] = false;

    // Create a queue for BFS
    list<int> queue;
    // Mark the current node as visited and enqueue it
    visited[s] = true;
    queue.push_back(s);
    // 'i' will be used to get all adjacent
    // vertices of a vertex
    list<int>::iterator i;
    //cout << mymap.size() << endl;
    for(map<int,std::vector<float>>::const_iterator it = mymap.begin();
        it != mymap.end(); ++it)
    {
        std::cout << it->first << "\n";
        // print_float_vector(it->second);
        cout << endl;
    }
    while(!queue.empty())
    {
        // Dequeue a vertex from queue and print it
        s = queue.front();
        //print_float_vector(new_keys_map[s]);
        cout << s << endl;
        bfsPoints.emplace_back(mymap[s]);
        queue.pop_front();
        // Get all adjacent vertices of the dequeued
        // vertex s. If a adjacent has not been visited,
        // then mark it visited and enqueue it
        for (i = adj[s].begin(); i != adj[s].end(); ++i)
        {
            if (!visited[*i])
            {
                visited[*i] = true;
                queue.push_back(*i);
            }
        }
    }
    //print the points of
    // print_nodes(bfsPoints);
}

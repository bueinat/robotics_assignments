#include "Graph.h"
#include <bits/stdc++.h>
using namespace std;

void Graph::print_nodes(const KdNodeVector &nodes) {
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

void Graph::print_float_vector(vector<float> const &vec) {
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
    adj[w].push_back(v);
}

// utility function to form edge between two vertices
// source and dest
void add_edge(vector<int> adj[], int src, int dest)
{
	adj[src].push_back(dest);
	adj[dest].push_back(src);
}

// a modified version of BFS that stores predecessor
// of each vertex in array p
// and its distance from source in array d
// this code was taken from here: https://www.geeksforgeeks.org/shortest-path-unweighted-graph/
bool Graph::BFS(int src, int dest, int v, int pred[], int dist[])
{
    list<int> queue;
	bool visited[v];
	for (int i = 0; i < v; i++) {
		visited[i] = false;
		dist[i] = INT_MAX;
		pred[i] = -1;
	}

	visited[src] = true;
	dist[src] = 0;
	queue.push_back(src);

	// standard BFS algorithm
	while (!queue.empty()) {
		int u = queue.front();
		queue.pop_front();

        list<int>::iterator i;
        for (i = this->adj[u].begin(); i != this->adj[u].end(); ++i)
        {
            if (!visited[*i])
            {
                visited[*i] = true;
                dist[*i] = dist[u] + 1;
				pred[*i] = u;
                queue.push_back(*i);

                // We stop BFS when we find destination.
				if (*i == dest)
					return true;
            }
        }
	}

	return false;
}

// utility function to print the shortest distance
// between source vertex and destination vertex
vector<int> Graph::shortest_path_with_BFS(int s, int dest)
{
	// predecessor[i] array stores predecessor of
	// i and distance array stores distance of i from s
    int v = this->V;
	int pred[v], dist[v];

	if (this->BFS(s, dest, v, pred, dist) == false) {
		cout << "Given source and destination"
			<< " are not connected\n";
		return {};
	}

	// vector path stores the shortest path
	vector<int> path;
	int crawl = dest;
	path.push_back(crawl);
	while (pred[crawl] != -1) {
		path.push_back(pred[crawl]);
		crawl = pred[crawl];
	}

	// distance from source is in distance array
	cout << "Shortest path length is : "
		<< dist[dest];

	// printing path from source to destination
	cout << "\nPath is:\n";
	for (int i = path.size() - 1; i >= 0; i--)
		cout << path[i] << " ";

    return path;
}

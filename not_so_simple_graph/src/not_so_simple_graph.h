/*
    Not so simple Graph

    Write C++ code for implementing a Graph data structure that supports a directed graph with
    self-loops and parallel edges. You are expected to implement the following methods and a main
    method is already built for you:

        void *insertEdge*(int from, int to, int weight);   // 1
        bool *isEdge*(int from, int to);                   // 2
        int *sumEdge*(int from, int to);                   // 3
        vector<int> *getWeight*(int from, int to);         // 4
        vector<int> *getAdjacent*(int vertex);             // 5


    Sample Input:
        7
        1 0 0 10
        1 0 1 20
        1 0 2 30
        2 0 0
        3 0 2
        4 0 1
        5 0

    Sample Output:
        1
        30
        20
        0 1 2
*/

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <stack>
#include <queue>
#include <algorithm>
using namespace std;
class Graph
{
    struct Edge {
        int src, dest, weight;
    };

private:
    // TODO: Graph structure that supports parallel edges and self-loops
    map<int, vector<pair<int, int>>> graph;
    vector<int> previous;
    vector<Edge> edges;
    
    int numOfNodes = 0;
    int numOfEdges = 0;
public:
    void insertEdge(int from, int to, int weight);
    bool isEdge(int from, int to);
    int sumEdge(int from, int to);
    std::vector<int> getWeight(int from, int to);
    std::vector<int> getAdjacent(int vertex);
    vector<int> shortestPath(int src);
    vector<int> bellmanFord(int src);
    //vector<int> actualShortest(int src);
};

vector<int> Graph::shortestPath(int src) {
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    int numOfNodes = graph.size();
    vector<int> distances(numOfNodes, 999999999);
    vector<int> p(numOfNodes, -1);

    pq.push(make_pair(0, src)); // sorted by distance so vertex label is in the second position
    distances[src] = 0;

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        // get adjacent vetrices of the vertex
        vector<pair<int, int>> adjacents = graph[u];
        for (auto it = adjacents.begin(); it != adjacents.end(); it++) {
            int vertex = it->first;
            int weight = it->second;

            if (distances[vertex] > distances[u] + weight) {
                distances[vertex] = distances[u] + weight;
                p[vertex] = u;
                pq.push(make_pair(distances[vertex], vertex));
            }
        }
    }
    previous = p;
    return distances;
}

vector<int> Graph::bellmanFord(int src) {
    numOfNodes = graph.size();
    vector<int> distances(numOfNodes, 999999999);
    vector<int> p(numOfNodes, -1);
    int min = 999999999;
    distances[src] = 0;
    for (auto it = graph[src].begin(); it != graph[src].end(); it++) {
        if (it->first == src && it->second < min) {
            distances[src] = it->second;
            min = it->second;
        }
    }

    for (int i = 1; i <= numOfNodes - 1; i++) {
        for (int j = 0; j < numOfEdges; j++) { // go to each node
            int u = edges[j].src; // source
            int v = edges[j].dest; // destination
            int weight = edges[j].weight;

            //cout << edges[j].src << " " << edges[j].dest << " " << edges[j].weight << "\n";
            
            if (distances[u] + weight < distances[v] && distances[u] != 999999999) {
                distances[v] = distances[u] + weight;
            }
        }
    }

    for (int i = 0; i < numOfEdges; i++) {
        int u = edges[i].src;
        int v = edges[i].dest;
        int weight = edges[i].weight;
        if (distances[u] != 999999999 && distances[u] + weight < distances[v]) {
            printf("Graph contains negative weight cycle");
            return vector<int>(); // If negative cycle is detected, simply
                    // return
        }
    }
    return distances;
}

//vector<int> Graph::actualShortest(int src) {
//}

void Graph::insertEdge(int from, int to, int weight)
{
    /*
         TODO: insertEdge() adds a new edge between the from
         and to vertex.
    */
    if (weight != 0) {
        numOfEdges++;
        graph[from].push_back(make_pair(to, weight));
        Edge edge;
        edge.src = from;
        edge.dest = to;
        edge.weight = weight;
        edges.push_back(edge);
        if (graph.find(to) == graph.end()) {
            graph[to] = {};
        }
    }

}

bool Graph::isEdge(int from, int to)
{
    /*
        TODO: isEdge() returns a boolean indicating true
        if there is an edge between the from and to vertex
    */
    auto edges = graph[from];
    for (int i = 0; i < edges.size(); i++) {
        if (edges[i].first == to) {
            return true;
        }
    }

    return false;
}

int Graph::sumEdge(int from, int to)
{
    /*
        TODO: sumEdge() returns the sum of weights of all edges
        connecting the from and to vertex. Returns 0 if no edges
        connect the two vertices.
    */
    //vector<int> distances = bellmanFord(from);
    //int distance = distances[to];
    //if (distance == 999999999) {
    //    return 0;
    //}
    //return distance;
    int distance = 0;
    for (int i = 0; i < edges.size(); i++) {
        if (edges[i].src == from && edges[i].dest == to) {
            distance += edges[i].weight;
        }
    }
    return distance;
}

std::vector<int> Graph::getWeight(int from, int to)
{
    /*
        TODO: getWeight() returns a sorted vector containing all
        weights of the edges connecting the from and to vertex
    */
    stack<int> s;
    vector<int> weights;
    auto distances = shortestPath(from); // to get our distance values for the stack
    int curr = to;
    while (curr != from) {
        s.push(curr);
        curr = previous[curr]; // this gets all values but "from"
    }

    s.push(from);

    for (int i = 0; i < s.size(); i++) { // we want the weight between the curr and previous node
        auto vect = graph[s.top()]; // vector of adjacents
        s.pop();
        for (int j = 0; j < vect.size(); j++) {
            if (vect[j].first == s.top()) {
                weights.push_back(vect[j].second);
            }
        }
    }

    return weights;
}

std::vector<int> Graph::getAdjacent(int vertex)
{
    /*
        TODO: getAdjacent() returns a sorted vector of all vertices
        that are connected to a vertex
    */
    auto adjacentPairs = graph[vertex];
    vector<int> adjacentKeys;
    for (int i = 0; i < adjacentPairs.size(); i++) {
        adjacentKeys.push_back(adjacentPairs[i].first);
    }

    if (adjacentPairs.size() == 0) {
        return vector<int>();
    }
    return adjacentKeys;
}

//DFS

#include <bits/stdc++.h>

using namespace std;

class Graph{
    map<int, vector<int>> adjList;

    public:

        void addEdge(int u, int v){
            adjList[u].push_back(v);
            adjList[v].push_back(u);
        }

        void DFS(int start){
            stack<int> s;
            map<int, bool> visited;

            s.push(start);
            visited[start] = true;

            while(!s.empty()){
                int node = s.top();
                s.pop();
                cout << node << " ";

                for(auto neighbour: adjList[node]){
                    if(!visited[neighbour]){
                        s.push(neighbour);
                        visited[neighbour] = true;
                    }
                }
            }
        }
};

int main(){
    Graph g;
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 2);
    g.addEdge(2, 3);
    g.addEdge(3, 3);

    g.DFS(2);

    return 0;
}


//BFS

#include <bits/stdc++.h>

using namespace std;

class Graph{
    map<int, vector<int>> adjList;

    public:

        void addEdge(int u, int v){
            adjList[u].push_back(v);
            adjList[v].push_back(u);
        }

        void BFS(int start){
            queue<int> q;
            map<int, bool> visited;

            q.push(start);
            visited[start] = true;

            while(!q.empty()){
                int node = q.front();
                q.pop();
                cout << node << " ";

                for(auto neighbour: adjList[node]){
                    if(!visited[neighbour]){
                        q.push(neighbour);
                        visited[neighbour] = true;
                    }
                }
            }
        }
};


int main(){
    Graph g;
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 2);
    g.addEdge(2, 3);
    g.addEdge(3, 3);

    g.BFS(2);

    return 0;
}



//Dijkstra's Algorithm

#include <bits/stdc++.h>

using namespace std;

class Graph{
    map<int, vector<pair<int, int>>> adjList;

    public:
    
            void addEdge(int u, int v, int w){
                adjList[u].push_back({v, w});
                adjList[v].push_back({u, w});
            }
    
            void dijkstra(int start){
                priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
                map<int, int> dist;
    
                for(auto node: adjList){
                    dist[node.first] = INT_MAX;
                }
    
                pq.push({0, start});
                dist[start] = 0;
    
                while(!pq.empty()){
                    int node = pq.top().second;
                    pq.pop();
    
                    for(auto neighbour: adjList[node]){
                        if(dist[neighbour.first] > dist[node] + neighbour.second){
                            dist[neighbour.first] = dist[node] + neighbour.second;
                            pq.push({dist[neighbour.first], neighbour.first});
                        }
                    }
                }
    
                for(auto node: dist){
                    cout << node.first << " " << node.second << endl;
                }
            }
}


//Bellman-Ford Algorithm

#include <bits/stdc++.h>

using namespace std;
#include <iostream>
#include <vector>
#include <climits>
using namespace std;

class Graph {
    vector<int> source, destination, weight; // Separate vectors for edges
    int vertices;

public:
    Graph(int v) : vertices(v) {}

    void addEdge(int u, int v, int w) {
        source.push_back(u);
        destination.push_back(v);
        weight.push_back(w);
    }

    void bellmanFord(int start) {
        vector<int> dist(vertices, INT_MAX);
        dist[start - 1] = 0;

        // Relax edges (vertices - 1) times
        for (int i = 0; i < vertices - 1; ++i) {
            for (size_t j = 0; j < source.size(); ++j) {
                int u = source[j], v = destination[j], w = weight[j];
                if (dist[u - 1] != INT_MAX && dist[u - 1] + w < dist[v - 1]) {
                    dist[v - 1] = dist[u - 1] + w;
                }
            }
        }

        // Check for negative-weight cycles
        for (size_t j = 0; j < source.size(); ++j) {
            int u = source[j], v = destination[j], w = weight[j];
            if (dist[u - 1] != INT_MAX && dist[u - 1] + w < dist[v - 1]) {
                cout << "Negative-weight cycle detected" << endl;
                return;
            }
        }

        // Print shortest distances
        cout << "Shortest distances from source " << start << ":" << endl;
        for (int i = 0; i < vertices; ++i) {
            cout << start << " -> " << i + 1 << ": ";
            if (dist[i] == INT_MAX)
                cout << "Infinity" << endl;
            else
                cout << dist[i] << endl;
        }
    }
};

int main() {
    Graph g(3);
    g.addEdge(1, 2, 3);
    g.addEdge(2, 3, 4);
    g.addEdge(3, 1, -8);

    g.bellmanFord(1);

    return 0;
}


//Kruskal's Algorithm

#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

class Graph {
    vector<vector<int>> edges; // {weight, source, destination}
    int vertices;

    // Union-Find Helpers
    vector<int> parent, rank;

    int find(int v) {
        if (parent[v] != v)
            parent[v] = find(parent[v]);
        return parent[v];
    }

    void unionSets(int u, int v) {
        int rootU = find(u), rootV = find(v);
        if (rootU != rootV) {
            if (rank[rootU] > rank[rootV])
                parent[rootV] = rootU;
            else if (rank[rootU] < rank[rootV])
                parent[rootU] = rootV;
            else {
                parent[rootV] = rootU;
                rank[rootU]++;
            }
        }
    }

public:
    Graph(int v) : vertices(v), parent(v), rank(v, 0) {
        for (int i = 0; i < v; ++i)
            parent[i] = i;
    }

    void addEdge(int u, int v, int w) {
        edges.push_back({w, u - 1, v - 1});
    }

    void kruskalMST() {
        sort(edges.begin(), edges.end()); // Sort edges by weight
        vector<pair<int, int>> mstEdges;
        int mstCost = 0;

        for (auto &edge : edges) {
            int w = edge[0], u = edge[1], v = edge[2];
            if (find(u) != find(v)) {
                mstEdges.push_back({u + 1, v + 1});
                mstCost += w;
                unionSets(u, v);
            }
        }

        // Output the result
        cout << "MST edges:" << endl;
        for (auto &[u, v] : mstEdges)
            cout << "(" << u << ", " << v << ")" << endl;
        cout << "Total cost: " << mstCost << endl;
    }
};

int main() {
    Graph g(4);
    g.addEdge(1, 2, 4);
    g.addEdge(1, 3, 3);
    g.addEdge(2, 3, 2);
    g.addEdge(2, 4, 5);

    g.kruskalMST();

    return 0;
}


//Prim's Algorithm

#include <iostream>
#include <vector>
#include <climits>
using namespace std;

void primMST(vector<vector<int>> &graph, int V) {
    vector<int> key(V, INT_MAX);
    vector<bool> mstSet(V, false);
    vector<int> parent(V, -1);
    key[0] = 0;

    for (int count = 0; count < V - 1; ++count) {
        int minKey = INT_MAX, u;
        for (int v = 0; v < V; ++v)
            if (!mstSet[v] && key[v] < minKey)
                minKey = key[v], u = v;

        mstSet[u] = true;
        for (int v = 0; v < V; ++v)
            if (graph[u][v] && !mstSet[v] && graph[u][v] < key[v])
                key[v] = graph[u][v], parent[v] = u;
    }

    int totalCost = 0;
    cout << "MST edges: ";
    for (int i = 1; i < V; ++i) {
        cout << "(" << parent[i] + 1 << ", " << i + 1 << ") ";
        totalCost += graph[i][parent[i]];
    }
    cout << "\nTotal cost: " << totalCost << endl;
}

int main() {
    vector<vector<int>> graph = {
        {0, 5, 6, 0},
        {5, 0, 2, 0},
        {6, 2, 0, 7},
        {0, 0, 7, 0}};
    primMST(graph, 4);
    return 0;
}


// //Write a C++ program to detect whether an undirected graph contains a cycle
// or not using Depth First Search (DFS). The program should:
// a. Use an adjacency list to represent the graph.
// b. Return true if a cycle is detected, otherwise false.
// Example:
// Input:
// Graph: {(1, 2), (2, 3), (3, 1)}
// Output:
// Cycle detected: true


#include <bits/stdc++.h>
using namespace std;

class Graph {
    map<int, vector<int>> adjList;

    bool isCyclicUtil(int v, vector<bool> &visited, int parent) {
        visited[v] = true;
        for (int i : adjList[v]) {
            if (!visited[i]) {
                if (isCyclicUtil(i, visited, v))
                    return true;
            } else if (i != parent)
                return true;
        }
        return false;
    }

public:

    void addEdge(int u, int v) {
        adjList[u].push_back(v);
        adjList[v].push_back(u);
    }

    bool isCyclic() {
        vector<bool> visited(adjList.size(), false);
        for (auto i : adjList) {
            if (!visited[i.first] && isCyclicUtil(i.first, visited, -1))
                return true;
        }
        return false;
    }
};


int main() {
    Graph g;
    g.addEdge(1, 2);
    g.addEdge(2, 3);
    g.addEdge(3, 1);

    if (g.isCyclic()) {
        cout << "Cycle detected: true" << endl;
    } else {
        cout << "Cycle detected: false" << endl;
    }

    return 0;
}

// Write a C++ program to perform Topological Sort of a Directed Acyclic Graph
// (DAG) using DFS. The program should:
// a. Use an adjacency list to represent the graph.
// b. Print the topological order of the vertices.
// Example:
// Input:
// Graph: {(1, 2), (2, 3), (1, 3)}
// Output:
// Topological order: 1, 2, 3


#include <bits/stdc++.h>

using namespace std;


class Graph {
    map<int, vector<int>> adjList;

    void topologicalSortUtil(int v, vector<bool> &visited, stack<int> &s) {
        visited[v] = true;
        for (int i : adjList[v]) {
            if (!visited[i])
                topologicalSortUtil(i, visited, s);
        }
        s.push(v);
    }

public:

    void addEdge(int u, int v) {
        adjList[u].push_back(v);
    }

    void topologicalSort() {
        stack<int> s;
        vector<bool> visited(adjList.size(), false);
        for (auto i : adjList) {
            if (!visited[i.first])
                topologicalSortUtil(i.first, visited, s);
        }
        cout << "Topological order: ";
        while (!s.empty()) {
            cout << s.top() << " ";
            s.pop();
        }
        cout << endl;
    }
};


int main() {
    Graph g;
    g.addEdge(1, 2);
    g.addEdge(2, 3);
    g.addEdge(1, 3);

    g.topologicalSort();

    return 0;
}

// Write a C++ program to check whether a given undirected graph is Bipartite
// or not using Breadth First Search (BFS). The program should:
// Use an adjacency list to represent the graph.
// Return true if the graph is bipartite, otherwise false.
// Example:
// Input:
// Graph: {(1, 2), (2, 3), (1, 3)}
// Output:
// Bipartite: false


#include <bits/stdc++.h>

using namespace std;

class Graph {
    map<int, vector<int>> adjList;

public:

    void addEdge(int u, int v) {
        adjList[u].push_back(v);
        adjList[v].push_back(u);
    }

    bool isBipartite() {
        vector<int> color(adjList.size(), -1);
        queue<int> q;
        for (auto i : adjList) {
            if (color[i.first] == -1) {
                color[i.first] = 1;
                q.push(i.first);
                while (!q.empty()) {
                    int u = q.front();
                    q.pop();
                    for (int v : adjList[u]) {
                        if (color[v] == -1) {
                            color[v] = 1 - color[u];
                            q.push(v);
                        } else if (color[v] == color[u])
                            return false;
                    }
                }
            }
        }
        return true;
    }
};


int main() {
    Graph g;
    g.addEdge(1, 2);
    g.addEdge(2, 3);
    g.addEdge(1, 3);

    if (g.isBipartite()) {
        cout << "Bipartite: true" << endl;
    } else {
        cout << "Bipartite: false" << endl;
    }

    return 0;
}


// Write a C++ program to print all the nodes reachable from a given starting
// node in a directed graph using the Breadth First Search (BFS) method. The
// program should:
// Use an adjacency list to represent the graph.
// Display all nodes reachable from the given start node.


#include <bits/stdc++.h>

using namespace std;

class Graph {
    map<int, vector<int>> adjList;

public:

    void addEdge(int u, int v) {
        adjList[u].push_back(v);
    }

    void BFS(int start) {
        queue<int> q;
        vector<bool> visited(adjList.size(), false);
        q.push(start);
        visited[start] = true;
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            cout << u << " ";
            for (int v : adjList[u]) {
                if (!visited[v]) {
                    q.push(v);
                    visited[v] = true;
                }
            }
        }
        cout << endl;
    }
};


int main() {
    Graph g;
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 2);
    g.addEdge(2, 3);
    g.addEdge(3, 3);

    g.BFS(2);

    return 0;
}
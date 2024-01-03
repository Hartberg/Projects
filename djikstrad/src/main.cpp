// Dijkstra's Algorithm in C++
// C++ program for Dijkstra's single source shortest path
// algorithm. The program is for adjacency matrix
// representation of the graph
//using namespace std;
//#include <limits.h>
#include <Arduino.h>
#include "Zumo32U4.h"

// Number of vertices in the graph
#define V 17

int currentVerticy; // nåværende plassering
int nodeList[17];  // liste med punkter til mål

Zumo32U4ButtonA buttonA;

int graph[V][V] = {{0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                     {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                     {0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                     {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
                     {1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                     {0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0},
                     {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 0},
                     {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1},
                     {0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0},
                     {0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1, 0, 0},
                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0},
                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0},
                     {0, 0, 0, 1, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0}};

// A utility function to find the vertex with minimum
// distance value, from the set of vertices not yet included
// in shortest path tree
int minDistance(int dist[], bool sptSet[])
{

  // Initialize min value
  int min = 10000, min_index;

  for (int v = 0; v < V; v++)
    if (sptSet[v] == false && dist[v] <= min)
      min = dist[v], min_index = v;

  return min_index;
}

// A utility function to print the constructed distance
// array
void printSolution(int dist[])
{
  Serial.println("this is the shortest path ");
  for (int i = 0; i < V; i++) {
    Serial.print(i);
    Serial.print("    ");
    Serial.print(dist[i]);
    Serial.println(" ");
  }
}

// Function that implements Dijkstra's single source
// shortest path algorithm for a graph represented using
// adjacency matrix representation
void dijkstra(int graph[V][V], int src)
{
  int dist[V]; // The output array. dist[i] will hold the
               // shortest
  // distance from src to i

  bool sptSet[V]; // sptSet[i] will be true if vertex i is
                  // included in shortest
  // path tree or shortest distance from src to i is
  // finalized

  // Initialize all distances as INFINITE and stpSet[] as
  // false
  for (int i = 0; i < V; i++)
    dist[i] = 10000, sptSet[i] = false;

  // Distance of source vertex from itself is always 0
  dist[src] = 0;

  // Find shortest path for all vertices
  for (int count = 0; count < V - 1; count++)
  {
    // Pick the minimum distance vertex from the set of
    // vertices not yet processed. u is always equal to
    // src in the first iteration.
    int u = minDistance(dist, sptSet);

    // Mark the picked vertex as processed
    sptSet[u] = true;

    // Update dist value of the adjacent vertices of the
    // picked vertex.
    for (int v = 0; v < V; v++)

      // Update dist[v] only if is not in sptSet,
      // there is an edge from u to v, and total
      // weight of path from src to v through u is
      // smaller than current value of dist[v]
      if (!sptSet[v] && graph[u][v] && dist[u] != 100000 && dist[u] + graph[u][v] < dist[v])
        dist[v] = dist[u] + graph[u][v];
  }
  
  // print the constructed distance array
  printSolution(dist);
}

// driver's code
int mains()
{

  /* Let us create the example graph discussed above */
  

  // Function call
  dijkstra(graph, 5);
  

  return 0;
}

// Define a structure to store node information including parent
struct NodeInfo {
  int distance; // Distance from the source node
  int parent;   // Parent node in the shortest path
};

// Function to print the shortest path from the source to the destination
void printPath(int parent[], int j) {
  if (parent[j] == -1) {
    Serial.print(j);
    return;
  }

  printPath(parent, parent[j]);
  Serial.print(" -> ");
  Serial.print(j);
}

// Function implementing Dijkstra's algorithm with path reconstruction
void dijkstraWithPath(int graph[V][V], int src, int dest) {
  int dist[V];     // Output array. dist[i] will hold the shortest distance from src to i
  bool sptSet[V];  // Will be true if vertex i is included in the shortest path tree

  NodeInfo nodeInfo[V]; // Information about each node (distance and parent)
  
  // Initialize distances as INFINITE and sptSet[] as false
  for (int i = 0; i < V; i++) {
    dist[i] = 10000;
    sptSet[i] = false;
    nodeInfo[i].distance = 10000;
    nodeInfo[i].parent = -1;
  }

  // Distance of source vertex from itself is always 0
  dist[src] = 0;
  nodeInfo[src].distance = 0;

  // Find shortest path for all vertices
  for (int count = 0; count < V - 1; count++) {
    int u = -1; // Initialize u as -1
    
    // Find the vertex with the minimum distance from the set of vertices not yet processed
    for (int v = 0; v < V; v++) {
      if (!sptSet[v] && (u == -1 || dist[v] < dist[u])) {
        u = v;
      }
    }

    // Mark the picked vertex as processed
    sptSet[u] = true;

    // Update distances of the adjacent vertices of the picked vertex
    for (int v = 0; v < V; v++) {
      if (!sptSet[v] && graph[u][v] && dist[u] != 10000 && dist[u] + graph[u][v] < dist[v]) {
        dist[v] = dist[u] + graph[u][v];
        nodeInfo[v].distance = dist[v];
        nodeInfo[v].parent = u;
      }
    }
  }

  // Print the shortest path from src to dest
  Serial.print("Shortest path from ");
  Serial.print(src);
  Serial.print(" to ");
  Serial.print(dest);
  Serial.print(": ");
  printPath(nodeInfo, dest);
}


void setup()
{
  Serial.begin(9600);
}

void loop()
{
  if (buttonA.getSingleDebouncedPress())
  {
    //mains();
    dijkstraWithPath(graph, 0, 9);
    
  }
}

// This code is contributed by shivanisinghss2110

/*
{{0, 1, 2, 2, 1, 2, 5, 2, 3, 4, 3, 4, 5, 6, 5, 3, 4},
{1, 0, 1, 3, 2, 2, 6, 3, 3, 5, 4, 5, 6, 7, 6, 4, 5},
{2, 1, 0, 3, 2, 1, 6, 3, 2, 5, 4, 5, 6, 7, 6, 4, 5},
{2, 3, 3, 0, 1, 2, 3, 2, 3, 3, 3, 4, 4, 5, 5, 1, 2},
{1, 2, 2, 1, 0, 1, 4, 1, 2, 3, 2, 3, 4, 5, 4, 2, 3},
{2, 2, 1, 2, 1, 0, 5, 2, 1, 4, 3, 4, 5, 6, 5, 3, 4},
{5, 6, 6, 3, 4, 5, 0, 5, 6, 4, 5, 7, 5, 6, 7, 2, 3},
{2, 3, 3, 2, 1, 2, 5, 0, 3, 2, 1, 2, 3, 4, 3, 3, 3},
{3, 3, 2, 3, 2, 1, 6, 3, 0, 5, 4, 5, 6, 7, 6, 4, 5},
{4, 5, 5, 3, 3, 4, 4, 2, 5, 0, 1, 4, 1, 2, 3, 2, 1},
{3, 4, 4, 3, 2, 3, 5, 1, 4, 1, 0, 3, 2, 3, 4, 3, 2},
{4, 5, 5, 4, 3, 4, 7, 2, 5, 4, 3, 0, 3, 2, 1, 5, 5},
{5, 6, 6, 4, 4, 5, 5, 3, 6, 1, 2, 3, 0, 1, 2, 3, 2},
{6, 7, 7, 5, 5, 6, 6, 4, 7, 2, 3, 2, 1, 0, 1, 4, 3},
{5, 6, 6, 5, 4, 5, 7, 3, 6, 3, 4, 1, 2, 1, 0, 5, 4},
{3, 4, 4, 1, 2, 3, 2, 3, 4, 2, 3, 5, 3, 4, 5, 0, 1},
{4, 5, 5, 2, 3, 4, 3, 3, 5, 1, 2, 5, 2, 3, 4, 1, 0}}


{adjecancy
{{0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,}
{1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,}
{0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,}
{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,}
{1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,}
{0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,}
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0,}
{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 0,}
{0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,}
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1,}
{0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0,}
{0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1, 0, 0,}
{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0,}
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0,}
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0,}
{0, 0, 0, 1, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,}
{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0,}}
*/
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
int nodeList[V];  // liste med punkter til mål
int shortestPath[V];  // liste med alle noder i shortest path som 
int nrTurn = 1; // burkes til å iterere hvor man putter inn nodene i shortestPath[]

Zumo32U4ButtonA buttonA;

int graph[V][V] = {{0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                   {1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                   {0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                   {1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                   {0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0}, 
                   {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, 
                   {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0}, 
                   {0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}, 
                   {0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0}, 
                   {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0}, 
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0}, 
                   {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0}, 
                   {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}, 
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1}, 
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0}, 
                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0}};


// Define a structure to store node information including parent
struct NodeInfo {
  int distance; // Distance from the source node
  int parent;   // Parent node in the shortest path
};

// Function to print the shortest path from the source to the destination
void printPath(NodeInfo nodeInfo[], int j) {
  if (nodeInfo[j].parent == -1) {
    Serial.print(j);
    shortestPath[nrTurn] = j;
    nrTurn = 1; // resetter
    return;
  }

  printPath(nodeInfo, nodeInfo[j].parent);
  Serial.print(" -> ");
  Serial.print(j);
  
  shortestPath[nrTurn] = j; // legger til korteste vei i global array
  nrTurn ++;
}


// Function implementing Dijkstra's algorithm with path reconstruction
void dijkstraWithPath(int graph[V][V], int src, int dest) {
  int dist[V];     // Output array. dist[i] will hold the shortest distance from src to i
  bool sptSet[V];  // Will be true if vertex i is included in the shortest path tree
  shortestPath[0] = src; // setter startpunkt i globalarray 

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

  Serial.print("Shortest path from ");
  Serial.print(src);
  Serial.print(" to ");
  Serial.print(dest);
  Serial.print(": ");
  printPath(nodeInfo, dest);
  for (int i = 0; i < V; i++) {
    Serial.println(shortestPath[i]);
  }
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
    dijkstraWithPath(graph, 12, 8);
    
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

kjekkerstue
{{0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
{1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
{0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
{1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
{0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0}, 
{0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, 
{0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0}, 
{0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}, 
{0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0}, 
{0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0}, 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0}, 
{0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0}, 
{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0}, 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1}, 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0}, 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0}}

*/
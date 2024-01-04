// Dijkstra's Algorithm in C++
// C++ program for Dijkstra's single source shortest path
// algorithm. The program is for adjacency matrix
// representation of the graph
// using namespace std;
// #include <limits.h>
#include <Arduino.h>
#include "Zumo32U4.h"

// Number of vertices in the graph
#define V 17

// kjørevariabler
int position;                // bilens position ifrhold til linja 0-4000
unsigned int lineSensorArray[5]; // lager et array for bilen å sette tall inn i
int normalSpeed = 200;           // basis fart for

float lineMultiplier;

int currentVerticy;  // nåværende plassering
int nodeList[V];     // liste med punkter til mål
int shortestPath[V]; // liste med alle noder i shortest path som
int nrTurn = 1;      // burkes til å iterere hvor man putter inn nodene i shortestPath[]

unsigned long millisTimer; // timer til oled skjerm
unsigned long nodeDetectionCD; // samme som over

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4OLED oled;
Zumo32U4LineSensors lineSensors;
Zumo32U4Buzzer buzzer;

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
struct NodeInfo
{
  int distance; // Distance from the source node
  int parent;   // Parent node in the shortest path
};

// Function to print the shortest path from the source to the destination
void printPath(NodeInfo nodeInfo[], int j)
{
  if (nodeInfo[j].parent == -1)
  {
    Serial.print(j);
    shortestPath[nrTurn] = j;
    nrTurn = 1; // resetter
    return;
  }

  printPath(nodeInfo, nodeInfo[j].parent);
  Serial.print(" -> ");
  Serial.print(j);

  shortestPath[nrTurn] = j; // legger til korteste vei i global array
  nrTurn++;
}

// Function implementing Dijkstra's algorithm with path reconstruction
void dijkstraWithPath(int graph[V][V], int src, int dest)
{
  int dist[V];           // Output array. dist[i] will hold the shortest distance from src to i
  bool sptSet[V];        // Will be true if vertex i is included in the shortest path tree
  shortestPath[0] = src; // setter startpunkt i globalarray

  NodeInfo nodeInfo[V]; // Information about each node (distance and parent)

  // Initialize distances as INFINITE and sptSet[] as false
  for (int i = 0; i < V; i++)
  {
    dist[i] = 10000;
    sptSet[i] = false;
    nodeInfo[i].distance = 10000;
    nodeInfo[i].parent = -1;
  }

  // Distance of source vertex from itself is always 0
  dist[src] = 0;
  nodeInfo[src].distance = 0;

  // Find shortest path for all vertices
  for (int count = 0; count < V - 1; count++)
  {
    int u = -1; // Initialize u as -1

    // Find the vertex with the minimum distance from the set of vertices not yet processed
    for (int v = 0; v < V; v++)
    {
      if (!sptSet[v] && (u == -1 || dist[v] < dist[u]))
      {
        u = v;
      }
    }

    // Mark the picked vertex as processed
    sptSet[u] = true;

    // Update distances of the adjacent vertices of the picked vertex
    for (int v = 0; v < V; v++)
    {
      if (!sptSet[v] && graph[u][v] && dist[u] != 10000 && dist[u] + graph[u][v] < dist[v])
      {
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
  for (int i = 0; i < V; i++)
  {
    Serial.println(shortestPath[i]);
  }
}

// kalibrerrrer linjesensor
void calibrate()
{ // kalibrere sensorene
  oled.clear();
  oled.gotoXY(0, 0);
  oled.print("calibrating");
  delay(500);
  for (int i = 0; i < 200; i++)
  {
    motors.setSpeeds(180, -180); // kjør i sirkel
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
  oled.clear();
  oled.setLayout21x8();
  delay(50); // tid til motoren og stoppe helt før den starter å kjøre fremover
}

void readSensors()
{
  position = lineSensors.readLine(lineSensorArray);
}

// følger linje med p regulering
void lineFollowP()
{
  lineMultiplier = (map(position, 0, 4000, 100.0, -100.0) / 100.0);
  int speedLeft = normalSpeed * (1.0 - lineMultiplier);
  int speedRight = normalSpeed * (1.0 + lineMultiplier);
  motors.setSpeeds(speedLeft, speedRight);
  //oled.clear();
  //oled.gotoXY(0, 0);
  //oled.println(speedLeft);
  //oled.gotoXY(0, 1);
  //oled.println(speedRight);
}

void printSensor()
{
  if (millis() - millisTimer > 20)
  {
    oled.clear();
    oled.gotoXY(0, 0);
    oled.print(lineSensorArray[0]);
    oled.gotoXY(0,1);
    oled.print(lineSensorArray[1]);
    oled.gotoXY(0,2);
    oled.print(lineSensorArray[2]);
    oled.gotoXY(0,3);
    oled.print(lineSensorArray[3]);
    oled.gotoXY(0,4);
    oled.print(lineSensorArray[4]);
    oled.gotoXY(10,0);
    oled.print(position);

    millisTimer = millis();
  }
}

bool nodeDetect(){
  if ((lineSensorArray[1] + lineSensorArray[2] + lineSensorArray[3] > 1800) && (millis() - nodeDetectionCD > 500)) { 
    buzzer.playFrequency(440, 100, 10);
    
    return true;
  }
  return false;
}

void setup()
{
  Serial.begin(9600);
  lineSensors.initFiveSensors(); 
  calibrate();
}

void loop()
{
  readSensors();
  printSensor();
  if (nodeDetect()){
    delay(100);
    motors.setSpeeds(0,0);
    delay(500);
    nodeDetectionCD = millis();
  }
  lineFollowP();
  /*
  if (buttonA.getSingleDebouncedPress())
  {

    dijkstraWithPath(graph, 12, 8);
  }
  */
}

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


adjency list // nabo starter kl12 og følger klokken
0 {{3,1},
1  {2,0},
2  {1,4},
3  {6,0},
4  {7,8,2},
5  {8},
6  {7,3,12},
7  {4,6,9},
8  {5,4},
9  {10,7,13},
10 {14,9},
11 {15,8,14},
12 {6},
13 {9},
14 {11,10,16},
15 {11},
16 {14}}
 
*/
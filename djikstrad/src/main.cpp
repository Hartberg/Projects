// Dijkstra's Algorithm in C++
// C++ program for Dijkstra's single source shortest path
// algorithm. The program is for adjacency matrix
// representation of the graph
// using namespace std;
// #include <limits.h>
#include <Arduino.h>
#include <Zumo32U4.h>
#include <Wire.h>

// Number of vertices in the graph
#define V 17


// kjørevariabler
int position;                    // bilens position ifrhold til linja 0-4000
unsigned int lineSensorArray[5]; // lager et array for bilen å sette tall inn i
int normalSpeed = 150;           // basis fart for
float lineMultiplier;            // brukes i pid

// til djikstra 

int nodeList[V];     // liste med punkter til mål
int shortestPath[V]; // liste med alle noder i shortest path som
int nrTurn = 1;      // burkes til å iterere hvor man putter inn nodene i shortestPath[]

int currentNode;  // nåværende plassering
int startNode = 15; // punktet vi starter på. endres når vi endrer startpunkt på banen
int nextNode; // neste node vi skal til
int destination = 4; // første endepunkt

unsigned long millisTimer;     // timer til oled skjerm
unsigned long nodeDetectionCD; // samme som over
bool nodeDetectBool = false; // status om node er detected
bool atNode = false; // status om man er på node

int nextAngle; // vinkel i neste kryss

// midelertidig global for print
int currentIndex; 
bool setupgyroreset= false; // faen shit gyro

// gyro (ikke babb)
const int32_t turnAngle45 = 0x20000000;             // This constant represents a turn of 45 degrees.
const int32_t turnAngle90 = turnAngle45 * 2;        // This constant represents a turn of 90 degrees.
const int32_t turnAngle1 = (turnAngle45 + 22) / 45; // This constant represents a turn of approximately 1 degree.
uint32_t turnAngle = 0;
int16_t turnRate;            // vet ikke
int16_t gyroOffset;          // vet ikke
uint16_t gyroLastUpdate = 0; // no greier
int32_t turnDegree;          // brukes som global retningsvariable

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4OLED oled;
Zumo32U4LineSensors lineSensors;
Zumo32U4Buzzer buzzer;
Zumo32U4IMU imu;

//adj graph for map
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

//adj list for map -99 is fill
int adjList[V][3] = 
{{3,1,-99},    
 {2,0,-99},     
 {1,4, -99},     
 {6,0,-99},    
 {7,8,2},  
 {8,-99,-99},      //5
 {7,3,12}, 
 {4,6,9},  
 {11,5,4},  
 {10,7,13},  
 {14,9,-99},     //10
 {15,8,14},  
 {6,-99,-99},        
 {9,-99,-99},        
 {11,10,16},   
 {11,-99,-99},      //15
 {14,-99,-99}};

int degreeList[V][3] =
{{0,90,-1},
 {90,270,-1},
 {270,315,-1},
 {0,180,-1},
 {0,45,135},
 {0,-1,-1},   //5
 {90,180,270},
 {180,270,315},
 {0,180,225},
 {90,135,270},
 {45,270,-1}, //10
 {45,180,315},
 {90,-1,-1},
 {90,-1,-1},
 {135,225,270}, 
 {225,-1,-1}, //15
 {90,-1,-1}}; 

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
  for (int i = 0; i < 100; i++)
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
}

void printSensor()
{
  if (millis() - millisTimer > 20)
  { 
    /*
    oled.clear();
    oled.gotoXY(0, 0);
    oled.print(lineSensorArray[0]);
    oled.gotoXY(0, 1);
    oled.print(lineSensorArray[1]);
    oled.gotoXY(0, 2);
    oled.print(lineSensorArray[2]);
    oled.gotoXY(0, 3);
    oled.print(lineSensorArray[3]);
    oled.gotoXY(0, 4);
    oled.print(lineSensorArray[4]);
    oled.gotoXY(10, 0);
    oled.print(position);
    oled.gotoXY(10, 1);
    oled.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    oled.gotoXY(10, 2);
    oled.print(turnDegree);
    */
   oled.clear();
    oled.gotoXY(0, 0);
    for (int i = 0; i <9; i++) {
      oled.print(shortestPath[i]);
    }
     
    oled.gotoXY(0, 1);
    oled.print("currentNode: ");
    oled.print(currentNode);
    oled.gotoXY(0, 2);
    oled.print("nextAngle: ");
    oled.print(nextAngle);
    oled.gotoXY(0, 3);
    oled.print("next node: ");
    oled.print(nextNode);
    oled.gotoXY(0, 4);
    oled.print("currentIndex: ");
    oled.print(currentIndex);
    oled.gotoXY(0, 5);
    oled.print("nextnode+1: ");
    oled.print(shortestPath[currentIndex+2]);
    oled.gotoXY(0, 6);
    oled.print("angle: ");
    oled.print(turnDegree);
    millisTimer = millis();
  }
}

// detect node and wait 100ms
bool nodeDetect()
{
  if ((lineSensorArray[1] + lineSensorArray[2] + lineSensorArray[3] > 1800) && (millis() - nodeDetectionCD > 500))
  {
    buzzer.playFrequency(440, 100, 10);
    nodeDetectionCD = millis();
    nodeDetectBool = true; 
  }
  if (millis()- nodeDetectionCD > 100 && nodeDetectBool == true){
    nodeDetectBool = false;
    atNode = true;
    motors.setSpeeds(0,0); // spar hjul
    return true;

  }
  return false;
}

void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

void turnSensorUpdate()
{
  if (buttonB.getSingleDebouncedPress())
  {
    turnSensorReset();
  }
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
  turnDegree = ((((int32_t)turnAngle >> 16) * 360) >> 16);
  if (((((int32_t)turnAngle >> 16) * 360) >> 16) < 0)
  {
    turnDegree += 360;
  }
  turnDegree = (turnDegree - 360) * -1; // nå får vi fra 0 til 360 med klokka
}

void turnSensorSetup()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();
  oled.clear();
  oled.print(F("Gyro cal"));
  ledYellow(1);
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady())
    {
    }
    imu.readGyro();
    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;
  oled.clear();
  turnSensorReset();
}

void turnToAngle(int angle){
  if (angle - turnDegree > 5 || angle - turnDegree < -5) {
    motors.setSpeeds(130,-130);
  }
  else{
      motors.setSpeeds(0,0);
      currentNode = nextNode;
      atNode = false;
      // spar hjul
  }
}

void nextAngleNode(){
  currentIndex = 0; // finn neste node i listen
  for (int i = 0; i < 11; i++){
    if (currentNode == shortestPath[i]){
      currentIndex = i;
      break;
    }
  }
  nextNode = shortestPath[currentIndex+1];
 
  int nextAngleIndex = 0;  // finn neste vinkel
  for (int i = 0; i < 3; i++){
    if (adjList[shortestPath[currentIndex+1]][i] == shortestPath[currentIndex+2]) {
      nextAngleIndex = i;
      
      break;
    }
  }
  nextAngle = degreeList[shortestPath[currentIndex+1]][nextAngleIndex];

}

void driveTo(int to){
  nextAngleNode();
  if (atNode) {
    turnToAngle(nextAngle);
  }
  else{
    lineFollowP();
  }
  
}


void setup()
{
  Serial.begin(9600);
  lineSensors.initFiveSensors();
  turnSensorSetup();
  delay(100);
  calibrate();
  dijkstraWithPath(graph, startNode, destination);
  oled.gotoXY(0,1);
  oled.println("press A When");
  oled.println("facing N");
  while (!buttonA.getSingleDebouncedPress()) {}
  oled.clear();
  oled.gotoXY(0,1);
  oled.println("Press A to start");
  delay(500);
  buttonA.waitForPress();
  currentNode = startNode;
  nextAngleNode();
  delay(500);

  

}

void loop()
{
  readSensors();
  turnSensorUpdate();
  nodeDetect();
  printSensor();
 // turnToAngle(180);
  if (setupgyroreset == true) {
    driveTo(destination);
  }
  if (buttonA.getSingleDebouncedPress()){
    turnSensorReset();
    setupgyroreset = true;
  }
  
  

  if(currentNode == destination){
    motors.setSpeeds(0,0);
    buttonA.waitForPress();
  }
  
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
8  {11,5,4},
9  {10,7,13},
10 {14,9},
11 {15,8,14},
12 {6},
13 {9},
14 {11,10,16},
15 {11},
16 {14}}
grader fra/til
0 {{0,90},
1  {90,270},
2  {270,315},
3  {0,180},
4  {0,45,135},
5  {0},
6  {90,180,270},
7  {180,270,315},
8  {0,180,225},
9  {90,135,270},
10 {45,270},
11 {45,180,315},
12 {90},
13 {90},
14 {135,225,270},
15 {225},
16 {90}}



*/
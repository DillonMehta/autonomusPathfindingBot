#define INF 9999  //representation for infinity or no connection
String maze[5][4] = {
  { "", "R", "L", "" },
  { "", "BR", "BL", "" },
  { "B", "TR", "LT", "B" },
  { "T", "BR", "BL", "T" },
  { "", "T", "T", "" }        
};


int orientation = 2;
//T-0, R-1, B-2, L-3
int gateCount = 3;  //VERY IMPORTANT
int startNode = 1;
int G1 = 3;
int G2 = 2;
int G3 = 6;
int G4 = 6;
int G5 = 6;
// int G6 = 0;
int endNode = 3;

int endPoints[] = { startNode, G1,G2,G3, G4,G5, endNode };


//Susu ------------------------------------------------------------------------------------------------------------------------
#include <Wire.h>
#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>


/*
Include Pololu3piPlus32U4IMU.h in one of your cpp/ino files to
enable IMU functionality.
*/
#include <Pololu3piPlus32U4IMU.h>

using namespace Pololu3piPlus32U4;

OLED display;

Encoders encoders;
Buzzer buzzer;
Motors motors;
IMU imu;
//Button A buttonA;


int16_t maxSpeed = 100;

const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.05; 


//GYRO ------------------------------------------------------------------------------------------------------------------------
const int32_t turnAngle45 = 0x20000000;

// This constant represents a turn of 90 degrees.
const int32_t turnAngle90 = turnAngle45 * 2;

// This constant represents a turn of approximately 1 degree.
const int32_t turnAngle1 = (turnAngle45 + 22) / 45;
/*
Our convention is that a value of 0x20000000 represents a 45
degree counter-clockwise rotation.  This means that a uint32_t
can represent any angle between 0 degrees and 360 degrees.  If
you cast it to a signed 32-bit integer by writing
(int32_t)turnAngle, that integer can represent any angle between
-180 degrees and 180 degrees. */
uint32_t turnAngle = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;
//DRIVING ------------------------------------------------------------------------------------------------------------------------
double s1 = 100;
double s2 = 100;


//IR Sensor for Angular Velocity

volatile double eCount = 0;
volatile double eCount2 = 0;
volatile double dT = 0;
volatile double dT2 = 0;

void countL() {
  eCount= encoders.getCountsLeft();
  dT = eCount / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE;
  // Serial.println(dT);
}
void countR() {
  eCount2 = encoders.getCountsRight();
  dT2 = eCount2 / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE;
  // Serial.print("\t");
  // Serial.println(dT2);
}



//PATHFINDING
int currentDistance;  // THIS IS USED TO GLOBALIZE THE ITERATED VALUE FROM THE DISTANCE ARRAY


//DRIVING -------------------------------------------------------------------------------------------------------------------

//For every exit command in every move function, step must increment because the motion is inside of the loop
// UPDATE dRECORD EVERY TIME TOO
int step = 0;

void update() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  turnSensorReset();
  turnAngle=0;
  //dRecord2 = dT2;
  step++;
  
}

void go_fwd() {
  //debug
  //Serial.println("forward");
  int intent = currentDistance;
  turnSensorUpdate();
  countL();
  countR();
  
  if (eCount / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE >= intent) {
    motors.setSpeeds(0, 0);
    delay(1000);
    update();

  } else {
    motors.setSpeeds(s1, s2);
    // Serial.print(s1);
    // Serial.print("\t");
    // Serial.println(s2);
  }
}
void go_bck() {// ONLY OCCURS AT THE END SQUARE
  /*
  int intent = currentDistance;
  mpu6050.update();
  prevAngle = angle;
  angle = mpu6050.getAngleZ();
  if (dT - dRecord >= intent) {
    go_back(0, 0);
    delay(1000);
    update();

  } else {
    int angDif = round((angle - prevAngle) * 500);
    if (abs(angDif) > 1) {
      if (angDif > 0) {
        s2 += 1;
        s1 -= 1;
      } else if (angDif < 0) {
        s2 -= 1;
        s1 += 1;
      }
    }
    go_back(s1, s2);
    // Serial.print(s1);
    // Serial.print("\t");
    // Serial.println(s2);
  } */
}

void turn_left() {
  Serial.println("Turn Left");
  turnSensorUpdate();
  if ((((int32_t)turnAngle >> 16) * 360) >> 16 >= 90) {
    motors.setSpeeds(0,0);
    delay(1000);
    update();
  } else {
    // Serial.print(fAngle);
    // Serial.print("\t");
    // Serial.println(angle);
    motors.setSpeeds(-s1,s2);
  }
}

void turn_right() {
  Serial.println("Turn Right");
  turnSensorUpdate();
  if ((((int32_t)turnAngle >> 16) * 360) >> 16 <= -90) {
    motors.setSpeeds(0,0);
    delay(1000);
    update();
  } else {
    // Serial.print(fAngle);
    // Serial.print("\t");
    // Serial.println(angle);
    motors.setSpeeds(s1,-s2);
  }
}

void fullTurn() {  //will go counterclockwise
  Serial.println("FullTurn");
  turnSensorUpdate();
  if (turnAngle <= -turnAngle90) {
    motors.setSpeeds(0,0);
    delay(1000);
    update();
  } else {
    // Serial.print(fAngle);
    // Serial.print("\t");
    // Serial.println(angle);
    motors.setSpeeds(s1,-s2);
  }
}

void stop_Stop(){
  motors.setSpeeds(0,0);
}



//PATHFINDING --------------------------------------------------------------------------------------------------------------------

int functionCount = 1;
void (*functionArray[60])();  // LEFT OFF HERE
int distance[60];             //should be parallel to functionArray and will use the function Count index
int prevNode = -1;

void emptyFunction() {  //FILLING FUNCTION ARRAY WITH STOP VALUES
  functionArray[0] = go_fwd;
  distance[0] = 11.5+25;
  for (int i = 1; i < 60; i++) {
    functionArray[i] = stop_Stop;
    distance[i] = 45;//BASE DISTANCE
  }
}

void printPath(int parent[], int node) {
  if (parent[node] == -1) {
    Serial.println(node);
   // route[routeCount] = node;
    return;
  }
  printPath(parent, parent[node]);
  Serial.print(" --> ");
  Serial.println(node);

  //RECORDING NODE PATH TO FUNCTION STEPS
  Serial.println(node - prevNode);
  int distanceAdditive = 50; //can Add Calibration Distance 
  switch (node - prevNode) { 
    case -1: //left
      switch(orientation){
        case 0:
          functionArray[functionCount] = turn_left;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        case 1:
          functionArray[functionCount] = fullTurn;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        case 2:
          functionArray[functionCount] = turn_right;
          functionArray[functionCount+1] = go_fwd; 
          functionCount++;  
          break;
        case 3:
          //functionArray[functionCount] = ;     
          distance[functionCount-1] += distanceAdditive;      
          functionCount--;
          break;
        default:
          //Serial.println(node-prevNode);
          break;
      }
      orientation = 3; //left
      break;
    case 1: //right
      switch(orientation){
        case 0:
          functionArray[functionCount] = turn_right;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        case 1:
          //functionArray[functionCount] = ;     
          distance[functionCount-1] += distanceAdditive;    
          functionCount--;
          break;
        case 2:
          functionArray[functionCount] = turn_left;
          functionArray[functionCount+1] = go_fwd;  
          functionCount++; 
          break;
        case 3:
          functionArray[functionCount] = fullTurn;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        default:
          //Serial.println(node-prevNode);
          break;
      }
      orientation = 1; //right
      break;
    case -4://up
      switch(orientation){
        case 0:
          distance[functionCount-1] += distanceAdditive;    
          functionCount--;
          break;
        case 1:
          functionArray[functionCount] = turn_left;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        case 2:
          functionArray[functionCount] = fullTurn;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        case 3:
          functionArray[functionCount] = turn_right;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        default:
          //Serial.println(node-prevNode);
          break;
      }
      orientation = 0;
      break;
    case 4:
      switch(orientation){
        case 0:
          functionArray[functionCount] = fullTurn;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        case 1:
          functionArray[functionCount] = turn_right;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        case 2:
          distance[functionCount-1] += distanceAdditive;  
          functionCount--;
          break;
        case 3:
          functionArray[functionCount] = turn_left;
          functionArray[functionCount+1] = go_fwd;
          functionCount++;
          break;
        default:
          //Serial.println(node-prevNode);
          break;        
      }
      orientation = 2;
      break;
    default:
      //Serial.println(node-prevNode);
      break;
  }
  if (node == endPoints[gateCount + 1]){ //The End Square
    functionArray[functionCount+1] = go_bck;
    functionCount++;
    distance[functionCount] = 13;
  }
  prevNode = node;
  functionCount++;  //MOVED HERE SO THAT ANALYSIS COULD TAKE PLACE BEFORE INCREMENT
}

//Adjacency Matrix
bool adjMatrix[20][20];

void createMatrix() {
  //EMPTY MATRIX:
  for (int a = 0; a < 20; a++) {
    for (int b = 0; b < 20; b++) {
      adjMatrix[a][b] = 1;
    }
  }
  //OVERWRITING VALUES
  for (int a = 0; a < 20; a++) {
    int i = a / 4;
    int j = a % 4;
    String str = maze[i][j];
    int w = 0;                   //DEFAULT CONNECTION WEIGHT
    // if (str.indexOf('G') >= 0) {  //IF ITS A GATE ZONE
    //   w = -16;                    //GATE ZONE CONNECTION WEIGHT
    // }
    if (str.indexOf('T') < 0 && i > 0) {  //IF THERE IS NO TOP BARRIER && IS NOT IN THE TOP ROW
      adjMatrix[4 * (i - 1) + j][4 * i + j] = w;
      // if (maze[i - 1][j].indexOf('G') >= 0) {  //CHECK IF IT LEADS TO A GATE ZONE
      //   adjMatrix[4 * (i - 1) + j][4 * i + j] = -16;
      // }
    }
    if (str.indexOf('B') < 0 && i < 4) {  // IF THERE IS NO BOTTOM BARRIER && IS NOT IN THE BOTTOM ROW
      adjMatrix[4 * (i + 1) + j][4 * i + j] = w;
      // if (maze[i + 1][j].indexOf('G') >= 0) {  //CHECK IF IT LEADS TO A GATE ZONE
      //   adjMatrix[4 * (i + 1) + j][4 * i + j] = -16;
      // }
    }
    if (str.indexOf('L') < 0 && j > 0) {  //IF THERE IS NO LEFT BARRIER && IS NOT IN THE LEFT-MOST COLUMN
      adjMatrix[4 * i + j][4 * i + j - 1] = w;
      // if (maze[i][j - 1].indexOf('G') >= 0) {  //CHECK IF IT LEADS TO A GATE ZONE
      //   adjMatrix[4 * i + j][4 * i + j - 1] = -16;
      // }
    }
    if (str.indexOf('R') < 0 && j < 3) {  // IF THERE IS NO RIGHT BARRIER && IS NOT IN THE RIGHT MOST COLUMN
      adjMatrix[4 * i + j][4 * i + j + 1] = w;
      // if (maze[i][j + 1].indexOf('G') >= 0) {  //CHECK IF IT LEADS TO A GATE ZONE
      //   adjMatrix[4 * i + j][4 * i + j + 1] = -16;
      // }
    }
  }
  
  for (int a = 0; a < 20; a++) {
    for (int b = 0; b < 20; b++) {
      Serial.print(adjMatrix[a][b]);
      Serial.print("\t");
     }
     Serial.println("");//DEBUG CHECKING
   }
   
}

int hCost(int node) {
  return abs(node % 4 - endNode % 4) + abs(node / 4 - endNode / 4);
}

void Path() {
  for (int a = 1; a < gateCount + 2; a++) {
    endNode = endPoints[a];
    startNode = endPoints[a - 1];
    int parent[20];  // Array to store the path
    for (int i = 0; i < 20; i++) {
      parent[i] = -1;  // Initialize all nodes as unvisited
    }


    int currentNode = startNode;
    bool openSet[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    bool closedSet[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    int fCostList[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    openSet[startNode] = 1;
    //route[routeCount-1] = startNode;
    prevNode = startNode;
    while (currentNode != endNode) { 
      int minCost = INF;
      for (int i = 0; i < 20; i++) {  //SETTING CURRENT TO NODE IN OPENSET WITH THE LOWEST F COST
        if (openSet[i]) {
          if (fCostList[i] <= minCost) {
            minCost = fCostList[i];
            currentNode = i;
            //Serial.println(minCost);
          }
        }
      }
      openSet[currentNode] = 0;    //removing current from openSet
      closedSet[currentNode] = 1;  //adding current to closedSet

      if (currentNode == endNode) {
        break;
      }
      //TRAVERSE NEIGHBORS AND CHECK IF THEY ARE IN CLOSED; THEN CHECK IF THE NEW PATH IS
      if (currentNode > 3) {                                                                                        //TOP
        //Serial.println("got here 1");
        if (!(closedSet[currentNode - 4]) && adjMatrix[currentNode][currentNode - 4] < 1) {                         //if neighbor is NOT in closed OR IS traversable
          Serial.println("got here2");
          if (adjMatrix[currentNode][currentNode - 4] < fCostList[currentNode - 4] || !openSet[currentNode - 4]) {  //if new path to neighbor IS shorter OR neighbor is NOT in OPEN
            fCostList[currentNode - 4] = adjMatrix[currentNode][currentNode - 4] + hCost(currentNode - 4);
            parent[currentNode - 4] = currentNode;
            Serial.println("got here3");
            if (!openSet[currentNode - 4]) {
              openSet[currentNode - 4] = true;
              Serial.println("got here5");
            }
          }
        }
      }
      if (currentNode < 16) {                                                                                       //BOTTOM
        if (!(closedSet[currentNode + 4]) && adjMatrix[currentNode][currentNode + 4] < 1) {                         //if neighbor is NOT in closed OR IS traversable
        
          if (adjMatrix[currentNode][currentNode + 4] < fCostList[currentNode + 4] || !openSet[currentNode + 4]) {  //if new path to neighbor IS shorter OR neighbor is NOT in OPEN
            fCostList[currentNode + 4] = adjMatrix[currentNode][currentNode + 4] + hCost(currentNode + 4);
            parent[currentNode + 4] = currentNode;
          
            if (!openSet[currentNode + 4]) {
              openSet[currentNode + 4] = true;
              Serial.println("got here3");
            }
          }
        }
      }
      if (currentNode % 4 < 3) {                                                                                    //RIGHT
        if (!(closedSet[currentNode + 1]) && adjMatrix[currentNode][currentNode + 1] < 1) {                         //if neighbor is NOT in closed OR IS traversable
          if (adjMatrix[currentNode][currentNode + 1] < fCostList[currentNode + 1] || !openSet[currentNode + 1]) {  //if new path to neighbor IS shorter OR neighbor is NOT in OPEN
            fCostList[currentNode + 1] = adjMatrix[currentNode][currentNode + 1] + hCost(currentNode + 1);
            parent[currentNode + 1] = currentNode;
            if (!openSet[currentNode + 1]) {
              openSet[currentNode + 1] = true;
            }
          }
        }
      }
      if (currentNode % 4 > 0) {                                                                                    //LEFT
        if (!(closedSet[currentNode - 1]) && adjMatrix[currentNode][currentNode - 1] < 1) {                         //if neighbor is NOT in closed OR IS traversable
          if (adjMatrix[currentNode][currentNode - 1] < fCostList[currentNode - 1] || !openSet[currentNode - 1]) {  //if new path to neighbor IS shorter OR neighbor is NOT in OPEN
            fCostList[currentNode - 1] = adjMatrix[currentNode][currentNode - 1] + hCost(currentNode - 1);
            parent[currentNode - 1] = currentNode;
            if (!openSet[currentNode - 1]) {
              openSet[currentNode - 1] = true;
            }
          }
        }
      }
    }
    if (currentNode == endNode) {
      Serial.print("Path: ");
      printPath(parent, endNode);
    }
  }
}

void setup() {
  delay(1000);

  turnSensorSetup();
  turnSensorReset();

  createMatrix();
  emptyFunction();
  Path();
  // for(int i = 0; i < 50; i++){
  //   currentDistance = distance[i];
  //   functionArray[i]();
  //   if (functionArray[i] == stop_Stop){
  //     break;
  //   }
  //   Serial.println("");
  // }
for (int i = 0 ; i < 50; i++){
     if (functionArray[i] == go_fwd) {
     Serial.print("Going fwd ");
   } else if (functionArray[i] == turn_right) {
     Serial.print("Turning right ");
   } else if (functionArray[i] == turn_left) {
     Serial.print("Turning left ");
   } else if (functionArray[i] == fullTurn) {
     Serial.print("Full Turn");
   }else if (functionArray[i] == stop_Stop){
     Serial.print("Stop");
   }else{
     Serial.print("Else");
   }
   Serial.println(currentDistance);
   }
   Serial.println("finished");
delay(5000);
}

void loop() {

  // put your main code here, to run repeatedly
  // mpu6050.update();
  // angle = mpu6050.getAngleZ();
  // mpu6050.update();
  // angle = mpu6050.getAngleZ();
  currentDistance = distance[step];
  functionArray[step]();
}

//Pololu included gyro stuff

// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate()
{
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
}

/* This should be called in setup() to enable and calibrate the
gyro.  It uses the display, yellow LED, and button A.  While the
display shows "Gyro cal", you should be careful to hold the robot
still.

The digital zero-rate level of the gyro can be as high as
25 degrees per second, and this calibration helps us correct for
that. */
void turnSensorSetup()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  display.clear();
  display.print(F("Gyro cal"));

  // Turn on the yellow LED in case the display is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) 
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  display.clear();
  turnSensorReset();

  turnSensorUpdate();
  display.gotoXY(0, 0);
  display.print((((int32_t)turnAngle >> 16) * 360) >> 16);
  display.print(F("   "));

  display.clear();
}

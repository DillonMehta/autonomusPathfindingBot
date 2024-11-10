#define INF 9999  //representation for infinity or no connection
String maze[4][4] = {
  { "R", "L", "R", "L" },
  { "B", "", "B", "" },
  { "T", "", "TR", "L" },
  { "", "", "R", "L" }
};



int gateCount = 5;  //VERY IMPORTANT
int startNode = 15;
int G1 = 2;
int G2 = 8;
int G3 = 0;
int G4 = 10;
int G5 = 3;
// int G6 = 0;
int endNode = 14;

int endPoints[] = { startNode, G1,G2,G3, G4,G5, endNode };

//DRIVING ------------------------------------------------------------------------------------------------------------------------
#include <MPU6050_tockn.h>
#include <Wire.h>


#define SPEED 150  //100 = 0.5 meters in 1350 ms  |  125 = 0.5 meters in 1075 ms |
#define TURN_SPEED 100
#define speedPinR 9           //  Front Wheel PWM pin connect Model-Y M_B ENA
#define RightMotorDirPin1 22  //Front Right Motor direction pin 1 to Model-Y M_B IN1  (K1)
#define RightMotorDirPin2 24  //Front Right Motor direction pin 2 to Model-Y M_B IN2   (K1)
#define LeftMotorDirPin1 26   //Front Left Motor direction pin 1 to Model-Y M_B IN3 (K3)
#define LeftMotorDirPin2 28   //Front Left Motor direction pin 2 to Model-Y M_B IN4 (K3)
#define speedPinL 10          //  Front Wheel PWM pin connect Model-Y M_B ENB

#define speedPinRB 11         //  Rear Wheel PWM pin connect Left Model-Y M_A ENA
#define RightMotorDirPin1B 5  //Rear Right Motor direction pin 1 to Model-Y M_A IN1 ( K1)
#define RightMotorDirPin2B 6  //Rear Right Motor direction pin 2 to Model-Y M_A IN2 ( K1)
#define LeftMotorDirPin1B 7   //Rear Left Motor direction pin 1 to Model-Y M_A IN3  (K3)
#define LeftMotorDirPin2B 8   //Rear Left Motor direction pin 2 to Model-Y M_A IN4 (K3)
#define speedPinLB 12         //  Rear Wheel PWM pin connect Model-Y M_A ENB

//IR Sensor for Angular Velocity
const int encoder1 = 18;
const int encoder2 = 19;

volatile double eCount = 0;
volatile double eCount2 = 0;
volatile double dT = 0;
volatile double dT2 = 0;

void count() {
  eCount++;
  dT = eCount / 20.0 * PI * 6;
  // Serial.println(dT);
}
void count2() {
  eCount2++;
  dT2 = eCount2 / 20.0 * PI * 6;
  // Serial.print("\t");
  // Serial.println(dT2);
}


//GYROSCOPE -----------------------------------------------------------------------------------------------------
MPU6050 mpu6050(Wire);
double fAngle = 0;
double init_angle;

void init_GPIO() {
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(RightMotorDirPin1B, OUTPUT);
  pinMode(RightMotorDirPin2B, OUTPUT);
  pinMode(speedPinLB, OUTPUT);

  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT);
  pinMode(speedPinRB, OUTPUT);

  stop_Stop();
}




//PATHFINDING
int currentDistance;  // THIS IS USED TO GLOBALIZE THE ITERATED VALUE FROM THE DISTANCE ARRAY


//DRIVING -------------------------------------------------------------------------------------------------------------------
/*motor control*/
void go_advance(int speed1, int speed2) {
  RL_fwd(speed1);
  RR_fwd(speed2);
  FR_fwd(speed2);
  FL_fwd(speed1);
}
void go_back(int speed1, int speed2) {
  RL_bck(speed1);
  RR_bck(speed2);
  FR_bck(speed2);
  FL_bck(speed1);
}
void right_shift(int speed_fl_fwd, int speed_rl_bck, int speed_rr_fwd, int speed_fr_bck) {
  FL_fwd(speed_fl_fwd);
  RL_bck(speed_rl_bck);
  RR_fwd(speed_rr_fwd);
  FR_bck(speed_fr_bck);
}
void left_shift(int speed_fl_bck, int speed_rl_fwd, int speed_rr_bck, int speed_fr_fwd) {
  FL_bck(speed_fl_bck);
  RL_fwd(speed_rl_fwd);
  RR_bck(speed_rr_bck);
  FR_fwd(speed_fr_fwd);
}

void left_turn(int speed) {
  RL_bck(0);
  RR_fwd(speed);
  FR_fwd(speed);
  FL_bck(0);
}
void right_turn(int speed) {
  //  RL_fwd(speed);
  //   RR_bck(0);
  //  FR_bck(0);
  //  FL_fwd(speed);

  RL_fwd(speed);
  RR_fwd(0);
  FR_fwd(0);
  FL_fwd(speed);
}
void left_back(int speed) {
  RL_fwd(0);
  RR_bck(speed);
  FR_bck(speed);
  FL_fwd(0);
}
void right_back(int speed) {
  RL_bck(speed);
  RR_fwd(0);
  FR_fwd(0);
  FL_bck(speed);
}
void clockwise(int speed) {
  RL_fwd(speed);
  RR_bck(speed);
  FR_bck(speed);
  FL_fwd(speed);
}
void countclockwise(int speed) {
  RL_bck(speed);
  RR_fwd(speed);
  FR_fwd(speed);
  FL_bck(speed);
}
void FR_fwd(int speed)  //front-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  analogWrite(speedPinR, speed);
}
void FR_bck(int speed)  // front-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  analogWrite(speedPinR, speed);
}
void FL_fwd(int speed)  // front-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  analogWrite(speedPinL, speed);
}
void FL_bck(int speed)  // front-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  analogWrite(speedPinL, speed);
}

void RR_fwd(int speed)  //rear-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B, LOW);
  analogWrite(speedPinRB, speed);
}
void RR_bck(int speed)  //rear-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, HIGH);
  analogWrite(speedPinRB, speed);
}
void RL_fwd(int speed)  //rear-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1B, HIGH);
  digitalWrite(LeftMotorDirPin2B, LOW);
  analogWrite(speedPinLB, speed);
}
void RL_bck(int speed)  //rear-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, HIGH);
  analogWrite(speedPinLB, speed);
}

void stop_Stop()  //Stop
{
  analogWrite(speedPinLB, 0);
  analogWrite(speedPinRB, 0);
  analogWrite(speedPinL, 0);
  analogWrite(speedPinR, 0);
}

int s1 = 125;
int s2 = 125;
double prevAngle = 0;
double angle = 0;

// CALIBRATION VALUES
double fullTurnOffset = -25.5;
double leftAngleOffset = -16.5;
double rightAngleOffset = -15.0;

//For every exit command in every move function, step must increment because the motion is inside of the loop
// UPDATE dRECORD EVERY TIME TOO
int step = 0;
double dRecord = 0;
double dRecord2 = 0;

void update() {
  dRecord = dT;
  //dRecord2 = dT2;
  step++;
  mpu6050.begin();
  mpu6050.update();
  fAngle = mpu6050.getAngleZ();
}
void go_fwd() {
  //debug
  //Serial.println("forward");
  int intent = currentDistance;
  mpu6050.update();
  prevAngle = angle;
  angle = mpu6050.getAngleZ();
  if (dT - dRecord >= intent) {
    go_advance(0, 0);
    delay(1000);
    update();

  } else {
  

    int angDif = round((angle - prevAngle) * 500);
    if (abs(angDif) > 1) {
      if (angDif > 0) {
        s2 -= 1;
        s1 += 1;
      } else if (angDif < 0) {
        s2 += 1;
        s1 -= 1;
      }
    }
    go_advance(s1, s2);
    // Serial.print(s1);
    // Serial.print("\t");
    // Serial.println(s2);
  }
}
void go_bck() {// ONLY OCCURS AT THE END SQUARE
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
  }
}



void turn_right() {
  // Serial.println("Turn Right");
  mpu6050.update();
  angle = mpu6050.getAngleZ();
  if (fAngle - angle >= 90.0 + rightAngleOffset) {
    clockwise(0);
    delay(1000);
    update();
  } else {
    // Serial.print(fAngle);
    // Serial.print("\t");
    // Serial.println(angle);
    clockwise(100);
  }
}
void turn_left() {
  // Serial.println("Turn left");
  mpu6050.update();
  angle = mpu6050.getAngleZ();
  Serial.println(angle);
  if (angle - fAngle >= 90.0 + leftAngleOffset) {
    countclockwise(0);
    delay(1000);
    update();
  } else {

    // Serial.print(fAngle);
    // Serial.print("\t");
    // Serial.println(angle);
    countclockwise(100);
  }
}
void fullTurn() {  //will go counterclockwise
  //Serial.println("Doing full turn");
  mpu6050.update();
  angle = mpu6050.getAngleZ();
  if (angle - fAngle >= 180.0 + fullTurnOffset) {
    countclockwise(0);
    delay(1000);
    update();
  } else {

    // Serial.print(fAngle);
    // Serial.print("\t");
    // Serial.println(angle);
    countclockwise(100);
  }
}




//PATHFINDING --------------------------------------------------------------------------------------------------------------------

int functionCount = 1;
void (*functionArray[50])();  // LEFT OFF HERE
int distance[50];             //should be parallel to functionArray and will use the function Count index
int prevNode = -1;

void emptyFunction() {  //FILLING FUNCTION ARRAY WITH STOP VALUES
  functionArray[0] = go_fwd;
  distance[0] = 11.5+25;
  for (int i = 1; i < 50; i++) {
    functionArray[i] = stop_Stop;
    distance[i] = 45;//BASE DISTANCE
  }
}

int orientation = 0;
void printPath(int parent[], int node) {
  if (parent[node] == -1) {
    // Serial.println(node);
   // route[routeCount] = node;
    return;
  }
  printPath(parent, parent[node]);
  // Serial.print(" --> ");
  // Serial.println(node);

  //RECORDING NODE PATH TO FUNCTION STEPS
  //Serial.println(node - prevNode);
  int distanceAdditive = 45; //can Add Calibration Distance 
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
bool adjMatrix[16][16];

void createMatrix() {
  //EMPTY MATRIX:
  for (int a = 0; a < 16; a++) {
    for (int b = 0; b < 16; b++) {
      adjMatrix[a][b] = 1;
    }
  }
  //OVERWRITING VALUES
  for (int a = 0; a < 16; a++) {
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
    if (str.indexOf('B') < 0 && i < 3) {  // IF THERE IS NO BOTTOM BARRIER && IS NOT IN THE BOTTOM ROW
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
  //   for (int a = 0; a < 16; a++) {
  //   for (int b = 0; b < 16; b++) {
  //     Serial.print(adjMatrix[a][b]);
  //     Serial.print("\t");
  //   }
  //   Serial.println("");//DEBUG CHECKING
  // }
}

int hCost(int node) {
  return abs(node % 4 - endNode % 4) + abs(node / 4 - endNode / 4);
}

void Path() {
  for (int a = 1; a < gateCount + 2; a++) {
    endNode = endPoints[a];
    startNode = endPoints[a - 1];
    int parent[24];  // Array to store the path
    for (int i = 0; i < 24; i++) {
      parent[i] = -1;  // Initialize all nodes as unvisited
    }


    int currentNode = startNode;
    bool openSet[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    bool closedSet[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    int fCostList[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    openSet[startNode] = 1;
    //route[routeCount-1] = startNode;
    prevNode = startNode;
    while (currentNode != endNode) {
      int minCost = INF;
      for (int i = 0; i < 16; i++) {  //SETTING CURRENT TO NODE IN OPENSET WITH THE LOWEST F COST
        if (openSet[i] == true) {

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
        if (!(closedSet[currentNode - 4]) && adjMatrix[currentNode][currentNode - 4] < 0) {                         //if neighbor is NOT in closed OR IS traversable
          if (adjMatrix[currentNode][currentNode - 4] < fCostList[currentNode - 4] || !openSet[currentNode - 4]) {  //if new path to neighbor IS shorter OR neighbor is NOT in OPEN
            fCostList[currentNode - 4] = adjMatrix[currentNode][currentNode - 4] + hCost(currentNode - 4);
            parent[currentNode - 4] = currentNode;
            if (!openSet[currentNode - 4]) {
              openSet[currentNode - 4] = true;
            }
          }
        }
      }
      if (currentNode < 12) {                                                                                       //BOTTOM
        if (!(closedSet[currentNode + 4]) && adjMatrix[currentNode][currentNode + 4] < 0) {                         //if neighbor is NOT in closed OR IS traversable
          if (adjMatrix[currentNode][currentNode + 4] < fCostList[currentNode + 4] || !openSet[currentNode + 4]) {  //if new path to neighbor IS shorter OR neighbor is NOT in OPEN
            fCostList[currentNode + 4] = adjMatrix[currentNode][currentNode + 4] + hCost(currentNode + 4);
            parent[currentNode + 4] = currentNode;
            if (!openSet[currentNode + 4]) {
              openSet[currentNode + 4] = true;
            }
          }
        }
      }
      if (currentNode % 4 < 3) {                                                                                    //RIGHT
        if (!(closedSet[currentNode + 1]) && adjMatrix[currentNode][currentNode + 1] < 0) {                         //if neighbor is NOT in closed OR IS traversable
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
        if (!(closedSet[currentNode - 1]) && adjMatrix[currentNode][currentNode - 1] < 0) {                         //if neighbor is NOT in closed OR IS traversable
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
      //Serial.print("Path: ");
      printPath(parent, endNode);
    }
  }
}




void setup() {
  init_GPIO();
  Serial.begin(115200);
  delay(500);
  attachInterrupt(digitalPinToInterrupt(encoder1), count, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2), count2, RISING);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  //SETTING THE INITIAL ANGLE
  mpu6050.update();
  fAngle = mpu6050.getAngleZ();
  init_angle = fAngle;

  createMatrix();
  emptyFunction();
  // Serial.println("got here");
  Path();
  // for(int i = 0; i < 50; i++){
  //   currentDistance = distance[i];
  //   functionArray[i]();
  //   if (functionArray[i] == stop_Stop){
  //     break;
  //   }
  //   Serial.println("");
  // }
// for (int i = 0 ; i < 50; i++){
//   currentDistance = distance[i];
//     if (functionArray[i] == go_fwd) {
//     Serial.print("Going fwd ");
//   } else if (functionArray[i] == turn_right) {
//     Serial.print("Turning right ");
//   } else if (functionArray[i] == turn_left) {
//     Serial.print("Turning left ");
//   } else if (functionArray[i] == fullTurn) {
//     Serial.print("Full Turn");
//   }else if (functionArray[i] == stop_Stop){
//     Serial.print("Stop");
//   }else{
//     Serial.print("Else");
//   }
//   Serial.println(currentDistance);
//   }
//   Serial.println("finished");
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

#include <PololuBuzzer.h>

//Susu ------------------------------------------------------------------------------------------------------------------------
#include <Wire.h>
#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>

#include <Pololu3piPlus32U4IMU.h>
using namespace Pololu3piPlus32U4;
OLED display;
Encoders encoders;
Buzzer buzzer;
Motors motors;
IMU imu;

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

uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;

uint16_t gyroLastUpdate = 0;
//DRIVING ------------------------------------------------------------------------------------------------------------------------
double s1min = 80; 
double s2min = 80;
double Kpf = 1; // to reach the proper distance
double Kps = 6; //to go straight, only affects right motor
//turning
double t1min = 35;
double t2min = 35;
double Kpt = 0.15; //for turning
double left_angle=85.7;
double right_angle=86.2;

//IR Sensor for Angular Velocity

volatile double eCount = 0;
volatile double eCount2 = 0;
volatile double dT = 0;
volatile double dT2 = 0;


void setup() {
  delay(1000);
  turnSensorSetup();
  turnSensorReset();
  buzzer.playFrequency(440, 200, 15);

  turn_left();
  turn_left();
  turn_left();
  turn_left();
  turn_left();
  turn_left();
  turn_left();
  turn_left();

  delay(5000);

  turn_right();
  turn_right();
  turn_right();
  turn_right();
  turn_right();
  turn_right();
  turn_right();
  turn_right();
  buzzer.playFrequency(440, 200, 15);

 
delay(5000);
}

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
double d1(){return eCount / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE;}
double d2(){return eCount2 / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE;}


void move_fwd(int intent) {
  //debug
  //Serial.println("forward");
  turnSensorUpdate();
  countL();
  countR();
  
  while (!(eCount / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE >= intent)) {
    turnSensorUpdate();
    countL();
    countR();
    motors.setSpeeds( (intent - d1()) * Kpf + s1min, (intent - d1()) * Kpf + s2min - ang() * Kps);

  } 
    motors.setSpeeds(0, 0);
    delay(400);
    update();
  
}


void turn_left() {
  Serial.println("Turn Left");
  turnSensorUpdate();
  while (ang() < left_angle) {
    turnSensorUpdate();
    countL();
    countR();
    motors.setSpeeds(-t1min - abs(left_angle - (ang())) * Kpt, t2min + abs(left_angle - (ang())) * Kpt);
  } 
    // Serial.print(fAngle);
    // Serial.print("\t");
    // Serial.println(angle);
    motors.setSpeeds(0,0);
    delay(400);
    update();
  
}

void turn_right() {
  Serial.println("Turn Right");
  turnSensorUpdate();
  while (ang() > -right_angle) {
    turnSensorUpdate();
    countL();
    countR();
    motors.setSpeeds(t1min + abs(right_angle + (ang())) * Kpt, -t2min - abs(right_angle + (ang())) * Kpt);
  }   // -90 < fullTurn < 0
    motors.setSpeeds(0,0);
    delay(400);
    update();
    
    
  
}

void fullTurn() {  //will go clockwise
  Serial.println("FullTurn");
  turnSensorUpdate();
  if (turnAngle > -180)  {
    motors.setSpeeds(t1min + abs(-90 - ang()) * Kpt, -t2min - abs(-90 - ang()) * Kpt);
  } 

    motors.setSpeeds(0,0);
    delay(1000);
    update();
  
}

void stop_Stop(){
  motors.setSpeeds(0,0);
}



//PATHFINDING --------------------------------------------------------------------------------------------------------------------


void loop() {

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

  
  int32_t d = (int32_t)turnRate * dt;

 
  turnAngle += (int64_t)d * 14680064 / 17578125;
}
/*
int32_t ang(){ return ((((int32_t)turnAngle >> 16) * 360) >> 16);
  }
*/
double ang()
{
  return ((int32_t)turnAngle) * (360.0 / 4294967296.0);
}
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
  display.print(turnAngle);
  display.print(F("   "));

  display.clear();
}

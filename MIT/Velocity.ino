//Includes ------------------------------------------------------------------------------------------------------------------------
#include <Wire.h>
#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>
#include <PololuBuzzer.h>
#include <Pololu3piPlus32U4IMU.h>
using namespace Pololu3piPlus32U4;
OLED display;
Encoders encoders;
Buzzer buzzer;
Motors motors;
IMU imu;
//Don't Touch ----------------------------------------------------------------------------------------------------------
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.05; 
// This constant represents a turn of 90 degrees.

uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;
double lt0;
double ls0;
double rt0;
double rs0;
double start_time;
double total_distance;
volatile double eCount = 0;
volatile double eCount2 = 0;
double vel1;
double vel2;
double total_turns=0;
//turning
double pwm_min = 35;
double Kpt = 0.5; //for turning
double left_angle=85.37;
double right_angle=85.37;
double Ks = 1; // to reach the proper distance
double kP = 6; //to go straight, only affects right motor
const double turnTime = 1; //time for each turn.
//Movement Values (Change here) ------------------------------------------------------------------------------------------------------------------------

double targetTime = 60;


char movement[200] = "F30 B30 L R E ";



//setup ================================================================================
void setup() {
  delay(1000);
  turnSensorSetup();
  turnSensorReset();
  start_time = micros();
  
  buzzer.playFrequency(440, 200, 15);
  buzzer.playFrequency(440, 200, 15);
 
  delay(5000);
}
//
void update() {
  turnSensorUpdate();
  eCount= encoders.getCountsLeft();
  eCount2 = encoders.getCountsRight();
  
}
void reset() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  turnSensorReset();
  turnAngle=0;
}
double dL(){return eCount / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE;}
double dR(){return eCount2 / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE;}
double vL(){
  int vel = (dL()-ls0)/(micros()-lt0)*100000;
  ls0 = dL();
  lt0 = micros();
  return vel;
}
double vR(){
  int vel = (dR()-rs0)/(micros()-rt0)*100000;  
  rs0 = dR();
  rt0 = micros();
  return vel;
}

void fwd(double distance) {
  update();
  double t0 = micros();
  double delta_T = findTime(distance);
  double vL_calc;
  double vR_calc;
  double left_pwm = pwm_min;
  double right_pwm = pwm_min;
  while ((micros()-t0)<= delta_T/4) {
    left_pwm += kP * (16*distance/(3*pow(delta_T/100000,2))*(micros()-t0)/100000 - vL());
    right_pwm += kP * (16*distance/(3*pow(delta_T/100000,2))*(micros()-t0)/100000 - vR());
    motors.setSpeeds(left_pwm, right_pwm);
  }
  while ((micros()-t0)<= 3*delta_T/4) {
    left_pwm += kP * (4*distance/(3*delta_T) - vL());
    right_pwm += kP * (4*distance/(3*delta_T) - vR());
    motors.setSpeeds(left_pwm, right_pwm);
  }  
  while(dL() < distance){
    left_pwm += kP *( 4*distance/(3*delta_T) - 16*distance/(3*pow(delta_T/100000,2))*(micros()-t0)/100000- vL());
    right_pwm += kP *( 4*distance/(3*delta_T) -16*distance/(3*pow(delta_T/100000,2))*(micros()-t0)/100000- vR());
    motors.setSpeeds(left_pwm, right_pwm);
  }
    motors.setSpeeds(0, 0);
    delay(300);
    reset();
}


void back(double intent) {
  //debug
  //Serial.println("forward");
  update();
  
  while (!(dL() >= intent/4)) {
    update();
  } 
    motors.setSpeeds(0, 0);
    delay(300);
    reset();
}
void end() {
  fwd(44);
}
void left() {
  Serial.println("Turn Left");
  
  while (ang() < left_angle) {
    motors.setSpeeds(-pwm_min - abs(left_angle - (ang())) * Kpt, pwm_min + abs(left_angle - (ang())) * Kpt);
  } 
    motors.setSpeeds(0,0);
    delay(250);
    reset();
}

void right() {
 
  while (ang() > -right_angle) {  
    update();
    motors.setSpeeds(pwm_min + abs(right_angle + (ang())) * Kpt, -pwm_min - abs(right_angle + (ang())) * Kpt);
  }   // -90 < fullTurn < 0
    motors.setSpeeds(0,0);
    delay(250);
    reset();
}

//helper functions -----------------------------------------------------------------------

void loop() {

}
double findTime(double distance){
  return (targetTime - total_turns*turnTime) * (distance/total_distance) * 100000;
}
void calculateTotalDistance(const char* commands) {
  const char* ptr = commands;
  while (*ptr != '\0') {
    if (*ptr == 'F' || *ptr == 'B') {
      ptr++;
      int distance = 0;
      while (*ptr >= '0' && *ptr <= '9') {
        distance = distance * 10 + (*ptr - '0');
        ptr++;
      }
      total_distance += distance;
    } else {
      ptr++;
    }
  }
}
void calculateTotalTurns(const char* commands) {
  const char* ptr = commands;
  while (*ptr != '\0') {
    if (*ptr == 'L' || *ptr == 'R') {
      total_turns++;
    }
    ptr++;
  }
}

void processCommands(const char* commands) {
  const char* ptr = commands; // Pointer to traverse the char array
  while (*ptr != '\0') { // Loop until null terminator
    if (*ptr == 'F' || *ptr == 'B') {
      char cmd = *ptr; // Store the command ('F' or 'B')
      ptr++; // Move to the number part
      int distance = 0;
      while (*ptr >= '0' && *ptr <= '9') { // Extract number
        distance = distance * 10 + (*ptr - '0');
        ptr++;
      }
      if (cmd == 'F') {
        fwd(distance);
      } else {
        back(distance);
      }
    } else if (*ptr == 'L') {
      left();
      ptr++;
    } else if (*ptr == 'R') {
      right();
      ptr++;
    } else if (*ptr == 'E') {
      end();
      ptr++;
    } else {
      ptr++; // Skip unrecognized characters
    }
  }
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

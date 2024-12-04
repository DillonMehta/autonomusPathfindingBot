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
const int32_t turnAngle45 = 0x20000000;
// This constant represents a turn of 90 degrees.
const int32_t turnAngle90 = turnAngle45 * 2;
// This constant represents a turn of approximately 1 degree.
const int32_t turnAngle1 = (turnAngle45 + 22) / 45;
uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;
double t_zero = 0;
double startTime;

volatile double eCount = 0;
volatile double eCount2 = 0;
double vel1;
double vel2;
//Movement Values (Change here) ------------------------------------------------------------------------------------------------------------------------
double targetTime = 60;
double s1min = 30; 
double s2min = 30;
double Ks = 1; // to reach the proper distance
double Kps = 6; //to go straight, only affects right motor
//turning
double t1min = 35;
double t2min = 35;
double Kpt = 0.5; //for turning
double left_angle=85.37;
double right_angle=85.37;
char[200] movement = "F30 B30 L R E";

void move_control(){
  int c = 0;
  while(movement[c] != 'E'){
    if(movement[c]==
  }
}
//DRIVING -------------------------------------------------------------------------------------------------------------------
void setup() {
  delay(1000);
  turnSensorSetup();
  turnSensorReset();
  parseMovementCommands(movement);
  
  buzzer.playFrequency(440, 200, 15);
  buzzer.playFrequency(440, 200, 15);
 
delay(5000);
}

void update() {
  eCount= encoders.getCountsLeft();
  eCount2 = encoders.getCountsRight();
  
}
void reset() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  turnSensorReset();
  turnAngle=0;
}
double d1(){return eCount / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE;}
double d2(){return eCount2 / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE;}

void fwd(int intent) {
  //debug
  //Serial.println("forward");
  turnSensorUpdate();
  
  while (!(d1() >= intent/4)) {
    turnSensorUpdate();
    count();
    

  } 
    motors.setSpeeds(0, 0);
    delay(300);
    reset();
  
}


void left() {
  Serial.println("Turn Left");
  turnSensorUpdate();
  while (ang() < left_angle) {
    turnSensorUpdate();
    count();
    motors.setSpeeds(-t1min - abs(left_angle - (ang())) * Kpt, t2min + abs(left_angle - (ang())) * Kpt);
  } 
    // Serial.print(fAngle);
    // Serial.print("\t");
    // Serial.println(angle);
    motors.setSpeeds(0,0);
    delay(250);
    reset();
  
}

void right() {
  Serial.println("Turn Right");
  turnSensorUpdate();
  while (ang() > -right_angle) {
    turnSensorUpdate();
    count();
    count();
    motors.setSpeeds(t1min + abs(right_angle + (ang())) * Kpt, -t2min - abs(right_angle + (ang())) * Kpt);
  }   // -90 < fullTurn < 0
    motors.setSpeeds(0,0);
    delay(250);
    reset();
    
    
  
}

//helper functions -----------------------------------------------------------------------

void loop() {

}
void parseMovementCommands(char* commands) {
  int len = strlen(commands);
  int i = 0;
  
  while (i < len) {
    // Skip any whitespace
    while (i < len && isspace(commands[i])) {
      i++;
    }
    
    // Check if we've reached the end of the string
    if (i >= len) break;
    
    // Parse command
    switch (commands[i]) {
      case 'F': // Forward
        // Look for the number following F or B
          i++; // Move past F or B
          int distance = 0;
          
          // Parse the number
          while (i < len && isdigit(commands[i])) {
            distance = distance * 10 + (commands[i] - '0');
            i++;
          }

          move(distance);
          break;
      case 'B': // Backward
        
          // Look for the number following F or B
          i++; // Move past F or B
          int distance = 0;
          
          // Parse the number
          while (i < len && isdigit(commands[i])) {
            distance = distance * 10 + (commands[i] - '0');
            i++;
          }
          
          back(distance);
          break;
      
      case 'L': // Left turn
        left();
        i++;
        break;
      
      case 'R': // Right turn
        right();
        i++;
        break;
      
      case 'E': // End of commands
        end();
        return; // Exit the function completely
      
      default:
        // Skip any unrecognized characters
        i++;
        break;
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

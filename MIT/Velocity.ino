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
double kS = 0;
double kP = 1; //to go straight, only affects right motor
const double turnTime = 500; //time for each turn in ms.
//Movement Values (Change here) ------------------------------------------------------------------------------------------------------------------------

double targetTime = 4;
double end_distance = 50;

char movement[200] = "F50 E";



//setup ================================================================================
void setup() {
  delay(500);
  turnSensorSetup();
  turnSensorReset();
  calculateTotalTurns(movement);
  calculateTotalDistance(movement);
  start_time = micros();
  buzzer.playFrequency(440, 200, 15);
  buzzer.playFrequency(440, 200, 15);
  processCommands(movement);
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
  double vel = (dL()-ls0)/(micros()-lt0)*1000000;
  ls0 = dL();
  lt0 = micros();
  return vel;
}
double vR(){
  double vel = (dR()-rs0)/(micros()-rt0)*1000000;  
  rs0 = dR();
  rt0 = micros();
  return vel;
}
void fwd(double distance) {
  update();
  double t0 = micros(); // Start time in microseconds
  double delta_T = findTime(distance);
  double delta_T_us = delta_T * 1e6; // Convert delta_T from seconds to microseconds
  double left_pwm = pwm_min;
  double right_pwm = pwm_min;

  double velocity_setpoint = 0; // Initialize the velocity setpoint
  double elapsed_time;

  // Main control loop
  while (true) {
    elapsed_time = micros() - t0; // Elapsed time in microseconds
    update();
    // Exit condition: Distance has been covered or time has exceeded delta_T
    if (dL() >= distance || elapsed_time >= delta_T_us) {
      break;
    }

    // Determine velocity setpoint based on elapsed time
    if (elapsed_time <= delta_T_us / 4) {
      // Acceleration phase
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * (elapsed_time / 1e6);
    } else if (elapsed_time <= 3 * delta_T_us / 4) {
      // Constant velocity phase
      velocity_setpoint = (4.0 * distance) / (3.0 * delta_T);
    } else {
      // Deceleration phase
      double t_dec = elapsed_time - 3 * delta_T_us / 4;
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * ((delta_T / 4) - t_dec / 1e6);
    }

    // Update PWM values based on velocity feedback and setpoint
    double velocity_error_L = velocity_setpoint - vL();
    double velocity_error_R = velocity_setpoint - vR();

    left_pwm += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R + kS * ( vL()-vR());

    // Constrain PWM values to valid range
    left_pwm = constrain(left_pwm, pwm_min, 400);
    right_pwm = constrain(right_pwm, pwm_min, 400);

    // Set motor speeds
    motors.setSpeeds(left_pwm, right_pwm);
  }

  // Stop motors at the end
  motors.setSpeeds(0, 0);
  delay(50); // Small delay to ensure stop
  reset();   // Reset necessary parameters
}
/*
void fwd(double distance) {
  update();
  double t0 = micros();
  double delta_T = findTime(distance);
  double vL_calc;
  double vR_calc;
  double left_pwm = pwm_min;
  double right_pwm = pwm_min;
  while ((micros()-t0)<= delta_T/4) {
    left_pwm += kP * (16*distance/(3*pow(delta_T/1000000,2))*(micros()-t0)/1000000 - vL());
    right_pwm += kP * (16*distance/(3*pow(delta_T/1000000,2))*(micros()-t0)/1000000 - vR());
    motors.setSpeeds(left_pwm, right_pwm);
  }
  while ((micros()-t0)<= 3*delta_T/4) {
    left_pwm += kP * (4*distance/(3*delta_T) - vL());
    right_pwm += kP * (4*distance/(3*delta_T) - vR());
    motors.setSpeeds(left_pwm, right_pwm);
  }  
  while(dL() < distance){
    left_pwm += kP *( 4*distance/(3*delta_T) - 16*distance/(3*pow(delta_T/1000000,2))*(micros()-t0)/1000000- vL());
    right_pwm += kP *( 4*distance/(3*delta_T) -16*distance/(3*pow(delta_T/1000000,2))*(micros()-t0)/1000000- vR());
    motors.setSpeeds(left_pwm, right_pwm);
  }
    motors.setSpeeds(0, 0);
    delay(50);
    reset();
}



void end() {
  update();
  double t0 = micros();
  double delta_T = findTime(end_distance);
  double left_pwm = pwm_min;
  double right_pwm = pwm_min;
  double final_target = targetTime*1e6 - (micros() - start_time); 
  while(dL() < end_distance){
    update();
    left_pwm += kP * (end_distance/(final_target/1e6) -vL());
    right_pwm += kP * (end_distance/(final_target/1e6) -vR());
    motors.setSpeeds(left_pwm, right_pwm);
  }
  motors.setSpeeds(0, 0);
  reset();
}
*/
void end() {
  update();
  double distance = end_distance;
  double t0 = micros(); // Start time in microseconds
  double delta_T = targetTime - (micros() - start_time)/1e6;
  double delta_T_us = delta_T * 1e6; // Convert delta_T from seconds to microseconds
  double left_pwm = pwm_min;
  double right_pwm = pwm_min;

  double velocity_setpoint = 0; // Initialize the velocity setpoint
  double elapsed_time;

  // Main control loop
  while (true) {
    elapsed_time = micros() - t0; // Elapsed time in microseconds
    update();
    // Exit condition: Distance has been covered or time has exceeded delta_T
    if (dL() >= distance || elapsed_time >= delta_T_us) {
      break;
    }

    // Determine velocity setpoint based on elapsed time
    if (elapsed_time <= delta_T_us / 4) {
      // Acceleration phase
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * (elapsed_time / 1e6);
    } else if (elapsed_time <= 3 * delta_T_us / 4) {
      // Constant velocity phase
      velocity_setpoint = (4.0 * distance) / (3.0 * delta_T);
    } else {
      // Deceleration phase
      double t_dec = elapsed_time - 3 * delta_T_us / 4;
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * ((delta_T / 4) - t_dec / 1e6);
    }

    // Update PWM values based on velocity feedback and setpoint
    double velocity_error_L = velocity_setpoint - vL();
    double velocity_error_R = velocity_setpoint - vR();

    left_pwm += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R + kS *( vL()-vR());

    // Constrain PWM values to valid range
    left_pwm = constrain(left_pwm, pwm_min, 400);
    right_pwm = constrain(right_pwm, pwm_min, 400);

    // Set motor speeds
    motors.setSpeeds(left_pwm, right_pwm);
  }

  // Stop motors at the end
  motors.setSpeeds(0, 0);
  delay(50); // Small delay to ensure stop
  reset();   // Reset necessary parameters
}
void left() {
  int starting = millis();
  while (ang() < left_angle) {
    update();
    motors.setSpeeds(-pwm_min - abs(left_angle - (ang())) * Kpt, pwm_min + abs(left_angle - (ang())) * Kpt);
  }
  motors.setSpeeds(0,0);
  while(millis() - starting < turnTime){
    motors.setSpeeds(0,0);
  }
  reset();
}

void right() {
  int starting = millis();
  while (ang() > -right_angle) {  
    update();
    motors.setSpeeds(pwm_min + abs(right_angle + (ang())) * Kpt, -pwm_min - abs(right_angle + (ang())) * Kpt);
  }   // -90 < fullTurn < 0
    motors.setSpeeds(0,0);
    while(millis() - starting < turnTime){
      motors.setSpeeds(0,0);
    }
    reset();
}

//helper functions -----------------------------------------------------------------------

void loop() {

}
double findTime(double distance){
  return (targetTime - total_turns*turnTime/1e3) * (distance/total_distance); //seconds
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
  total_distance += end_distance;
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
        fwd(-distance);
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

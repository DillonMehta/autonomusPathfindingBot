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
const float WHEEL_CIRCUMFERENCE = 10.25; 
const float BOT_RADIUS = 4.21; //cm horizontal radius(wheels to center)

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
double total_turns=0;
//turning
double pwm_min = 35;
double Kpt = 1; //for turning
double left_angle=90;
double right_angle=90;
double kPs = 0.2; //to go straight, only affects right motor
double kP = 0.4; 
const double turnTime = 1500; //time for each turn in ms.
//Movement Values (Change here) ------------------------------------------------------------------------------------------------------------------------

double targetTime = 60;
double end_distance = 100;
double end_delay = 0;
char movement[200] = "F35 R F50 L F100 R F50 L T L F200 L F100 L F50 L F150 R F100 R F150 L F50 L T L F50 R F150 L E";



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
double dR(){return eCount2 / (CLICKS_PER_ROTATION * GEAR_RATIO) * (WHEEL_CIRCUMFERENCE - 0.05);}
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
    if (dL() >= distance || elapsed_time  >= delta_T_us + 1e5) {
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
    right_pwm += kP * velocity_error_R -  kPs * ang();

    // Constrain PWM values to valid range
    left_pwm = constrain(left_pwm, pwm_min, 400);
    right_pwm = constrain(right_pwm, pwm_min, 400);

    // Set motor speeds
    motors.setSpeeds(left_pwm, right_pwm);
  }

  // Stop motors at the end
  motors.setSpeeds(0, 0);
  delay(10); // Small delay to ensure stop
  reset();   // Reset necessary parameters
}
void back(double distance) {
  update();
  double t0 = micros();
  double delta_T= findTime(distance);             // Start time in microseconds
  double delta_T_us = delta_T * 1e6; // Convert delta_T from seconds to microseconds
  double left_pwm = -pwm_min;       // Initialize PWM for backward movement
  double right_pwm = -pwm_min;

  double velocity_setpoint = 0; // Velocity setpoint for each phase
  double elapsed_time;

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

    // Update PWM values based on velocity feedback and setpoint (negative for backward motion)
    double velocity_error_L = -velocity_setpoint - vL();
    double velocity_error_R = -velocity_setpoint - vR();

    left_pwm += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R;

    // Constrain PWM values to valid range
    left_pwm = constrain(left_pwm, -400, -pwm_min);
    right_pwm = constrain(right_pwm, -400, -pwm_min);

    // Set motor speeds
    motors.setSpeeds(left_pwm, right_pwm);
  }

  // Stop motors at the end
  motors.setSpeeds(0, 0);
  delay(50); // Small delay to ensure stop
  reset();   // Reset necessary parameters
}

void end(double d) {
  update();
  delay(end_delay);
  double distance = end_distance + d;
  double t0 = micros(); // Start time in microseconds
  double delta_T = ((targetTime *1e6 + start_time) - t0)/1e6; 
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
    if (dL() >= distance || elapsed_time  >= delta_T_us + 1e5) {
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
    double velocity_error_R = velocity_setpoint - vR() -  kPs * ang();

    left_pwm += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R;

    // Constrain PWM values to valid range
    left_pwm = constrain(left_pwm, pwm_min, 400);
    right_pwm = constrain(right_pwm, pwm_min, 400);

    // Set motor speeds
    motors.setSpeeds(left_pwm, right_pwm);
  }
  // Stop motors at the end
  motors.setSpeeds(0, 0);
  delay(250); // Small delay to ensure stop
  reset();   // Reset necessary parameters
}

void left() {
  int starting = millis();
  update();
  delay(200);
  while (dR() < BOT_RADIUS * 3.14159 ) {
    update();
    motors.setSpeeds(0, pwm_min + abs(left_angle - (ang())) * Kpt);
  }
  motors.setSpeeds(0,0);
  delay(turnTime > (millis() - starting) ? turnTime - (millis() - starting) + 100: 100);
  reset();
}
void right() {
  int starting = millis();
  update();
  delay(200);
  while (dL() < BOT_RADIUS * 3.14159 ) {  
    update();
    motors.setSpeeds(pwm_min + abs(right_angle + (ang())) * Kpt,0);
  }   // -90 < fullTurn < 0
  motors.setSpeeds(0,0);
  delay(turnTime > (millis() - starting) ? turnTime - (millis() - starting)  + 100: 100);
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
      float distance = 0.0;
      float factor = 1.0;
      bool isDecimal = false;
      while ((*ptr >= '0' && *ptr <= '9') || *ptr == '.') {
        if (*ptr == '.') {
          isDecimal = true;
          factor = 0.1;
        } else if (!isDecimal) {
          distance = distance * 10 + (*ptr - '0');
        } else {
          distance += (*ptr - '0') * factor;
          factor *= 0.1;
        }
        ptr++;
      }
      total_distance += distance;
    } else if(*ptr ==' G'){
      total_distance += 70;
      
    } else{
      ptr++;
    }
  }
  total_distance += end_distance;
  total_distance -= total_turns * 2 * BOT_RADIUS;
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
   // Flag to track if the previous command was a turn
  bool previousBack=false;
  while (*ptr != '\0') { // Loop until null terminator
    if (*ptr == 'F' || *ptr == 'B') {
      char cmd = *ptr; // Store the command ('F' or 'B')
      ptr++; // Move to the number part
      float distance = 0.0;
      float factor = 1.0;
      bool isDecimal = false;
      while ((*ptr >= '0' && *ptr <= '9') || *ptr == '.') { // Extract number
        if (*ptr == '.') {
          isDecimal = true;
          factor = 0.1;
        } else if (!isDecimal) {
          distance = distance * 10 + (*ptr - '0');
        } else {
          distance += (*ptr - '0') * factor;
          factor *= 0.1;
        }
        ptr++;
      }

      distance -= 2 * BOT_RADIUS; // Subtract BOT_RADIUS if there was a turn before

      
      
      // Ensure distance is non-negative
      if (cmd == 'F') {
        if(previousBack){
          distance += 2* BOT_RADIUS;
        }
        fwd(distance);
        previousBack=true;
      } else {
        if(previousBack){
          distance += 2* BOT_RADIUS;
        }
        back(distance);
        previousBack = false;
      }
    } else if (*ptr == 'T') {
        fwd(35-2*BOT_RADIUS);
        back(35);
        ptr++;
        previousBack=false;
    } else if (*ptr == 'S') {
        fwd(35);
        back(35);
        ptr++;
        previousBack=false;
    } else if (*ptr == 'L') {
      left();
      ptr++;
      previousBack=false;
    } else if (*ptr == 'R') {
      right();
      ptr++;
      previousBack=false;
    } else if (*ptr == 'E') {
      if(previousBack){
        end(2* BOT_RADIUS);
      }
      end(0);
      ptr++;
      previousBack=false;
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

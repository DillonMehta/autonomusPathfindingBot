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


// turning
double pwm_min = 35;
double str_min = 45;
double Kpt = 0.75; //for turning
double left_angle=85.8;
double right_angle=85.8;
double kPs  = 0.1; // to go straight, only affects right motor
double kP   = 0.1;
const double turnTime = 1600; // time for each turn in ms.


//Movement Values (Change here) ------------------------------------------------------------------------------------------------------------------------
double targetTime    = 60;
double end_distance  = 44.5 + 50;
double end_delay     = 0;
char movement[200] = "F30.5 R F50 L F100 R F50 L F50 B50 L F200 L F100 L F50 L F150 R F100 R F150 L F50 L F50 B50 L F50 R F150 L E";


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


void loop() {
  // no-op
}


// ---------------------------------------------------------------------------
// Helpers for encoders, gyro updates, etc. (unchanged except for labeling)
// ---------------------------------------------------------------------------
void update() {
  turnSensorUpdate();
  eCount  = encoders.getCountsLeft();
  eCount2 = encoders.getCountsRight();
}


void reset() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  turnSensorReset();
  turnAngle = 0;
}


double dL() { 
  return eCount  / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE; 
}
double dR() {
  return eCount2 / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE; 
}
double vL() {
  double vel = (dL() - ls0) / (micros() - lt0) * 1000000;
  ls0 = dL();
  lt0 = micros();
  return vel;
}
double vR() {
  double vel = (dR() - rs0) / (micros() - rt0) * 1000000;
  rs0 = dR();
  rt0 = micros();
  return vel;
}


// ---------------------------------------------------------------------------
// Forward/Backward functions (unchanged)
// ---------------------------------------------------------------------------
void fwd(double distance) {
  update();
  double t0 = micros(); // Start time
  double delta_T    = findTime(distance) - 0.2;
  double delta_T_us = delta_T * 1e6;
  double left_pwm   = str_min;
  double right_pwm  = str_min;
  
  double velocity_setpoint = 0;
  double elapsed_time;


  while (true) {
    elapsed_time = micros() - t0;
    update();


    if (dL() >= distance) {
      break;
    }
    
    // Accel / constant / decel phases
    if (elapsed_time <= delta_T_us / 4) {
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * (elapsed_time / 1e6);
    } else if (elapsed_time <= 3 * delta_T_us / 4) {
      velocity_setpoint = (4.0 * distance) / (3.0 * delta_T);
    } else {
      double t_dec = elapsed_time - 3 * delta_T_us / 4;
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * ((delta_T / 4) - t_dec / 1e6);
    }


    // PWM feedback
    double velocity_error_L = velocity_setpoint - vL();
    double velocity_error_R = velocity_setpoint - vR();
    
    left_pwm  += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R - kPs * ang();


    left_pwm  = constrain(left_pwm,  str_min, 400);
    right_pwm = constrain(right_pwm, str_min, 400);


    motors.setSpeeds(left_pwm, right_pwm);
  }


  motors.setSpeeds(0, 0);
  delay(200);
  reset();
}


void back(double distance) {
  update();
  double t0         = micros();
  double delta_T    = findTime(distance) - 0.2;
  double delta_T_us = delta_T * 1e6;
  double left_pwm   = -str_min;
  double right_pwm  = -str_min;


  double velocity_setpoint = 0;
  double elapsed_time;


  while (true) {
    elapsed_time = micros() - t0;
    update();


    if (dL() <= -distance) {
      break;
    }
    
    // Accel / constant / decel phases
    if (elapsed_time <= delta_T_us / 4) {
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * (elapsed_time / 1e6);
    } else if (elapsed_time <= 3 * delta_T_us / 4) {
      velocity_setpoint = (4.0 * distance) / (3.0 * delta_T);
    } else {
      double t_dec = elapsed_time - 3 * delta_T_us / 4;
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * ((delta_T / 4) - t_dec / 1e6);
    }


    double velocity_error_L = -velocity_setpoint - vL();
    double velocity_error_R = -velocity_setpoint - vR();


    left_pwm  += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R - kPs * ang();


    left_pwm  = constrain(left_pwm,  -400, -str_min);
    right_pwm = constrain(right_pwm, -400, -str_min);


    motors.setSpeeds(left_pwm, right_pwm);
  }


  motors.setSpeeds(0, 0);
  delay(200);
  reset();
}


void end() {
  update();
  double distance    = end_distance;
  double t0          = micros();
  double delta_T     = ((targetTime *1e6 + start_time) - t0) / 1e6;
  double delta_T_us  = delta_T * 1e6;
  double left_pwm    = str_min;
  double right_pwm   = str_min;


  double velocity_setpoint = 0;
  double elapsed_time;


  while (true) {
    elapsed_time = micros() - t0;
    update();


    if (dL() >= distance) {
      break;
    }
    
    // Accel / constant / decel phases
    if (elapsed_time <= delta_T_us / 4) {
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * (elapsed_time / 1e6);
    } else if (elapsed_time <= 3 * delta_T_us / 4) {
      velocity_setpoint = (4.0 * distance) / (3.0 * delta_T);
    } else {
      double t_dec = elapsed_time - 3 * delta_T_us / 4;
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * ((delta_T / 4) - t_dec / 1e6);
    }


    // PWM feedback
    double velocity_error_L = velocity_setpoint - vL();
    double velocity_error_R = velocity_setpoint - vR();
    
    left_pwm  += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R - kPs * ang();


    left_pwm  = constrain(left_pwm,  str_min, 400);
    right_pwm = constrain(right_pwm, str_min, 400);


    motors.setSpeeds(left_pwm, right_pwm);
  }


  motors.setSpeeds(0, 0);
  delay(10);
  reset();
}


// ---------------------------------------------------------------------------
// Original left() and right() - time-based with a near-90° turn
// ---------------------------------------------------------------------------
void left() {
  int starting = millis();
  delay(100);
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
  delay(100);
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




void processCommands(const char* commands)
{
  const char* ptr = commands;
  while (*ptr != '\0')
  {
    // -------------------------------------------------
    // F or B
    // -------------------------------------------------
    if (*ptr == 'F' || *ptr == 'B') {
      char cmd = *ptr;
      ptr++;  // Move past 'F' or 'B'


      // Parse distance
      float distance = 0.0;
      float factor   = 1.0;
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

      if (cmd == 'F') {
  
        fwd(distance);
        
      } else {  // 'B'
    
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


double findTime(double distance){
  return (targetTime - total_turns*turnTime/1e3) * (distance/total_distance); //seconds
}


void calculateTotalDistance(const char* commands) {
  const char* ptr = commands;
  while (*ptr != '\0') {
    if (*ptr == 'F' || *ptr == 'B') {
      ptr++;
      float distance = 0.0;
      float factor   = 1.0;
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

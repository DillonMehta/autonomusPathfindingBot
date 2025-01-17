// Includes ------------------------------------------------------------------------------------------------------------------------
#include <Wire.h>
#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>
#include <PololuBuzzer.h>
#include <Pololu3piPlus32U4IMU.h>


using namespace Pololu3piPlus32U4;


// Pololu objects
OLED display;
Encoders encoders;
Buzzer buzzer;
Motors motors;
IMU imu;


// Don't Touch ----------------------------------------------------------------------------------------------------------
const int   CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO          = 29.86F;
const float WHEEL_DIAMETER      = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.25;
const float BOT_RADIUS          = 4.3; // cm horizontal radius (wheels to center)


// Global variables for gyro-based turning
uint32_t turnAngle     = 0;
int16_t  turnRate      = 0;
int16_t  gyroOffset    = 0;
uint16_t gyroLastUpdate= 0;


// Misc. distances
double lt0, ls0, rt0, rs0, a0;  
double start_time      = 0; 
double total_distance  = 0; 
volatile double eCount = 0;
volatile double eCount2= 0;
double total_turns     = 0;


// slow turns

const double pwm_min     = 25;     // minimal PWM for movement
double Kpt         = 0.1;    // proportional factor for turning
double left_angle  = 85.8;   // approx. “normal” left turn angle
double right_angle = 85.8;   // approx. “normal” right turn angle
const double turnTime = 2500;


//straight
double kPs = 0.05;          // small angle correction for going straight
double kP  = 0.1;            // for velocity control
double str_min = 60;


// Movement Values (Change here) ------------------------------------------------------------------------------------------------
double targetTime   = 60;
double end_distance = 44.7 + 50;   // distance in cm to move when "end"
double end_delay    = 0;    // not currently used

// Example movement array
char movement[200] = "L L L L";
//"F30.3 R F50 L F100 R F50 L F50 B50 L F200 L F100 L F50 L F150 R F100 R F150 L F50 L F50 B50 L F50 R F150 L E";


// -------------------------------------------------------------------------------------------------
// setup
// -------------------------------------------------------------------------------------------------
void setup() {
  delay(500);
  turnSensorSetup();
  turnSensorReset();
  calculateTotalTurns(movement);
  calculateTotalDistance(movement);
  start_time = micros();


  // A couple of beeps:
  buzzer.playFrequency(440, 200, 15);
  buzzer.playFrequency(440, 200, 15);


  // Process the command array
  processCommands(movement);


  // Another beep or two:
  buzzer.playFrequency(440, 200, 15);
  buzzer.playFrequency(440, 200, 15);


  delay(5000);
}


// -------------------------------------------------------------------------------------------------
// loop
// -------------------------------------------------------------------------------------------------
void loop() {
  // nothing here
}


// -------------------------------------------------------------------------------------------------
// update & reset
// -------------------------------------------------------------------------------------------------
void update() {
  turnSensorUpdate();
  eCount  = encoders.getCountsLeft();
  eCount2 = encoders.getCountsRight();
}


void reset() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  eCount = 0;
  eCount2 = 0;
  turnSensorReset();
  turnAngle = 0;

}


// -------------------------------------------------------------------------------------------------
// Distances & velocities
// -------------------------------------------------------------------------------------------------
double dL() { 
  return eCount / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE; 
}
double dR() {
  return eCount2 / (CLICKS_PER_ROTATION * GEAR_RATIO) * (WHEEL_CIRCUMFERENCE -0.02);
}
double vL() {
  double vel = (dL() - ls0) / (micros() - lt0) * 1e6;
  ls0 = dL();
  lt0 = micros();
  return vel;
}
double vR() {
  double vel = (dR() - rs0) / (micros() - rt0) * 1e6;
  rs0 = dR();
  rt0 = micros();
  return vel;
}
double aV(){
  double vel = (ang()-a0)/(micros() - rt0) * 1e6;
  a0 = ang();
  rt0 = micros();
  return vel;
}

// -------------------------------------------------------------------------------------------------
// Forward, Back, End - trapezoidal velocity style
// -------------------------------------------------------------------------------------------------
void fwd(double distance) {
  reset();
  double t0         = micros();
  double delta_T    = findTime(distance);
  double delta_T_us = delta_T * 1e6;


  double left_pwm   = str_min;
  double right_pwm  = str_min;
  double velocity_setpoint = 0;
  double elapsed_time;


  while (true) {
    elapsed_time = micros() - t0;
    update();


    if (dR() >= distance) {
      break;
    }

    // Accel / constant / decel
    if (elapsed_time <= delta_T_us / 4.0) {
      // Acceleration
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * (elapsed_time / 1e6);
    } else if (elapsed_time <= 3.0 * delta_T_us / 4.0) {
      // Constant speed
      velocity_setpoint = (4.0 * distance) / (3.0 * delta_T);
    } else {
      // Deceleration
      double t_dec = elapsed_time - 3.0 * delta_T_us / 4.0;
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * ((delta_T / 4.0) - t_dec / 1e6);
    }

    // P-control
    double velocity_error_L = velocity_setpoint - vL();
    double velocity_error_R = velocity_setpoint - vR();


    left_pwm  += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R;


    left_pwm  = constrain(left_pwm,  str_min, 400);
    right_pwm = constrain(right_pwm, str_min, 400);

    right_pwm -= kPs * ang();

    motors.setSpeeds(left_pwm, right_pwm);
  }
  

  // Stop
  motors.setSpeeds(0, 0);
  reset();
  delay(15);
}


void back(double distance) {
  reset();
  double t0         = micros();
  double delta_T    = findTime(distance);
  double delta_T_us = delta_T * 1e6;


  double left_pwm   = -pwm_min;
  double right_pwm  = -pwm_min;


  double velocity_setpoint = 0;
  double elapsed_time;


  while (true) {
    elapsed_time = micros() - t0;
    update();


    if (dL() < -distance) {
      break;
    }


    if (elapsed_time <= delta_T_us / 4.0) {
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T)  * (elapsed_time / 1e6);
    } else if (elapsed_time <= 3.0 * delta_T_us / 4.0) {
      velocity_setpoint = (4.0 * distance) / (3.0 * delta_T);
    } else {
      double t_dec = elapsed_time - 3.0 * delta_T_us / 4.0;
      velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * ((delta_T / 4.0) - t_dec / 1e6);
    }


    double velocity_error_L = -velocity_setpoint - vL();
    double velocity_error_R = -velocity_setpoint - vR();


    left_pwm  += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R;


    left_pwm  = constrain(left_pwm,  -400, -pwm_min);
    right_pwm = constrain(right_pwm, -400, -pwm_min);
    right_pwm -= kPs * ang();

    motors.setSpeeds(left_pwm, right_pwm);
  }


  motors.setSpeeds(0, 0);
  reset();
  delay(15);
}


void end(double d) {
  // Original logic if you want it separate, 
  reset();
  double distance   = end_distance + d;
  double t0         = micros();
  double delta_T    = ((targetTime *1e6 + start_time) - t0) / 1e6;
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

    if(delta_T > 0){
      velocity_setpoint = (distance-dL()) / (delta_T-elapsed_time/1e6);
    }else{
      velocity_setpoint = 40;
    }
    double velocity_error_L = velocity_setpoint - vL();
    double velocity_error_R = velocity_setpoint - vR();


    left_pwm  += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R;


    left_pwm  = constrain(left_pwm,  pwm_min, 400);
    right_pwm = constrain(right_pwm, pwm_min, 400);
    right_pwm -= kPs * ang();

    motors.setSpeeds(left_pwm, right_pwm);
  }


  motors.setSpeeds(0, 0);
  delay(10);
  reset();
}


// -------------------------------------------------------------------------------------------------
// Original near-90 left() / right() – angle-based + final time wait
// -------------------------------------------------------------------------------------------------
void left2() {
  int starting = millis();
  delay(250);
  reset();
  while (ang() < left_angle) {
    update();
    motors.setSpeeds(-pwm_min - abs(90 - (ang())) * Kpt, pwm_min + abs(90 - (ang())) * Kpt);
  }
  motors.setSpeeds(0, 0);
  delay(turnTime > (millis() - starting) ? turnTime - (millis() - starting) : 0);
  reset();
}
void left() {
  int starting = millis();
  delay(250);
  reset();
  double t0         = micros();
  double delta_T    =  turnTime - 500;
  double delta_T_us = delta_T * 1e6;


  double left_pwm   = -pwm_min;
  double right_pwm  = pwm_min;


  double velocity_setpoint = 0;
  double elapsed_time;


  while (true) {
    elapsed_time = micros() - t0;
    update();


    if (ang() > left_angle) {
      break;
    }


    if (elapsed_time <= delta_T_us / 4.0) {
      velocity_setpoint = (16.0 * left_angle) / (3.0 * delta_T * delta_T)  * (elapsed_time / 1e6);
    } else if (elapsed_time <= 3.0 * delta_T_us / 4.0) {
      velocity_setpoint = (4.0 * left_angle) / (3.0 * delta_T);
    } else {
      double t_dec = elapsed_time - 3.0 * delta_T_us / 4.0;
      velocity_setpoint = (16.0 * left_angle) / (3.0 * delta_T * delta_T) * ((delta_T / 4.0) - t_dec / 1e6);
    }


    double velocity_error_L = -velocity_setpoint - aV();
    double velocity_error_R = velocity_setpoint - aV();


    left_pwm  += Kpt * velocity_error_L;
    right_pwm += Kpt * velocity_error_R;


    left_pwm  = constrain(left_pwm,  -300, -pwm_min);
    right_pwm = constrain(right_pwm, pwm_min, 300);

    motors.setSpeeds(left_pwm, right_pwm);
  }


  motors.setSpeeds(0, 0);
  reset();
  delay(turnTime > (millis() - starting) ? turnTime - (millis() - starting) : 0);

}

void right() {
  reset();
  int starting = millis();
  delay(250);
  while (ang() > -right_angle) {
    update();
    motors.setSpeeds(pwm_min + abs(90 + (ang())) * Kpt, -pwm_min - abs(90 + (ang())) * Kpt);
  }
  motors.setSpeeds(0, 0);
  delay(turnTime > (millis() - starting) ? turnTime - (millis() - starting) : 0);
  reset();
}


// -------------------------------------------------------------------------------------------------
// NEW: time-based turning for commands like L100 or R50
// -------------------------------------------------------------------------------------------------
void leftTime(double deg)
{
  // scale factor based on ratio to your "normal" left_angle
  double scale = deg / left_angle;
  if (scale < 0.1) scale = 0.1; // don't let scale get too tiny


  int starting = millis();
  delay(100);


  while (ang() < deg) {
    update();
    double error = fabs(deg - ang());
    double pwm   = pwm_min + error * Kpt;
    if (pwm > 400) pwm = 400;
    motors.setSpeeds(0, pwm);
  }
  motors.setSpeeds(0, 0);


  // scale the time-based wait
  while (millis() - starting < (int)(turnTime * scale)) {
    motors.setSpeeds(0, 0);
  }
  reset();
}


void rightTime(double deg)
{
  double scale = deg / right_angle;
  if (scale < 0.1) scale = 0.1;


  int starting = millis();
  delay(100);


  while (ang() > -deg) {
    update();
    double error = fabs(-deg - ang());
    double pwm   = pwm_min + error * Kpt;
    if (pwm > 400) pwm = 400;
    motors.setSpeeds(pwm, 0);
  }
  motors.setSpeeds(0, 0);


  while (millis() - starting < (int)(turnTime * scale)) {
    motors.setSpeeds(0, 0);
  }
  reset();
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
    } else {
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
     // Subtract BOT_RADIUS if there was a turn before

      
      
      // Ensure distance is non-negative
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
      end(0);
      ptr++;
    } else {
      ptr++; // Skip unrecognized characters
     
    }
  }
}


// -------------------------------------------------------------------------------------------------
// Pololu gyro functions (unchanged except naming maybe)
// -------------------------------------------------------------------------------------------------
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}


void turnSensorUpdate()
{
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;


  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;


  int32_t d = (int32_t)turnRate * dt;
  // Pololu recommended scaling
  turnAngle += (int64_t)d * 14680064 / 17578125;
}


double ang()
{
  // convert from 32-bit turnAngle to degrees
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


  ledYellow(1);
  delay(500);


  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    while(!imu.gyroDataReady()) {}
    imu.readGyro();
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;


  display.clear();
  turnSensorReset();
  turnSensorUpdate();
  display.gotoXY(0, 0);
  display.print(turnAngle);
  display.print(F("   "));
  display.clear();
}


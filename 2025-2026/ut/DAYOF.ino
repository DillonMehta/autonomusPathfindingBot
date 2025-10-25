// Includes ------------------------------------------------------------------------------------------------------------------------
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
// Don't Touch ----------------------------------------------------------------------------------------------------------
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.22;
const float BOT_RADIUS = 4.2; // cm horizontal radius(wheels to center)

// Global variables for gyro-based turning
double turnAngle = 0;
double turnRate = 0;
double gyroOffset = 0;
double gyroLastUpdate = 0;
double velocity_error_R;
double velocity_error_L;
// dillon new
double total_fwd = 0;
double delayer_amt;

// Misc. distances
double lt0, ls0, rt0, rs0;
double start_time = 0;
double total_distance = 0;
volatile double eCount = 0;
volatile double eCount2 = 0;
double total_turns = 0;
int total_pivots = 0;
// slow turns
double left_slip_factor = 1.09;
double right_slip_factor = 1.14;
double velocity_setpoint = 20.0; // cm/s
const double pwm_min = 35; // minimal PWM for movement
double Kpt = 0;            // proportional factor for turning  
const double turnTime = 600;
double left_ang = 88;
double right_ang = 89;
// straight
double kPs = 0.2; // small angle correction for going straight
double kP = 0.4;  // for velocity control
double swerve_kD = 1.5;//0.1;
double str_min = 45;
double mehta_sahni_constant = 0.082;

// Movement Values (Change here) ------------------------------------------------------------------------------------------------------------------------
double mass = 100;  //grams, scale factor
double targetTime = 10; // subtract 1.75 seconds for every 50cm the end has, including the dist_diff
// 20 seconds for turn cali codes
double end_distance = 44.5; //
double end_delay = 0;
double time_diff = 0; // seconds
double ang_diff = 0;  // left is positive
double dist_diff = 0; // forwawrd is positive
// 30.5 is enter
// fwd before and after pivot subtract 4.2 (bot radius) IE, F45.8

//bf 8.5 af 6.5
// for cali use 30 sec for 2 squares and 9 seconds for 4 backs
char movement[400] = "R R R R R R R R R R R R R R R R";
//"F30.5 R F100 L F30 B30 L F50 R F145.8 > F45.8 B150 R F50 R F45.8 < F105.8 B10 L F50 L F145.8 > F45.8  B55.5";
//"F80.3 R F50 L F50 L F100 R F50 B F50 L F100 L F50 R F50 R F50 L F50 L F50 B F150 R F50 R F50 B F50 L F50 L F100 L F100 L F50 R F100 L F50 L F50 L F50 R F50 L F50 L F50 E";
// F30.3 F50 R F50 L F50 L F100 R F50 B F50 L F100 L F50 R F50 R F50 L F50 L F50 B F150 R F50 R F50 B F50 L F50 L F100 L F100 L F50 R F100 L F50 L F50 L F50 R F50 L F50 L F50 E
// F50 L F50 L F50 L F50 L F50 L F50 L F50 L F50 L;
// F50 R F50 R F50 R F50 R F50 R F50 R F50 R F50 R;
// F30.3 R F50 L F100 R F50 L F30 B F30 R F200 L F100 L F50 L F150 R F100 R F150 L F50 L F30 B F30 R F50 R F150 L E
// F50 B F50 B F50 B F50 B

// setup ================================================================================
void setup()
{
  display.clear();

  display.print("Bat: ");
  display.print(readBatteryMillivolts()/1000.0, 2); // 2 decimal places
  display.print(" V");
  display.display();
  delay(500);
  display.clear();
  turnSensorSetup();
  turnSensorReset();
  calculateTotalTurns(movement);
  calculateTotalFwd(movement);
  delayer_amt = (time_diff * 1000) / (total_turns + total_fwd);
  calculateTotalDistance(movement);
  start_time = micros();
  buzzer.playFrequency(440, 200, 15);
  buzzer.playFrequency(440, 200, 15);
  delay(50);
  processCommands(movement);
  buzzer.playFrequency(440, 200, 15);
  buzzer.playFrequency(440, 200, 15);

  delay(5000);
}
//
void update()
{
  turnSensorUpdate();
  eCount = encoders.getCountsLeft();
  eCount2 = encoders.getCountsRight();
}
void reset()
{
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  turnSensorReset();
  turnAngle = 0;
}
double dL() { return eCount / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE; }
double dR() { return eCount2 / (CLICKS_PER_ROTATION * GEAR_RATIO) * (WHEEL_CIRCUMFERENCE - mehta_sahni_constant); }
double vL()
{
  double vel = (dL() - ls0) / (micros() - lt0) * 1000000;
  ls0 = dL();
  lt0 = micros();
  return vel;
}
double vR()
{
  double vel = (dR() - rs0) / (micros() - rt0) * 1000000;
  rs0 = dR();
  rt0 = micros();
  return vel;
}
void fwd(double distance)
{
  update();
  double t0 = micros(); // Start time in microseconds
  double delta_T = findTime(distance);
  double delta_T_us = delta_T * 1e6; // Convert delta_T from seconds to microseconds
  double left_pwm = str_min;
  double right_pwm = str_min;
  double velocity_setpoint = 0; // Initialize the velocity setpoint
  double elapsed_time;

  // Main control loop
  while (true)
  {
    elapsed_time = micros() - t0; // Elapsed time in microseconds
    update();
    // Exit condition: Distance has been covered or time has exceeded delta_T
    if (dL() >= distance)
    {
      break;
    }
    if (distance < 40){
      velocity_setpoint = distance/delta_T;
    }else{
      // Determine velocity setpoint based on elapsed time
      if (elapsed_time <= delta_T_us / 4)
      {
        // Acceleration phase
        velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * (elapsed_time / 1e6);
      }
      else if (elapsed_time <= 3 * delta_T_us / 4)
      {
        // Constant velocity phase
        velocity_setpoint = (4.0 * distance) / (3.0 * delta_T);
      }
      else
      {
        // Deceleration phase
        double t_dec = elapsed_time - 3 * delta_T_us / 4;
        velocity_setpoint = (16.0 * distance) / (3.0 * delta_T * delta_T) * ((delta_T / 4) - t_dec / 1e6);
      }
    }

    // Update PWM values based on velocity feedback and setpoint
    velocity_error_L = velocity_setpoint - vL();
    velocity_error_R = velocity_setpoint - vR();

    left_pwm += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R;

    // Constrain PWM values to valid range
    left_pwm = constrain(left_pwm, str_min, 400);
    right_pwm = constrain(right_pwm, str_min, 400);
    right_pwm -= kPs * ang();
    // Set motor speeds
    motors.setSpeeds(left_pwm, right_pwm);
  }

  // Stop motors at the end
  motors.setSpeeds(0, 0);
  delay(10 + delayer_amt); // Small delay to ensure stop
  reset();                 // Reset necessary parameters
}

void end(double d)
{
  motors.setSpeeds(0, 0);
  reset();
  delay(200);
  double distance = end_distance + d;
  double t0 = micros();
  double delta_T = (((targetTime * 1e6 + start_time) - t0) / 1e6);
  double delta_T_us = delta_T * 1e6;
  if (findTime(distance) > delta_T)
  {
    fwd(distance);
  }
  else
  {
    double left_pwm = 35;
    double right_pwm = 35 + 0.08;
    double velocity_setpoint = (distance) / (delta_T);
    double elapsed_time;

    while (true)
    {
      elapsed_time = micros() - t0;
      update();
      if (dL() >= distance)
      {
        break;
      }
      velocity_error_L = velocity_setpoint - vL();
      velocity_error_R = velocity_setpoint - vR();

      left_pwm += kP * velocity_error_L;
      right_pwm += kP * velocity_error_R;
      left_pwm = constrain(left_pwm, 35, 400);
      right_pwm = constrain(right_pwm, 35, 400);
      right_pwm -= kPs * ang();

      motors.setSpeeds(left_pwm, right_pwm);
    }
  }
  motors.setSpeeds(0, 0);
  if (ang_diff > 0)
  {
    delay(100);
    while (ang() < ang_diff)
    {
      update();
      motors.setSpeeds(-pwm_min - abs(ang_diff - (ang())) * Kpt, pwm_min + abs(ang_diff - (ang())) * Kpt);
    }
  }
  else if (ang_diff < 0)
  {
    delay(100);
    while (ang() > ang_diff)
    {
      update();
      motors.setSpeeds(pwm_min + abs(-ang_diff + (ang())) * Kpt, -pwm_min - abs(-ang_diff + (ang())) * Kpt);
    }
  }
  fwd(dist_diff);
  display.clear();
  display.print(dL(), 2);
  delay(500);
  display.clear();
  display.print((micros() - start_time) / 1e6, 2);
}

void back(double distance) {
  update();
  double t0 = micros();
  double delta_T = findTime(distance);
  double delta_T_us = delta_T * 1e6;
  double left_pwm = -str_min;
  double right_pwm = -str_min;
  double velocity_setpoint = 0;
  double elapsed_time;

  while (true) {
    elapsed_time = micros() - t0;
    update();
    if (-dL() >= distance) {
      break;
    }

    if (elapsed_time <= delta_T_us / 4) {
      velocity_setpoint = - (16.0 * distance) / (3.0 * delta_T * delta_T) * (elapsed_time / 1e6);
    } else if (elapsed_time <= 3 * delta_T_us / 4) {
      velocity_setpoint = - (4.0 * distance) / (3.0 * delta_T);
    } else {
      double t_dec = elapsed_time - 3 * delta_T_us / 4;
      velocity_setpoint = - (16.0 * distance) / (3.0 * delta_T * delta_T) * ((delta_T / 4) - t_dec / 1e6);
    }

    velocity_error_L = velocity_setpoint - vL();
    velocity_error_R = velocity_setpoint - vR();

    left_pwm += kP * velocity_error_L;
    right_pwm += kP * velocity_error_R;

    left_pwm = constrain(left_pwm, -400, -str_min);
    right_pwm = constrain(right_pwm, -400, -str_min);
    right_pwm -= kPs * ang();
    motors.setSpeeds(left_pwm, right_pwm);
  }

  motors.setSpeeds(0, 0);
  delay(10 + delayer_amt);
  reset();
}
void left()
{
  int starting = millis();
  update();
  delay(200);
  while (-dL() + dR() < BOT_RADIUS * 3.14159)
  {
    update();
    motors.setSpeeds(-pwm_min, pwm_min);
  }
  motors.setSpeeds(0, 0);
  // delay(turnTime > (millis() - starting) ? turnTime - (millis() - starting) + 150: 150);
  delay(200);
  reset();
}
void pivotLeft()
{
  int starting = millis();
  reset();
  update();
  delay(200);
  double right_pwm = 100;
  double velocity_error_R;
  while (ang() < left_ang) /*dR() < BOT_RADIUS * 3.14159*left_slip_factor*mass/100.0*/
  {
    update();
    velocity_error_R = vR() *left_slip_factor*mass/100.0 - velocity_setpoint;
    right_pwm -= swerve_kD * velocity_error_R;
    motors.setSpeeds(0, right_pwm);
  }
  motors.setSpeeds(0, 0);
  // delay(turnTime > (millis() - starting) ? turnTime - (millis() - starting) + 150: 150);
  delay(100);
  reset();

}

void left(int val) {
  int starting = millis();
  delay(100);
  reset();
  while (ang() < val-3) {
    update();
    motors.setSpeeds(-pwm_min - abs(val - (ang())) * Kpt, pwm_min + abs(val - (ang())) * Kpt);
  }
  motors.setSpeeds(0, 0);
  
  delay(100);
  reset();
}
void right()
{
  int starting = millis();
  update();
  delay(200);
  while (dL() + -dR() < BOT_RADIUS * 3.14159)
  {
    update();
    motors.setSpeeds(pwm_min, -pwm_min);
  }
  motors.setSpeeds(0, 0);
  // delay(turnTime > (millis() - starting) ? turnTime - (millis() - starting) + 150: 150);
  delay(200);
  reset();
}
void pivotRight()
{
  int starting = millis();
  reset();
  update();
  delay(200);
  double left_pwm = 100;
  double velocity_error_L;
  while (ang() > -right_ang /*dL() < BOT_RADIUS * 3.14159*right_slip_factor*mass/100.0*/)
  {
    update();
    velocity_error_L = vL()*right_slip_factor*mass/100.0 - velocity_setpoint;
    left_pwm -= swerve_kD * velocity_error_L;
    motors.setSpeeds(left_pwm,0);
  }
  motors.setSpeeds(0, 0);
  // delay(turnTime > (millis() - starting) ? turnTime - (millis() - starting) + 150: 150);
  delay(100);
  reset();

}
void right(int val) {
  reset();
  int starting = millis();
  delay(150);
  reset();
  while (ang() > -val+3) {
    update();
    motors.setSpeeds(pwm_min + abs(val + (ang())) * Kpt, -pwm_min - abs(val + (ang())) * Kpt);
  }
  motors.setSpeeds(0, 0);
  delay(250);
  reset();
}


// helper functions -----------------------------------------------------------------------

void loop()
{
}
double findTime(double distance)
{
  return (targetTime - total_turns * (turnTime) / 1e3 - total_pivots * PI * BOT_RADIUS /velocity_setpoint) * (distance / total_distance); // seconds
}
void calculateTotalDistance(const char *commands)
{
  const char *ptr = commands;
  while (*ptr != '\0')
  {
    if (*ptr == 'F' || *ptr == 'B')
    {
      ptr++;
      float distance = 0.0;
      float factor = 1.0;
      bool isDecimal = false;
      while ((*ptr >= '0' && *ptr <= '9') || *ptr == '.')
      {
        if (*ptr == '.')
        {
          isDecimal = true;
          factor = 0.1;
        }
        else if (!isDecimal)
        {
          distance = distance * 10 + (*ptr - '0');
        }
        else
        {
          distance += (*ptr - '0') * factor;
          factor *= 0.1;
        }
        ptr++;
      }
      total_distance += distance;
    }
    else if (*ptr == ' G')
    {
      total_distance += 70;
    }
    else
    {
      ptr++;
    }
  }
  total_distance += end_distance;
  total_distance -= total_turns * 2 * BOT_RADIUS;
}
void calculateTotalTurns(const char *commands)
{
  const char *ptr = commands;
  while (*ptr != '\0')
  {
    if (*ptr == 'L' || *ptr == 'R')
    {
      total_turns++;
    }
    else if (*ptr == '<' || *ptr == '>')
    {
      total_pivots++;
    }
    ptr++;
  }
}
void calculateTotalFwd(const char *commands)
{
  const char *ptr = commands;
  while (*ptr != '\0')
  {
    if (*ptr == 'F' || *ptr == 'B')
    {
      total_fwd++;
    }
    ptr++;
  }
}

void processCommands(const char* commands) {
  const char* ptr = commands; // Pointer to traverse the char array
   // Flag to track if the previous command was a turn
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

      if (cmd == 'F') {

        fwd(distance);

      } else {

        back(distance);

      }
    } else if (*ptr == 'L') {
      ptr++;
      float turn_angs = 0.0;
      float factor = 1.0;
      bool isDecimal = false;
      while ((*ptr >= '0' && *ptr <= '9') || *ptr == '.') { // Extract number
        if (*ptr == '.') {
          isDecimal = true;
          factor = 0.1;
        } else if (!isDecimal) {
          turn_angs = turn_angs * 10 + (*ptr - '0');
        } else {
          turn_angs += (*ptr - '0') * factor;
          factor *= 0.1;
        }
        ptr++;
      }
      if (true){
        left();
      }else{
        left(turn_angs);
      }
      ptr++;

    } else if (*ptr == 'R') {
      ptr++;
      float turn_angs = 0.0;
      float factor = 1.0;
      bool isDecimal = false;
      while ((*ptr >= '0' && *ptr <= '9') || *ptr == '.') { // Extract number
        if (*ptr == '.') {
          isDecimal = true;
          factor = 0.1;
        } else if (!isDecimal) {
          turn_angs = turn_angs * 10 + (*ptr - '0');
        } else {
          turn_angs += (*ptr - '0') * factor;
          factor *= 0.1;
        }
        ptr++;
      }
      if (true){
        right();
      }else{
        right(turn_angs);
      }
      ptr++;
    } else if (*ptr == 'E') {

        end(0);//-BOT_RADIUS

      ptr++;
    } else if (*ptr == 'D') {
      ptr++;
      float delaytime = 0.0;
      float factor = 1.0;
      bool isDecimal = false;
      while ((*ptr >= '0' && *ptr <= '9') || *ptr == '.') { // Extract number
        if (*ptr == '.') {
          isDecimal = true;
          factor = 0.1;
        } else if (!isDecimal) {
          delaytime = delaytime * 10 + (*ptr - '0');
        } else {
          delaytime += (*ptr - '0') * factor;
          factor *= 0.1;
        }
        ptr++;
      }
      if (delaytime==0){
        delay(500);
      }else{
        delay(delaytime);
      }
      ptr++;
    } 
    else if (*ptr == '<') {

      pivotLeft();//-BOT_RADIUS

      ptr++;
    } 
    else if (*ptr == '>') {

      pivotRight();//-BOT_RADIUS

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
  if(fabs(turnRate) * 0.07 < 0.1){return;}

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;
  
  double d = turnRate * 0.07 * dt/1e6;

  turnAngle += d;
}
/*
int32_t ang(){ return ((((int32_t)turnAngle >> 16) * 360) >> 16);
  }
*/
double ang()
{
  return turnAngle;
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
    while (!imu.gyroDataReady())
    {
    }
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

/* This demo shows how the 3pi+ can use its gyroscope to detect
when it is being rotated, and use the motors to resist that
rotation.

Be careful to not move the robot for a few seconds after starting
it while the gyro is being calibrated.  During the gyro
calibration, the yellow LED is on and the words "Gyro cal" are
displayed on the display.

After the gyro calibration is done, press button A to start the
demo.  If you try to turn the 3pi+, or put it on a surface that
is turning, it will drive its motors to counteract the turning.

This demo only uses the Z axis of the gyro, so it is possible to
pick up the 3pi+, rotate it about its X and Y axes, and then put
it down facing in a new position. */

#include <Wire.h>
#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>

/* The IMU is not fully enabled by default since it depends on the
Wire library, which uses about 1400 bytes of additional code space
and defines an interrupt service routine (ISR) that might be
incompatible with some applications (such as our TWISlave example).

Include Pololu3piPlus32U4IMU.h in one of your cpp/ino files to
enable IMU functionality.
*/
#include <Pololu3piPlus32U4IMU.h>

using namespace Pololu3piPlus32U4;

// Change next line to this if you are using the older 3pi+
// with a black and green LCD display:
// LCD display;
OLED display;

Encoders encoders;
Buzzer buzzer;
Motors motors;
IMU imu;
//Button A buttonA;

#include "TurnSensor.h"

unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 5;

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

int ang = 0;

int16_t maxSpeed = 100;

const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.05; //10.0531

float Sl = 0.0F;
float Sr = 0.0F;

int lastError;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(57600);
  delay(1000);

  turnSensorSetup();
  turnSensorReset();

  move(50);
  move(50);

}

void loop() {
  // put your main code here, to run repeatedly:
}

void move(float dist) {

  int in = turnAngle;
  int diff;

  while (Sr < dist) {

    currentMillis = millis();

    if (currentMillis > prevMillis + PERIOD) {

      countsLeft += encoders.getCountsAndResetLeft();
      countsRight += encoders.getCountsAndResetRight();

      Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) *  WHEEL_CIRCUMFERENCE);
      Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) *  WHEEL_CIRCUMFERENCE);

      float subtract = dist - (dist / 70 * 15);

      int rWheel = 40;
      int lWheel = 40;
      int newSpeed = 40;

      diff = countsLeft - countsRight;

      if (Sr < dist){

        newSpeed =  max(rWheel, rWheel * (diff / 2) * 1.5);

        motors.setSpeeds(lWheel, newSpeed);
      } else {
        motors.setSpeeds(0, 0);
      }

      prevLeft = countsLeft;
      prevRight = countsRight;
      prevMillis = currentMillis;

    }

    turnSensorUpdate();
    display.gotoXY(0, 0);
    display.print(diff);
    display.print(F("   "));

  }

  delay(2000);

  Sr = 0.0F;
  Sl = 0.0F;
  countsLeft = 0;
  countsRight = 0;
  prevLeft = 0;
  prevRight = 0;
  prevMillis = 0;
  currentMillis = 0;


}

void t() {

  float dist = 6.485;

  while (Sr < dist) {

    currentMillis = millis();

    if (currentMillis > prevMillis + PERIOD) {

      countsLeft += encoders.getCountsAndResetLeft();
      countsRight += encoders.getCountsAndResetRight();

      Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) *  WHEEL_CIRCUMFERENCE);
      Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) *  WHEEL_CIRCUMFERENCE);

      int wheelSpeed = 35;
      float subtract = dist - (dist / 10 * 15);

      if (Sr < dist){
        if (Sr > subtract) {
          wheelSpeed = 35 * ((15 - Sr) / 10);
          if (wheelSpeed < 20) wheelSpeed = 20;
        }
        motors.setSpeeds(-wheelSpeed, wheelSpeed);
      } else {
        motors.setSpeeds(0, 0);
      }

      prevLeft = countsLeft;
      prevRight = countsRight;
      prevMillis = currentMillis;

    }

    turnSensorUpdate();
    display.gotoXY(0, 0);
    display.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    display.print(F("   "));

  }

  delay(2000);
  Sr = 0;
  Sl = 0;
  countsLeft = 0;
  countsRight = 0;
  prevLeft = 0;
  prevRight = 0;
  prevMillis = 0;
  currentMillis = 0;

}

void left() {

  delay(500);
  ang += 90;

  if (ang == 180) {
    ang = -180;
  }

  while (ang != (((int32_t)turnAngle >> 16) * 360) >> 16){

    turnSensorUpdate();
    display.gotoXY(0, 0);
    display.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    display.print(F("   "));

    motors.setSpeeds(-30, 30);
  
  }

  motors.setSpeeds(0, 0);
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  turnAngle += 90;
  delay(2000);

}

void right() {

  delay(500);
  ang -= 90;

  if (ang == 180) {
    ang = -180;
  }

  while (ang != (((int32_t)turnAngle >> 16) * 360) >> 16){

    turnSensorUpdate();
    display.gotoXY(0, 0);
    display.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    display.print(F("   "));

    motors.setSpeeds(30, -30);
  
  }

  motors.setSpeeds(0, 0);
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  turnAngle -= 90;
  delay(2000);

}

void tdfhd() {

  turnSensorReset();
  int final = ((((int32_t)turnAngle >> 16) * 360) >> 16) + 90;
  turnAngle = turnAngle + 90;

  while (final > (((int32_t)turnAngle >> 16) * 360) >> 16){

    turnSensorUpdate();

    int32_t turnSpeed = -(int32_t)turnAngle / (turnAngle1 / 28)
    - turnRate / 40;

    turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);

    motors.setSpeeds(-turnSpeed, turnSpeed);
  
  }

  display.clear();
  motors.setSpeeds(0, 0);
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  delay(2000);

}

void checkEncoders() {

  currentMillis = millis();

  if (currentMillis > prevMillis + PERIOD) {

    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) *  WHEEL_CIRCUMFERENCE);
    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) *  WHEEL_CIRCUMFERENCE);

    int wheelSpeed = 50;

    if (Sr < 50){
      if (Sr > 40){
        wheelSpeed = 50 * ((30 - Sr) / 10);
        if (wheelSpeed < 20) wheelSpeed = 20;
      }
      motors.setSpeeds(wheelSpeed, wheelSpeed);
    } else {
      motors.setSpeeds(0, 0);
    }

    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMillis = currentMillis;

  }

}



Other Stuff:




// Turnsensor.h provides functions for configuring the
// 3pi+ 32U4's gyro, calibrating it, and using it to
// measure how much the robot has turned about its Z axis.
//
// This file should be included *once* in your sketch,
// somewhere after you define objects named buttonA,
// display, and imu.

#include <Wire.h>

// This constant represents a turn of 45 degrees.
const int32_t turnAngle45 = 0x20000000;

// This constant represents a turn of 90 degrees.
const int32_t turnAngle90 = turnAngle45 * 2;

// This constant represents a turn of approximately 1 degree.
const int32_t turnAngle1 = (turnAngle45 + 22) / 45;

/* turnAngle is a 32-bit unsigned integer representing the amount
the robot has turned since the last time turnSensorReset was
called.  This is computed solely using the Z axis of the gyro, so
it could be inaccurate if the robot is rotated about the X or Y
axes.

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

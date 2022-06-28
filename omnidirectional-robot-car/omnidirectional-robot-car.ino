#include <math.h>
#include <AFMotor.h>

#include "PS2_Controller.h"

AF_DCMotor motorBackLeft(1);   // Motor controlling Back-Left wheel
AF_DCMotor motorBackRight(2);  // Motor controlling Back-Right wheel
AF_DCMotor motorFrontRight(3); // Motor controlling Front-Right wheel
AF_DCMotor motorFrontLeft(4);  // Motor controlling Front-Left wheel

PS2_Controller ps2_controller;  // Interacts with the PS2 Controller

bool in_failed_state = false;

void setup()
{
  // Start serial communication
  Serial.begin(9600);
  Serial.println("");

  // Setup pins and ports: configure(clock, attention, command, data)
  ps2_controller.configure(A0, A1, A2, A3);
  delay(1000);

  in_failed_state = !ps2_controller.is_supported();
}

void loop()
{
  if (in_failed_state)
  {
    ps2_controller.configure(A0, A1, A2, A3);
    delay(1000);

    in_failed_state = !ps2_controller.is_supported();

    return;
  }

  if (!ps2_controller.read_data())
  {
    delay(100);
    return;
  }

  // Considers both 127 and 128 to be "neutral" values (ie. default position).
  double neutralValue = (255 - 0) / 2.0;

  double joyLeftX  = -ps2_controller.get_joystick_value(PSS_LX) + neutralValue;
  double joyLeftY  =  ps2_controller.get_joystick_value(PSS_LY) - neutralValue;
  double joyRightX = -ps2_controller.get_joystick_value(PSS_RX) + neutralValue; // Used for rotation in-place
  double joyRightY =  ps2_controller.get_joystick_value(PSS_RY) - neutralValue; // Currently unused

  joyLeftX  = abs(joyLeftX)  > 1.0 ? joyLeftX  : 0.0;
  joyLeftY  = abs(joyLeftY)  > 1.0 ? joyLeftY  : 0.0;
  joyRightX = abs(joyRightX) > 1.0 ? joyRightX : 0.0;
  joyRightY = abs(joyRightY) > 1.0 ? joyRightY : 0.0;

  double angleJoyLeft     = atan2(joyLeftY, joyLeftX);
  double magnitudeJoyLeft = sqrt(joyLeftY * joyLeftY + joyLeftX * joyLeftX);

  double velocityFrontLeft  = sin(angleJoyLeft + M_PI/4) * magnitudeJoyLeft + joyRightX;
  double velocityBackLeft   = sin(angleJoyLeft - M_PI/4) * magnitudeJoyLeft + joyRightX;
  double velocityFrontRight = sin(angleJoyLeft - M_PI/4) * magnitudeJoyLeft - joyRightX;
  double velocityBackRight  = sin(angleJoyLeft + M_PI/4) * magnitudeJoyLeft - joyRightX;

  // Substract multiples of M_PI/2 so that the simplified angle is just the
  // remainder less than M_PI/2.
  double simplifiedAngle = abs(angleJoyLeft) - (uint8_t(abs(angleJoyLeft) / (M_PI/2)) * M_PI/2);

  // The max speed accounts for the fact that we are scaling a square with a
  // side length of neutral value to a circle with a radius of neutral value.
  double maxSpeed = neutralValue;

  // If the simplified angle is closer to the X axis, then use cos(...).
  // If the simplified angle is closer to the Y axis, then use sin(...).
  // Otherwise, it must be equal to M_PI/4 (aka. 45 deg), so use either.
  if (simplifiedAngle < M_PI/4)
  {
    maxSpeed = abs(neutralValue / cos(simplifiedAngle));
  }
  else
  {
    maxSpeed = abs(neutralValue / sin(simplifiedAngle));
  }

  // Wheels won't spin unless the speed is at least 100 ish.
  uint8_t speedFrontLeft  = map(abs(velocityFrontLeft),  0.0, maxSpeed, 100, 255);
  uint8_t speedBackLeft   = map(abs(velocityBackLeft),   0.0, maxSpeed, 100, 255);
  uint8_t speedFrontRight = map(abs(velocityFrontRight), 0.0, maxSpeed, 100, 255);
  uint8_t speedBackRight  = map(abs(velocityBackRight),  0.0, maxSpeed, 100, 255);

  uint8_t directionFrontLeft  = (velocityFrontLeft  > 1.0) ? FORWARD : (velocityFrontLeft  < -1.0) ? BACKWARD : RELEASE;
  uint8_t directionBackLeft   = (velocityBackLeft   > 1.0) ? FORWARD : (velocityBackLeft   < -1.0) ? BACKWARD : RELEASE;
  uint8_t directionFrontRight = (velocityFrontRight > 1.0) ? FORWARD : (velocityFrontRight < -1.0) ? BACKWARD : RELEASE;
  uint8_t directionBackRight  = (velocityBackRight  > 1.0) ? FORWARD : (velocityBackRight  < -1.0) ? BACKWARD : RELEASE;

  motorFrontLeft.setSpeed(speedFrontLeft);
  motorBackLeft.setSpeed(speedBackLeft);
  motorFrontRight.setSpeed(speedFrontRight);
  motorBackRight.setSpeed(speedBackRight);

  motorFrontLeft.run(directionFrontLeft);
  motorBackLeft.run(directionBackLeft);
  motorFrontRight.run(directionFrontRight);
  motorBackRight.run(directionBackRight);

  delay(100);

  // Serial.print("joyLeftX = ");  Serial.print(joyLeftX);  Serial.print(", ");
  // Serial.print("joyLeftY = ");  Serial.print(joyLeftY);  Serial.print(", ");
  // Serial.print("joyRightX = "); Serial.print(joyRightX); Serial.print(", ");
  // Serial.print("joyRightY = "); Serial.print(joyRightY); Serial.println("");

  // Serial.print("velocityFrontLeft = ");  Serial.print(velocityFrontLeft);  Serial.print(", ");
  // Serial.print("velocityBackLeft = ");   Serial.print(velocityBackLeft);   Serial.print(", ");
  // Serial.print("velocityFrontRight = "); Serial.print(velocityFrontRight); Serial.print(", ");
  // Serial.print("velocityBackRight = ");  Serial.print(velocityBackRight);  Serial.println("");

  // Serial.print("speedFrontLeft = ");  Serial.print(speedFrontLeft);  Serial.print(", ");
  // Serial.print("speedBackLeft = ");   Serial.print(speedBackLeft);   Serial.print(", ");
  // Serial.print("speedFrontRight = "); Serial.print(speedFrontRight); Serial.print(", ");
  // Serial.print("speedBackRight = ");  Serial.print(speedBackRight);  Serial.println("");

  // Serial.print("angleJoyLeft = ");     Serial.print(angleJoyLeft);     Serial.println("");
  // Serial.print("magnitudeJoyLeft = "); Serial.print(magnitudeJoyLeft); Serial.println("");
  // Serial.print("maxSpeed = ");         Serial.print(maxSpeed);         Serial.println("");

  // Serial.print("directionFrontLeft = ");  Serial.print(directionFrontLeft);  Serial.print(", ");
  // Serial.print("directionBackLeft = ");   Serial.print(directionBackLeft);   Serial.print(", ");
  // Serial.print("directionFrontRight = "); Serial.print(directionFrontRight); Serial.print(", ");
  // Serial.print("directionBackRight = ");  Serial.print(directionBackRight);  Serial.println("");
}

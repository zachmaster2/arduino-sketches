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

  double angleJoyLeft = atan2(joyLeftY, joyLeftX);

  double magnitudeJoyLeft = sqrt(joyLeftY * joyLeftY + joyLeftX * joyLeftX);

  double velocityFrontLeft  = sin(angleJoyLeft + M_PI/4) * magnitudeJoyLeft + joyRightX;
  double velocityBackLeft   = sin(angleJoyLeft - M_PI/4) * magnitudeJoyLeft + joyRightX;
  double velocityFrontRight = sin(angleJoyLeft - M_PI/4) * magnitudeJoyLeft - joyRightX;
  double velocityBackRight  = sin(angleJoyLeft + M_PI/4) * magnitudeJoyLeft - joyRightX;

  double speedFrontLeft  = abs(velocityFrontLeft);
  double speedBackLeft   = abs(velocityBackLeft);
  double speedFrontRight = abs(velocityFrontRight);
  double speedBackRight  = abs(velocityBackRight);

  // Substract multiples of M_PI so that the simplified angle is just the
  // remainder less than M_PI.
  double simplifiedAngle = abs(angleJoyLeft) - (uint8_t(abs(angleJoyLeft) / M_PI) * M_PI);

  // If the simplified angle is greater than M_PI, then subtract M_PI/2 so
  // that: M_PI/2 <= simplified angle <= 0
  if (simplifiedAngle > M_PI/2)
  {
    simplifiedAngle -= M_PI/2;
  }

  double maxJoyLeftX = neutralValue;
  double maxJoyLeftY = neutralValue;

  // If the simplified angle is closer to the X axis, then use cos(...).
  // If the simplified angle is closer to the Y axis, then use sin(...).
  // Otherwise, it must be equal to M_PI/4 (aka. 45 deg), so use either.
  if (simplifiedAngle < M_PI/4)
  {
    maxJoyLeftX = neutralValue;
    maxJoyLeftY = maxJoyLeftX * tan(simplifiedAngle);
  }
  else
  {
    maxJoyLeftY = neutralValue;
    maxJoyLeftX = maxJoyLeftY * tan((M_PI/2) - simplifiedAngle);
  }

  double maxMagnitudeJoyLeft = sqrt(maxJoyLeftY * maxJoyLeftY + maxJoyLeftX * maxJoyLeftX);

  double maxVelocityFrontLeft  = sin(angleJoyLeft + M_PI/4) * maxMagnitudeJoyLeft + joyRightX;
  double maxVelocityBackLeft   = sin(angleJoyLeft - M_PI/4) * maxMagnitudeJoyLeft + joyRightX;
  double maxVelocityFrontRight = sin(angleJoyLeft - M_PI/4) * maxMagnitudeJoyLeft - joyRightX;
  double maxVelocityBackRight  = sin(angleJoyLeft + M_PI/4) * maxMagnitudeJoyLeft - joyRightX;

  double maxSpeedFrontLeft  = abs(maxVelocityFrontLeft);
  double maxSpeedBackLeft   = abs(maxVelocityBackLeft);
  double maxSpeedFrontRight = abs(maxVelocityFrontRight);
  double maxSpeedBackRight  = abs(maxVelocityBackRight);

  double maxSpeedOverall = max(max(maxSpeedFrontLeft, maxSpeedBackLeft), max(maxSpeedFrontRight, maxSpeedBackRight));

  // Note: The wheels won't spin unless the speed is at least 100 ish.
  uint8_t motorSpeedFrontLeft  = velocityFrontLeft  ? map(speedFrontLeft,  0.0, maxSpeedOverall, 0, 255) : 0;
  uint8_t motorSpeedBackLeft   = velocityBackLeft   ? map(speedBackLeft,   0.0, maxSpeedOverall, 0, 255) : 0;
  uint8_t motorSpeedFrontRight = velocityFrontRight ? map(speedFrontRight, 0.0, maxSpeedOverall, 0, 255) : 0;
  uint8_t motorSpeedBackRight  = velocityBackRight  ? map(speedBackRight,  0.0, maxSpeedOverall, 0, 255) : 0;

  uint8_t motorDirectionFrontLeft  = (velocityFrontLeft  > 1.0) ? FORWARD : (velocityFrontLeft  < -1.0) ? BACKWARD : RELEASE;
  uint8_t motorDirectionBackLeft   = (velocityBackLeft   > 1.0) ? FORWARD : (velocityBackLeft   < -1.0) ? BACKWARD : RELEASE;
  uint8_t motorDirectionFrontRight = (velocityFrontRight > 1.0) ? FORWARD : (velocityFrontRight < -1.0) ? BACKWARD : RELEASE;
  uint8_t motorDirectionBackRight  = (velocityBackRight  > 1.0) ? FORWARD : (velocityBackRight  < -1.0) ? BACKWARD : RELEASE;

  motorFrontLeft.setSpeed(motorSpeedFrontLeft);
  motorBackLeft.setSpeed(motorSpeedBackLeft);
  motorFrontRight.setSpeed(motorSpeedFrontRight);
  motorBackRight.setSpeed(motorSpeedBackRight);

  motorFrontLeft.run(motorDirectionFrontLeft);
  motorBackLeft.run(motorDirectionBackLeft);
  motorFrontRight.run(motorDirectionFrontRight);
  motorBackRight.run(motorDirectionBackRight);

  delay(100);

  // Serial.print("joyLeftX = ");  Serial.print(joyLeftX);  Serial.print(", ");
  // Serial.print("joyLeftY = ");  Serial.print(joyLeftY);  Serial.print(", ");
  // Serial.print("joyRightX = "); Serial.print(joyRightX); Serial.print(", ");
  // Serial.print("joyRightY = "); Serial.print(joyRightY); Serial.println("");

  // Serial.print("angleJoyLeft = "); Serial.print(angleJoyLeft); Serial.println("");

  // Serial.print("magnitudeJoyLeft = "); Serial.print(magnitudeJoyLeft); Serial.println("");

  // Serial.print("velocityFrontLeft = ");  Serial.print(velocityFrontLeft);  Serial.print(", ");
  // Serial.print("velocityBackLeft = ");   Serial.print(velocityBackLeft);   Serial.print(", ");
  // Serial.print("velocityFrontRight = "); Serial.print(velocityFrontRight); Serial.print(", ");
  // Serial.print("velocityBackRight = ");  Serial.print(velocityBackRight);  Serial.println("");

  // Serial.print("maxJoyLeftX = "); Serial.print(maxJoyLeftX); Serial.print(", ");
  // Serial.print("maxJoyLeftY = "); Serial.print(maxJoyLeftY); Serial.println("");

  // Serial.print("maxMagnitudeJoyLeft = "); Serial.print(maxMagnitudeJoyLeft); Serial.println("");

  // Serial.print("maxVelocityFrontLeft = ");  Serial.print(maxVelocityFrontLeft);  Serial.print(", ");
  // Serial.print("maxVelocityBackLeft = ");   Serial.print(maxVelocityBackLeft);   Serial.print(", ");
  // Serial.print("maxVelocityFrontRight = "); Serial.print(maxVelocityFrontRight); Serial.print(", ");
  // Serial.print("maxVelocityBackRight = ");  Serial.print(maxVelocityBackRight);  Serial.println("");

  // Serial.print("maxSpeedOverall = "); Serial.print(maxSpeedOverall); Serial.println("");

  // Serial.print("motorSpeedFrontLeft = ");  Serial.print(motorSpeedFrontLeft);  Serial.print(", ");
  // Serial.print("motorSpeedBackLeft = ");   Serial.print(motorSpeedBackLeft);   Serial.print(", ");
  // Serial.print("motorSpeedFrontRight = "); Serial.print(motorSpeedFrontRight); Serial.print(", ");
  // Serial.print("motorSpeedBackRight = ");  Serial.print(motorSpeedBackRight);  Serial.println("");

  // auto motorDirectionToString = [](int motorDirection) -> String
  // {
  //   switch (motorDirection)
  //   {
  //     case FORWARD:  return "FORWARD";
  //     case BACKWARD: return "BACKWARD";
  //     case RELEASE:  return "RELEASE";
  //     default:       return String(motorDirection);
  //   }
  // };

  // Serial.print("motorDirectionFrontLeft = ");  Serial.print(motorDirectionToString(motorDirectionFrontLeft));  Serial.print(", ");
  // Serial.print("motorDirectionBackLeft = ");   Serial.print(motorDirectionToString(motorDirectionBackLeft));   Serial.print(", ");
  // Serial.print("motorDirectionFrontRight = "); Serial.print(motorDirectionToString(motorDirectionFrontRight)); Serial.print(", ");
  // Serial.print("motorDirectionBackRight = ");  Serial.print(motorDirectionToString(motorDirectionBackRight));  Serial.println("");

  // Serial.println(); Serial.println();

  // delay(2000);
}

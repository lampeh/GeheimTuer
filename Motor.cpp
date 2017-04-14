#include <avr/pgmspace.h>
#include <Arduino.h>
#include "Motor.h"

/*
__attribute__((always_inline))
static void _memcpy_P(void *dest, const void *src, size_t n) {
  for (size_t i = (n >> 2); i > 0; i--) {
    *(uint32_t *)dest = pgm_read_dword(src);
    dest += 4;
    src += 4;
  }

  n -= (n >> 2) * 4;

  while (n--) {
    *(uint8_t *)(dest++) = pgm_read_byte(src++);
  }
}



// set profile pointer, reset counters, set driver inputs
__attribute__((always_inline))
void initDrive(const enum motorDir dir, const struct accelProfile *const profile) {
  accelProfile = profile;

  _memcpy_P(&currentProfile, accelProfile, sizeof(currentProfile));

  if (currentProfile.maxMillis == 0) {
    motorFree();
    motorDisable();

    //debug(F("BUG: invalid accelProfile: maxMillis == 0 - door disabled"));

    doorState = doorError;
    setLeds1(blue, ledSolid);
    digitalWrite(statusLED, HIGH);

    return;
  }

  drivePWM = 0;

  driveTotalMillis = driveProfileMillis = 0;
  driveMillis = currentProfile.stepMillis; // TODO: this is a hackish way to start driving without waiting the first stepMillis interval

  //  motorDir = dir;
  setMotorDir(dir);
}
*/

void Motor::setMotorBits(const bool A, const bool B) {
  digitalWrite(motorPWMPin, LOW);
  digitalWrite(motorInAPin, A);
  digitalWrite(motorInBPin, B);
}

void Motor::_free() {
  setMotorBits(LOW, LOW);
}

void Motor::_brake() {
  _free();
  digitalWrite(motorPWMPin, HIGH);
}

void Motor::_forward() {
  setMotorBits(HIGH, LOW);
}

void Motor::_backward() {
  setMotorBits(LOW, HIGH);
}

void Motor::free() {
  _free();
  //debug(F("Brake released\r\n"));
}

void Motor::brake() {
  _brake();
  //debug(F("Brake engaged\r\n"));
}

void Motor::forward() {
  _forward();
  //debug(F("Forward\r\n"));
}

void Motor::backward() {
  _backward();
  //debug(F("Backward\r\n"));
}

void Motor::disable() {
  // actively pull diag pins low, disables both driver bridges
  digitalWrite(motorDiagAPin, LOW);
  pinMode(motorDiagAPin, OUTPUT);

  digitalWrite(motorDiagBPin, LOW);
  pinMode(motorDiagBPin, OUTPUT);

  //debug(F("Motor driver disabled\r\n"));
}

void Motor::enable() {
  // return control to pull-ups on the driver board
  pinMode(motorDiagAPin, INPUT);
  pinMode(motorDiagBPin, INPUT);

  //debug(F("Motor driver enabled\r\n"));
}

void Motor::setMotorDir(const enum motorDir dir) {
  switch (dir) {
    case motorForward:
      forward();
      break;

    case motorBackward:
      backward();
      break;

    default:
      break;
  }
}


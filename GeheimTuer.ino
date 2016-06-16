#include <WS2812.h>

// MAXPIX+EXTRAPIX must fit into uint8_t
#define MAXPIX 20  // LED stripe length
#define EXTRAPIX 3 // animation buffer, read window moves back and forth EXTRAPIX pixels

// WS2812 output
const int ledPin = 11;

cRGB leds[MAXPIX+EXTRAPIX];
cRGB tmp_leds[MAXPIX]; // temp array for fading and blinking
cRGB *ledptr; // read pointer for WS2812 library, points to start of window

// requires custom changes to accept uint8_t** argument
WS2812 LED(MAXPIX, &ledptr);

enum ledModes { ledSolid, ledFade, ledBlink, ledForward, ledBackward } ledMode;

// mode-dependent state
union ledState { 
  unsigned int ledidx;
  signed int fader;
  bool blink_on;
} ledState;

// pre-defined colors. GRB order, g++ doesn't implement out-of-order initializers
const cRGB traktor = {g: 0xB8/4, r: 0x67/4, b: 0xDC/4}; // https://wiki.attraktor.org/Corporate_Identity
const cRGB black = {g: 0x00, r: 0x00, b: 0x00};
const cRGB white = {g: 0x1F, r: 0x1F, b: 0x1F};
const cRGB red = {g: 0x00, r: 0x1F, b: 0x00};
const cRGB green = {g: 0x1F, r: 0x00, b: 0x00};
const cRGB blue = {g: 0x00, r: 0x00, b: 0x1F};

// macro instead of inline function. TODO: compare generated code again
// fill LED buffer with solid color, reset state, set mode
#define setLeds1(color1, mode) {\
  for (uint8_t i = 0; i < MAXPIX+EXTRAPIX; i++) {\
    leds[i] = (color1);\
  }\
  memset(&ledState, 0x00, sizeof(ledState));\
  ledMode = (mode);\
}

// fill LED buffer with alternating colors every $modulo pixels, reset state, set mode
#define setLeds2(color1, color2, modulo, mode) {\
  for (uint8_t i = 0; i < MAXPIX+EXTRAPIX; i++) {\
    leds[i] = (i % (modulo)) ? ((color2)) : ((color1));\
  }\
  memset(&ledState, 0x00, sizeof(ledState));\
  ledMode = (mode);\
}


// global motor direction variable, required for PWM-brake & coasting
// TODO: PWM-brake is never used, maybe refactor & remove
enum motorDir { forward, backward } motorDir;

// I/O connections to bridge driver module
// https://www.olimex.com/Products/RobotParts/MotorDrivers/BB-VNH3SP30/
// https://www.olimex.com/Products/RobotParts/MotorDrivers/BB-VNH3SP30/resources/BB-VNH3SP30_schematic.pdf
const int motorDiagAPin = 5;
const int motorDiagBPin = 6;
const int motorInAPin = 7;
const int motorInBPin = 8;
const int motorPWMPin = 9;

// ad-hoc acceleration profile
// TODO: self-calibration by means of driveTotalMillis
struct accelProfile {
  unsigned long maxMillis; // time until next step, 0 == end of profile
  unsigned long stepMillis; // modify PWM value every stepMillis ms
  double factor; // newValue = (oldValue + add) * factor
  signed int add;
  byte minPWM; // lower limit
  byte maxPWM; // upper limit
  byte startPWM; // initial value
  bool useStartPWM; // overwrite PWM value with startPWM, ignored on first element
  bool brake; // short motor if true, reset to motorDir if false
};

// slow seek to end-stop, used if door is in unknown position
const struct accelProfile accelProfileLow[] = {
  {
    maxMillis: 13000,
    stepMillis: 50,
    factor: 1,
    add: 4,
    minPWM: 0,
    maxPWM: 64,
    startPWM: 0,
    useStartPWM: true,
    brake: false
  },
  {
    maxMillis: 0
  }
};

// normal operation
// try accelerating as fast as possible without skipping belt teeth
// TODO: install less stiff spring between belt and door to dampen the initial pull
const struct accelProfile accelProfileHigh[] = {
  { // slow start
    maxMillis: 260,
    stepMillis: 10,
    factor: 1,
    add: 1,
    minPWM: 0,
    maxPWM: 255,
    startPWM: 0,
    useStartPWM: true, // first element always sets initial PWM value
    brake: false
  },
  { // accelerate and keep running
    maxMillis: 2000,
    stepMillis: 5,
    factor: 1,
    add: 1,
    minPWM: 0,
    maxPWM: 255,
    startPWM: 0,
    useStartPWM: false,
    brake: false
  },
  { // slow down before end-stop
    maxMillis: 1300,
    stepMillis: 100,
    factor: 0.9,
    add: 0,
    minPWM: 32,
    maxPWM: 255,
    startPWM: 0,
    useStartPWM: false,
    brake: false
  },
  { // seek end-stop
    maxMillis: 4000,
    stepMillis: 100,
    factor: 1,
    add: -1,
    minPWM: 32,
    maxPWM: 64,
    startPWM: 0,
    useStartPWM: false,
    brake: false
  },
  {
    maxMillis: 0
  }
};

/*
  { // coast
    maxMillis: 1000,
    stepMillis: 50,
    factor: 1,
    add: 0,
    minPWM: 0,
    maxPWM: 0,
    startPWM: 0,
    useStartPWM: true,
    brake: true
  },
  { // PWM-brake
    maxMillis: 1000,
    stepMillis: 50,
    factor: 1,
    add: 0,
    minPWM: 1,
    maxPWM: 1,
    startPWM: 1,
    useStartPWM: true,
    brake: true
  },
*/

const struct accelProfile *accelProfile; // pointer to current profile
int accelProfileIdx = 0;

// switch inputs, active-low
const int switchFront = 3; // front is at the closed position
const int switchFrontIRQ = 1; // TODO: maybe use interrupts & sleep

const int switchBack = 4;

// wire-ORed buttons, active-low
const int switchTrigger = 2;
const int switchTriggerIRQ = 0;


// unsorted variables below. TODO: refactor & cleanup

// buttons & switches do different things in different states
enum doorStates { doorClosed, doorOpening, doorOpen, doorClosing, doorBlocked, doorError } doorState;

double drivePWM = 0;
const double drivePWMscale = 1; // fiddling aid, constant factor applied to calculated PWM value. TODO: update profile & remove

// TODO: the timing system has some quirks. think again or document

unsigned long lastMillis = 0; // last loop's millis() result

unsigned long driveMillis = 0;
unsigned long driveStepMillis = 0;
unsigned long driveTotalMillis = 0;

const unsigned long openInterval = 3000; // ms before the door closes again, unless blocked by button
unsigned long openMillis = 0;

const unsigned long debounceInterval = 2; // shift inputs into debounce register every debounceInterval ms
unsigned long debounceMillis = 0;

// TODO: use structured profiles
const unsigned long ledSolidInterval = 250; // constantly update LED stripe in solid mode
const unsigned long ledFadeInterval = 20;
const unsigned long ledBlinkInterval = 1000;
const unsigned long ledMoveInterval = 250; // shift LED window every X ms
unsigned long ledMillis = 0;

bool swFront, swBack, swTrigger, motorDiagA, motorDiagB; // debounced inputs
bool swFrontDeglitch, swBackDeglitch, swTriggerDeglitch, motorDiagADeglitch, motorDiagBDeglitch; // TODO: quick hack. document
uint16_t swFrontDebounce, swBackDebounce, swTriggerDebounce, motorDiagADebounce, motorDiagBDebounce; // debounce shift registers


void setup() {
  Serial.begin(115200);
  Serial.print(F("\r\nDoor control init\r\n"));

  // disable driver bridges until setup complete
  motorDisable();

  digitalWrite(motorPWMPin, LOW);
  pinMode(motorPWMPin, OUTPUT);

  digitalWrite(motorInAPin, LOW);
  pinMode(motorInAPin, OUTPUT);

  digitalWrite(motorInBPin, LOW);
  pinMode(motorInBPin, OUTPUT);

// TODO: measure real PWM frequeny. any value other than 1 makes the motor whine
// the code says base frequency is ~31kHz
// VNH3SP30 datasheet limits the input PWM frequency to 10kHz - and yet it works. MOSFETs in thermal danger?
setPwmFrequency(motorPWMPin, 1);

  pinMode(switchFront, INPUT_PULLUP);
  pinMode(switchBack, INPUT_PULLUP);

  pinMode(switchTrigger, INPUT_PULLUP);

  setLeds1(traktor, ledSolid);
  ledptr = &leds[0];
  LED.setOutput(ledPin);
  LED.sync();

  // initialize debounce registers, assume steady state
  swFrontDebounce = (swFront = swFrontDeglitch = digitalRead(switchFront)) * 0xFFFF;
  swBackDebounce = (swBack = swBackDeglitch = digitalRead(switchBack)) * 0xFFFF;
  swTriggerDebounce = (swTrigger = swTriggerDeglitch = digitalRead(switchTrigger)) * 0xFFFF;

  Serial.print(F("Door status: "));

  if (swTrigger == LOW || (swFront == LOW && swBack == LOW)) {
    doorState = doorError;
    Serial.print(F("ERROR\r\n"));
  } else if (swFront == LOW) {
    doorState = doorClosed;
    Serial.print(F("closed\r\n"));
  } else if (swBack == LOW) {
    doorState = doorOpen;
    Serial.print(F("open\r\n"));
  } else {
    doorState = doorBlocked;
    Serial.print(F("blocked\r\n"));
  }

  if (doorState != doorError) {
    // enable driver
    motorEnable();

    // diag pull-ups should have brought the lines up by now
    motorDiagADebounce = (motorDiagA = motorDiagADeglitch = digitalRead(motorDiagAPin)) * 0xFFFF;
    motorDiagBDebounce = (motorDiagB = motorDiagBDeglitch = digitalRead(motorDiagBPin)) * 0xFFFF;

    if (doorState != doorBlocked) {
      motorBrake();
    } else {
      motorFree();
      setLeds1(red, ledSolid);
    }
  } else {
    setLeds1(red, ledBlink);
  }

// TODO: on-board status LEDs, independent of ws2812
digitalWrite(13, LOW);
pinMode(13, OUTPUT);

//  attachInterrupt(switchFrontIRQ, switchFrontInterrupt, CHANGE);
//  attachInterrupt(switchTriggerIRQ, switchTriggerInterrupt, FALLING);
}

void loop() {
  unsigned long currentMillis;
  unsigned long elapsedMillis;

  currentMillis = millis();
  // millis() will overflow after approximately 50 days.
  if (currentMillis < lastMillis) {
    elapsedMillis = (~0UL - lastMillis) + currentMillis;
  } else {
    elapsedMillis = currentMillis - lastMillis;
  }
  lastMillis = currentMillis;
  // assume that none of the following intervals overflow

/*
  debug(currentMillis, F("Elapsed millis: "));
  Serial.println(elapsedMillis);
*/


  // fast debounce driver error signals to filter short glitches
  debounce(motorDiagAPin, &motorDiagA, &motorDiagADeglitch, &motorDiagADebounce);
  debounce(motorDiagBPin, &motorDiagB, &motorDiagBDeglitch, &motorDiagBDebounce);

  // The WAM is overheating!
  if ((motorDiagA == LOW || motorDiagB == LOW) && doorState != doorError) {
    motorFree();
    debug(currentMillis, F("Motor driver error condition - door disabled\r\n"));
    setLeds1(red, ledFade);
    doorState = doorError;
  }


  debounceMillis += elapsedMillis;
  if (debounceMillis >= debounceInterval) {
    debounceMillis = 0;

    debounce(switchTrigger, &swTrigger, &swTriggerDeglitch, &swTriggerDebounce);
    debounce(switchFront, &swFront, &swFrontDeglitch, &swFrontDebounce);
    debounce(switchBack, &swBack, &swBackDeglitch, &swBackDebounce);
  }


// remote debugging aid
// TODO: formalize the commands
if (Serial.available()) {
  switch (Serial.read()) {
    case 'x':
        swTrigger = LOW; break;
    case 'X':
        swTrigger = HIGH; break;
    case 'b':
        swBack = LOW; break;
    case 'B':
        swBack = HIGH; break;
    case 'f':
        swFront = LOW; break;
    case 'F':
        swFront = HIGH; break;
    case 'm':
        Serial.println(currentMillis); break;
    case 'h':
      motorDisable();
      doorState = doorError;
      break;
    case 'r':
      motorEnable();
      doorState = doorBlocked;
      break;
  }
}


  switch(doorState) {
    case doorClosed:
      // wait until a button is pressed or the front switch opens
      if (swFront == HIGH || swTrigger == LOW) {
        if (swFront == HIGH) {
          // maybe someone tried to pull the door open
          // TODO: or the door slams too fast into the end-stop and rebounds. align profile & inertial reality
          debug(currentMillis, F("Front switch triggered - opening door\r\n"));
          initDrive(backward, accelProfileLow);
        } else {
          debug(currentMillis, F("Button switch triggered - opening door\r\n"));
          initDrive(backward, accelProfileHigh);
          swTrigger = HIGH; // re-arm swTrigger
        }
        setLeds2(white, black, 3, ledBackward);
        doorState = doorOpening;
      }
      break;

    case doorOpen:
      // hold the door open if button pressed during openInterval
      if (swTrigger == LOW) {
        debug(currentMillis, F("Button switch triggered - door blocked\r\n"));
        motorFree();
        setLeds1(red, ledSolid);
        doorState = doorBlocked;
        swTrigger = HIGH;
        break;
      }

      openMillis += elapsedMillis;
      if (openMillis >= openInterval || swBack == HIGH) {
        if (swBack == HIGH) {
          // maybe someone tried to pull the door closed
          // TODO: or the door slams too fast into the end-stop and rebounds. align profile & inertial reality
          debug(currentMillis, F("Back switch triggered - closing door\r\n"));
          initDrive(forward, accelProfileLow);
        } else {
          debug(currentMillis, F("openInterval timeout - closing door\r\n"));
          initDrive(forward, accelProfileHigh);
        }
        setLeds2(white, black, 3, ledForward);
        doorState = doorClosing;
      }
      break;

    case doorOpening:
      // are we there yet?
      if (swBack == LOW) {
        motorBrake();
        debug(currentMillis, F("Back switch triggered - door open\r\n"));
        debug(currentMillis, F("Total ms: "));
        Serial.println(driveTotalMillis);
        openMillis = 0;
        setLeds1(green, ledSolid);
        doorState = doorOpen;
        break;
      }
      // fall-through to next case

    case doorClosing:
      if (doorState == doorClosing && swFront == LOW) {
        motorBrake();
        debug(currentMillis, F("Front switch triggered - door closed\r\n"));
        debug(currentMillis, F("Total ms: "));
        Serial.println(driveTotalMillis);
        setLeds1(green, ledFade);
        doorState = doorClosed;
        break;
      }

      // stop if button pressed while in motion
      if (swTrigger == LOW) {
        motorFree(); // TODO: motorBrake() might be better for an emergency stop. doesn't make much different in our case, the belt skips and the door stops in both cases
        debug(currentMillis, F("Button switch triggered - door blocked\r\n"));
        setLeds1(red, ledSolid);
        doorState = doorBlocked;
        swTrigger = HIGH;
        break;
      }

      driveTotalMillis += elapsedMillis;
      driveStepMillis += elapsedMillis;
      if (driveStepMillis >= accelProfile[accelProfileIdx].stepMillis) {
        driveMillis += driveStepMillis;
        driveStepMillis = 0;

        // switch to next profile step
        if (driveMillis >= accelProfile[accelProfileIdx].maxMillis) {
          driveMillis = 0;
          accelProfileIdx++;

          // end-stop not reached at end of profile, someone might be in the way
          if (accelProfile[accelProfileIdx].maxMillis == 0) {
            motorFree();
            debug(currentMillis, F("End of profile - door blocked\r\n"));
            debug(currentMillis, F("Total ms: "));
            Serial.println(driveTotalMillis);
            setLeds1(red, ledBlink);
            doorState = doorBlocked;
            break;
          }

          if (accelProfile[accelProfileIdx].brake) {
            // short motor if PWM is HIGH, coast otherwise
            _motorFree();
          } else {
            switch(motorDir) {
              case forward:
                motorForward(); break;
              case backward:
                motorBackward(); break;
            }
          }

          // overwrite current PWM value
          if (accelProfile[accelProfileIdx].useStartPWM) {
            drivePWM = accelProfile[accelProfileIdx].startPWM;
          }

          debug(currentMillis, F("accelProfileIdx: "));
          Serial.println(accelProfileIdx);
        }

        // update PWM value
        drivePWM = min(accelProfile[accelProfileIdx].maxPWM, max(accelProfile[accelProfileIdx].minPWM,
                        (drivePWM + accelProfile[accelProfileIdx].add) * accelProfile[accelProfileIdx].factor));

        debug(currentMillis, F("Drive PWM: "));
        Serial.println(drivePWM);

        analogWrite(motorPWMPin, round(drivePWM*drivePWMscale));
      }
      break;

    case doorBlocked:
      // wait until a button is pressed
      if (swTrigger == LOW) {
        // close door only if end-stop is active
        if (swBack == LOW) {
          debug(currentMillis, F("Button switch triggered - closing door\r\n"));
//          initDrive(forward, accelProfileLow);
          initDrive(forward, accelProfileHigh);
          ledMode = ledForward;
          doorState = doorClosing;
        } else {
          debug(currentMillis, F("Button switch triggered - opening door\r\n"));
          initDrive(backward, accelProfileLow);
          ledMode = ledBackward;
          doorState = doorOpening;
        }
        setLeds2(white, black, 3, ledMode);
        swTrigger = HIGH;
      }
      break;

    case doorError:
      // wait for reset by qualified service technician
      // TODO: make it user-resettable
// wait until driver cools down
// TODO: this won't work now because motorDisable() pulls the pins low
if (motorDiagA == HIGH && motorDiagB == HIGH) {
  setLeds1(red, ledSolid);
  doorState = doorBlocked;
  digitalWrite(13, LOW);
} else {
  digitalWrite(13, HIGH);
}
      break;
  }


  ledMillis += elapsedMillis;
  switch(ledMode) {
    case ledSolid:
      if (ledMillis >= ledSolidInterval) {
        ledMillis = 0;
        ledptr = &leds[0];
        LED.sync();
      }
      break;

    case ledFade: {
      if (ledMillis >= ledFadeInterval) {
        ledMillis = 0;

        // TODO: really scale the values. this is too cheap
        for (uint8_t i = 0; i < MAXPIX; i++) {
          tmp_leds[i].r = max(0, (int)leds[i].r - abs(ledState.fader));
          tmp_leds[i].g = max(0, (int)leds[i].g - abs(ledState.fader));
          tmp_leds[i].b = max(0, (int)leds[i].b - abs(ledState.fader));
        }

        if (ledState.fader++ >= 255) {
          ledState.fader = -254;
        }

        ledptr = &tmp_leds[0];
        LED.sync();
      }
      break;
    }

    case ledBlink:
      if (ledMillis >= ledBlinkInterval) {
        ledMillis = 0;

        if (!ledState.blink_on) {
  //          memset(&tmp_leds, 0x00, sizeof(tmp_leds));
          for (uint8_t i = 0; i < MAXPIX; i++) {
            tmp_leds[i] = black;
          }
          ledptr = &tmp_leds[0];
        } else {
          ledptr = &leds[0];
        }
  
        ledState.blink_on = !ledState.blink_on;

        LED.sync();
      }
      break;

    case ledBackward:
      if (ledMillis >= ledMoveInterval) {
        ledMillis = 0;

        if (ledState.ledidx >= EXTRAPIX) {
          ledState.ledidx = 0;
        }
  
        // move window forward
        ledptr = &leds[ledState.ledidx++];
        LED.sync();
      }
      break;

    case ledForward:
      if (ledMillis >= ledMoveInterval) {
        ledMillis = 0;

        if (ledState.ledidx == 0) {
          ledState.ledidx = EXTRAPIX;
        }
  
        // move window backwards
        ledptr = &leds[--ledState.ledidx];
        LED.sync();
      }
      break;
  }
}

//__attribute__((always_inline))
inline void debug(const unsigned long timestamp, const __FlashStringHelper *const string) {
  Serial.print(timestamp);
  Serial.write(' ');
  Serial.print(string);
}

//__attribute__((always_inline))
inline void debounce(const int swPin, bool *const swVal, bool *const swDeglitch, uint16_t *const swDebounce) {
  // similiar to what digitalRead() does, assumes that swPin has no timer output assigned
  // this still leads to several indirect lookups per sample. direct PINx access might be faster
  debounce(portInputRegister(digitalPinToPort(swPin)), digitalPinToBitMask(swPin), swVal, swDeglitch, swDebounce);
}

//__attribute__((always_inline))
inline void debounce(const volatile uint8_t *const port, const uint8_t pinMask, bool *const swVal, bool *const swDeglitch, uint16_t *const swDebounce) {
  // NB: interference near multiples of 1/interval could cause spurious edges
  *swDebounce = ((*swDebounce << 1) | ((*port & pinMask)?(1):(0))) & 0x1FFF; // 13-bit debounce

  if (*swDebounce == 0x1000 && *swDeglitch == HIGH) { // edge HIGH -> 12xLOW
    *swVal = *swDeglitch = LOW;
  } else if (*swDebounce == 0x0FFF && *swDeglitch == LOW) { // edge LOW -> 12xHIGH
    *swVal = *swDeglitch = HIGH;
  }
}

// set profile pointer, reset counters, set driver inputs
// TODO: understand & fix enum namespace fnord
inline void initDrive(const int dir, const struct accelProfile *const profile) {
  motorDir = (enum motorDir)dir;

  accelProfile = profile;
  accelProfileIdx = 0;
  drivePWM = accelProfile[0].startPWM;
  driveMillis = driveStepMillis = driveTotalMillis = 0;

  switch(dir) {
    case forward:
      motorForward(); break;
    case backward:
      motorBackward(); break;
  }
}

// functions beginning with _motor don't output to Serial, used in early interrupt version
// TODO: either use them with interrupts or remove

inline void setMotorBits(const byte bits) {
  digitalWrite(motorPWMPin, LOW);
  digitalWrite(motorInAPin, (bits & 0x02) ? (HIGH) : (LOW));
  digitalWrite(motorInBPin, (bits & 0x01) ? (HIGH) : (LOW));
}

void motorDisable() {
  // actively pull diag pins low, disables driver bridges
  digitalWrite(motorDiagAPin, LOW);
  pinMode(motorDiagAPin, OUTPUT);

  digitalWrite(motorDiagBPin, LOW);
  pinMode(motorDiagBPin, OUTPUT);

  Serial.print(F("Motor driver disabled\r\n"));
}

void motorEnable() {
  // return control to pull-ups on the driver2 board
  pinMode(motorDiagAPin, INPUT);
  pinMode(motorDiagBPin, INPUT);
  Serial.print(F("Motor driver enabled\r\n"));
}

void motorBrake() {
  _motorBrake();
  Serial.print(F("Brake engaged\r\n"));
}

void _motorBrake() {
  setMotorBits(0x00);
  digitalWrite(motorPWMPin, HIGH);
}

void motorFree() {
  _motorFree();
  Serial.print(F("Brake disengaged\r\n"));
}

void _motorFree() {
  setMotorBits(0x00);
}

void motorForward() {
  _motorForward();
  Serial.print(F("Forward\r\n"));
}

void _motorForward() {
  setMotorBits(0x02);
}

void motorBackward() {
  _motorBackward();
  Serial.print(F("Backward\r\n"));
}

void _motorBackward() {
  setMotorBits(0x01);
}


// copy&pasted from http://playground.arduino.cc/Code/PwmFrequency

/**
 * Divides a given PWM pin frequency by a divisor.
 *
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 *
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 *
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 *
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://forum.arduino.cc/index.php?topic=16612#msg121031
 */
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = (TCCR0B & 0b11111000) | mode;
    } else {
      TCCR1B = (TCCR1B & 0b11111000) | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = (TCCR2B & 0b11111000) | mode;
  }
}


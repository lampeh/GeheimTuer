//#include <avr/wdt.h>

#include <WS2812.h>
#include <OneWire.h>

/* config */

// MAXPIX must fit uint8_t loop variable in ledFade, ledBlink
// MAXPIX+EXTRAPIX must fit uint8_t loop variable in setLeds
#define MAXPIX 20  // LED stripe length
#define EXTRAPIX 3 // animation buffer, read window moves back and forth EXTRAPIX pixels

// WS2812 data
const int ledPin = 11;

// I/O connections to bridge driver module
// https://www.olimex.com/Products/RobotParts/MotorDrivers/BB-VNH3SP30/
// https://www.olimex.com/Products/RobotParts/MotorDrivers/BB-VNH3SP30/resources/BB-VNH3SP30_schematic.pdf
const int motorDiagAPin = 5;
const int motorDiagBPin = 6;
const int motorInAPin = 7;
const int motorInBPin = 8;
const int motorPWMPin = 9;

// end-stop switches, active-low
// Panasonic AZ7121
const int switchFront = 3; // front is at the closed position
const int switchFrontIRQ = 1; // TODO: maybe use interrupts & sleep

const int switchBack = 4;

// wire-ORed buttons, active-low
const int switchTrigger = 2;
const int switchTriggerIRQ = 0;

// PIR sensor, active-high
// based on BIS0001, e.g.: https://learn.adafruit.com/pir-passive-infrared-proximity-motion-sensor/
const int pirPin = 12;

// on-board status LED
const int statusLED = 13;

// 1-wire bus
OneWire ds(A0);

// PWM fan control
const int fanPWMPin = 10;

// ad-hoc acceleration profile
// TODO: self-calibration by means of driveTotalMillis
// TODO: sense & limit motor current
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
    stepMillis: 5,
    factor: 1,
    add: 1,
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
// belt tension limits torque. important safety feature
// TODO: install less stiff spring between belt and door to dampen the initial pull
const struct accelProfile accelProfileHigh[] = {
  { // accelerate and keep running
    maxMillis: 2500,
    stepMillis: 4,
    factor: 1,
    add: 1,
    minPWM: 0,
    maxPWM: 255,
    startPWM: 0,
    useStartPWM: true, // first element always sets initial PWM value
    brake: false
  },
  { // slow down before end-stop
    maxMillis: 1000,
    stepMillis: 75,
    factor: 0.9,
    add: 0,
    minPWM: 50,
    maxPWM: 255,
    startPWM: 0,
    useStartPWM: false,
    brake: false
  },
  { // seek end-stop
    maxMillis: 3000,
    stepMillis: 100,
    factor: 1,
    add: -1,
    minPWM: 50,
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


/* motor driver */

// global motor direction variable, required for PWM-brake & coasting
// TODO: PWM-brake is never used, maybe refactor & remove
enum motorDir { forward, backward } motorDir;

const struct accelProfile *accelProfile; // pointer to current profile
byte accelProfileIdx = 0;


/* WS2812 LEDs */

cRGB leds[MAXPIX + EXTRAPIX];
cRGB tmp_leds[MAXPIX]; // temp array for fading and blinking
cRGB *ledptr; // read pointer for WS2812 library, points to start of window

// requires custom changes to accept uint8_t** argument
WS2812 LED(MAXPIX, &ledptr);

enum ledModes { ledSolid, ledFade, ledBlink, ledForward, ledBackward } ledMode;

// mode-dependent state
union ledState {
  unsigned int ledidx;
  signed int fader;
  bool blink_off;
} ledState;

// pre-defined colors. GRB order, g++ doesn't implement out-of-order initializers
const cRGB traktor = {g: 0xB8 / 7, r: 0x67 / 7, b: 0xDC / 7}; // https://wiki.attraktor.org/Corporate_Identity
const cRGB black = {g: 0x00, r: 0x00, b: 0x00};
const cRGB white = {g: 0x1F, r: 0x1F, b: 0x1F};
const cRGB red = {g: 0x00, r: 0x1F, b: 0x00};
const cRGB yellow = {g: 0x1F, r: 0x1F, b: 0x00};
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

// fill LED buffer with alternating colors every modulo pixels, reset state, set mode
#define setLeds2(color1, color2, modulo, mode) {\
    for (uint8_t i = 0; i < MAXPIX+EXTRAPIX; i++) {\
      leds[i] = (i % (modulo)) ? ((color2)) : ((color1));\
    }\
    memset(&ledState, 0x00, sizeof(ledState));\
    ledMode = (mode);\
    ledMoveInterval = 250;\
  }


/* unsorted variables below. TODO: refactor & cleanup */

// buttons & switches do different things in different states
enum doorStates { doorClosed, doorOpening, doorOpen, doorClosing, doorBlocked, doorBlockedPir, doorError } doorState;

double drivePWM;
const double drivePWMscale = 1; // fiddling aid, constant factor applied to calculated PWM value. TODO: update profile & remove

// TODO: the timing system has some quirks. think again or document

unsigned long lastMillis = 0; // last loop's millis() result

unsigned long driveMillis;
unsigned long driveProfileMillis;
unsigned long driveTotalMillis;

const unsigned long openInterval = 3000; // ms before the door closes again, unless blocked by button
unsigned long openMillis = 0;

const unsigned long debounceInterval = 2; // shift inputs into debounce register every debounceInterval ms
unsigned long debounceMillis = 0;

const unsigned long debounceIgnoreInterval = 100; // ignore buttons for debounceIgnoreInterval*debounceInterval ms
unsigned long debounceIgnore = 0;

// TODO: use structured profiles
unsigned long ledMillis = 0;
const unsigned long ledSolidInterval = 250; // constantly update LED stripe in solid mode
const unsigned long ledFadeInterval = 20;
const unsigned long ledBlinkInterval = 500;
unsigned long ledMoveInterval; // shift LED window every X ms - varied by drivePWM
const unsigned long ledMoveMin = 50; // minimum move interval if drivePWM == 255
const unsigned long ledMoveMax = 305; // maximum move interval if drivePWM == 0

const int dsMax = 3; // register at most dsMax DS18B20 sensors
byte dsAddrs[dsMax][8]; // 1-wire addresses
byte dsCount; // registered sensors
signed int dsResults[dsMax];

unsigned long dsMillis = 0;
const unsigned long dsInterval = 1000; // sensor read interval
const signed int tempMax = 38/0.0625; // run fan at full speed above tempMax
const signed int tempMin = 30/0.0625; // run fan at minimum speed at tempMin
const signed int tempHyst = 2/0.0625; // shut off fan at tempMin-tempHyst

byte fanPWM;
const byte fanMin = 170; // minimum PWM limit at temp > tempMin
const byte fanMax = 255; // maximum PWM limit at temp < tempMax
bool fanOverride = false;

bool swFront, swBack, swTrigger, motorDiagA, motorDiagB, pirTrigger; // debounced inputs, can be reset to wait for next cycle
bool swFrontDeglitch, swBackDeglitch, swTriggerDeglitch, motorDiagADeglitch, motorDiagBDeglitch, pirDeglitch; // debounced inputs, reset by every detected edge
uint16_t swFrontDebounce, swBackDebounce, swTriggerDebounce, motorDiagADebounce, motorDiagBDebounce, pirDebounce; // debounce shift registers


void setup() {
//  wdt_disable();

  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, HIGH);

  // disable driver bridges until setup complete
  motorDisable();

  // start fan on full speed
  pinMode(fanPWMPin, OUTPUT);
  digitalWrite(fanPWMPin, HIGH);
  fanPWM = 255;

  // initialize WS2812 LEDs
  setLeds1(traktor, ledSolid);
  ledptr = &leds[0];
  LED.setOutput(ledPin);
  LED.sync();

  Serial.begin(115200);
  Serial.print(F("\r\nDoor control init\r\n"));

  // TODO: check driver power dissipation
  // base PWM frequency is ~31kHz. any divisor other than 1 makes the motor whine
  // VNH3SP30 datasheet limits the input PWM frequency to 10kHz. MOSFETs in thermal danger?
  setPwmFrequency(motorPWMPin, 1); // also affects fan PWM output

  digitalWrite(motorPWMPin, LOW);
  pinMode(motorPWMPin, OUTPUT);

  digitalWrite(motorInAPin, LOW);
  pinMode(motorInAPin, OUTPUT);

  digitalWrite(motorInBPin, LOW);
  pinMode(motorInBPin, OUTPUT);

  pinMode(switchFront, INPUT_PULLUP);
  pinMode(switchBack, INPUT_PULLUP);

  pinMode(switchTrigger, INPUT_PULLUP);

  pinMode(pirPin, INPUT);

  // register sensors
  dsCount = scan1Wire(dsAddrs, dsMax);

  // initialize debounce registers, assume steady state
  swFrontDebounce = (swFront = swFrontDeglitch = digitalRead(switchFront)) * 0xFFFF;
  swBackDebounce = (swBack = swBackDeglitch = digitalRead(switchBack)) * 0xFFFF;
  swTriggerDebounce = (swTrigger = swTriggerDeglitch = digitalRead(switchTrigger)) * 0xFFFF;
  pirDebounce = (pirTrigger = pirDeglitch = digitalRead(pirPin)) * 0xFFFF;

  Serial.print(F("Door status: "));

  if (swTrigger == LOW || (swFront == LOW && swBack == LOW)) {
    doorState = doorError;
    setLeds1(blue, ledSolid);
    Serial.print(F("ERROR\r\n"));
  } else if (swFront == LOW) {
    doorState = doorClosed;
    setLeds1(green, ledFade);
    Serial.print(F("closed\r\n"));
/*
  } else if (swBack == LOW) {
    doorState = doorOpen;
    setLeds1(green, ledBlink);
    Serial.print(F("open\r\n"));
*/
  } else {
    doorState = doorBlocked;
    setLeds1(red, ledSolid);
    Serial.print(F("blocked\r\n"));
  }

  if (doorState != doorError) {
    if (doorState != doorBlocked) {
      motorBrake();
    } else {
      motorFree();
    }

    // enable driver
    motorEnable();

    // diag pull-ups should have brought the lines up by now
    motorDiagADebounce = (motorDiagA = motorDiagADeglitch = digitalRead(motorDiagAPin)) * 0xFFFF;
    motorDiagBDebounce = (motorDiagB = motorDiagBDeglitch = digitalRead(motorDiagBPin)) * 0xFFFF;
  }

  //  attachInterrupt(switchFrontIRQ, switchFrontInterrupt, CHANGE);
  //  attachInterrupt(switchTriggerIRQ, switchTriggerInterrupt, FALLING);

  digitalWrite(statusLED, LOW);

  // reset device if loop() takes too long
//  wdt_enable(WDTO_120MS);
}


void loop() {
  unsigned long currentMillis;
  unsigned long elapsedMillis;

//  wdt_reset();

  currentMillis = millis();
  // millis() will overflow after approximately 50 days.
  if (currentMillis < lastMillis) {
    elapsedMillis = (~0UL - lastMillis) + currentMillis;
  } else {
    elapsedMillis = currentMillis - lastMillis;
  }
  lastMillis = currentMillis;
  // assume that none of the following intervals overflow


  // add up dsMillis early, so the door driver can reset it
  dsMillis += elapsedMillis;


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

    if (!debounceIgnore) {
      debounce(switchTrigger, &swTrigger, &swTriggerDeglitch, &swTriggerDebounce);
    } else {
      debounceIgnore--;
    }
 
    debounce(switchFront, &swFront, &swFrontDeglitch, &swFrontDebounce);
    debounce(switchBack, &swBack, &swBackDeglitch, &swBackDebounce);
    debounce(pirPin, &pirTrigger, &pirDeglitch, &pirDebounce);
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
      case 'i':
        pirTrigger = HIGH; break;
      case 'I':
        pirTrigger = LOW; break;
      case 'm':
        debug(currentMillis, F("Door status: "));
        Serial.println(doorState);
        break;
      case 'h':
        motorDisable();
        setLeds1(red, ledFade);
        doorState = doorError;
        break;
      case 'r':
        motorEnable();
        setLeds1(red, ledSolid);
        doorState = doorBlocked;
        break;
      case 'k': {
          const cRGB white255 = {g: 0xFF, r: 0xFF, b: 0xFF};
          setLeds1(white255, ledSolid);
          break;
        }
      case 'K':
        setLeds1(black, ledSolid);
        break;
      case '1':
        dsCount = scan1Wire(dsAddrs, dsMax);
        break;
      case 'l':
        debug(currentMillis, F("Fan full\r\n"));
        digitalWrite(fanPWMPin, HIGH);
        fanOverride = true;
        fanPWM = 255;
        break;
      case 'L':
        fanOverride = false;
        break;
    }
  }


  switch (doorState) {
    case doorClosed:
      // wait until a button is pressed or the front switch opens
      if (swTrigger == LOW || swFront == HIGH) {
        if (swTrigger == LOW) {
          debug(currentMillis, F("Button switch triggered - opening door\r\n"));
          initDrive(backward, accelProfileHigh);
          debounceIgnore = debounceIgnoreInterval;
          swTrigger = HIGH; // re-arm swTrigger
        } else {
          // maybe someone tried to pull the door open
          // TODO: or the door slams too fast into the end-stop and rebounds. align profile & inertial reality
          debug(currentMillis, F("Front switch triggered - opening door\r\n"));
          initDrive(backward, accelProfileLow);
        }
        setLeds2(white, black, EXTRAPIX, ledBackward);
        doorState = doorOpening;
      }
      break;

    case doorOpen:
      // hold the door open if button pressed during openInterval
      if (swTrigger == LOW) {
        motorFree();
        debug(currentMillis, F("Button switch triggered - door blocked\r\n"));
        setLeds1(red, ledSolid);
        doorState = doorBlocked;
        swTrigger = HIGH;
        break;
      }

      // reset timeout while sensor input is HIGH
      if (pirTrigger == HIGH) {
        debug(currentMillis, F("Motion sensor triggered - door blocked by PIR\r\n"));
        setLeds1(yellow, ledBlink);
        doorState = doorBlockedPir;
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
        setLeds2(white, black, EXTRAPIX, ledForward);
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
        setLeds1(green, ledBlink);
        doorState = doorOpen;
        break;
      }
      // fall-through to doorClosing

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

      // keep fan on full while driving
      digitalWrite(fanPWMPin, HIGH);
      dsMillis = 0;
      fanPWM = 255;

      driveTotalMillis += elapsedMillis;
      driveMillis += elapsedMillis;
      if (driveMillis >= accelProfile[accelProfileIdx].stepMillis) {
        driveProfileMillis += driveMillis;
        driveMillis = 0;

        // switch to next profile step
        if (driveProfileMillis >= accelProfile[accelProfileIdx].maxMillis) {
          driveProfileMillis = 0;
          accelProfileIdx++;

          debug(currentMillis, F("accelProfileIdx: "));
          Serial.println(accelProfileIdx);

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
            switch (motorDir) {
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
        }

        // update PWM value
        drivePWM = min(accelProfile[accelProfileIdx].maxPWM, max(accelProfile[accelProfileIdx].minPWM,
                       (drivePWM + accelProfile[accelProfileIdx].add) * accelProfile[accelProfileIdx].factor));

        // update LED animation speed
        ledMoveInterval = ledMoveMin + ((255 - drivePWM) * ((double)(ledMoveMax - ledMoveMin) / 255));

        analogWrite(motorPWMPin, round(drivePWM * drivePWMscale));

        debug(currentMillis, F("Drive PWM: "));
        Serial.print(drivePWM);
        Serial.print(F(", ledMoveInterval: "));
        Serial.println(ledMoveInterval);
      }
      break;

    case doorBlockedPir:
      // motion stopped, return to work
      if (pirTrigger == LOW) {
        if (swBack == LOW) {
          debug(currentMillis, F("Motion sensor released - door open\r\n"));
          openMillis = 0;
          setLeds1(green, ledBlink);
          doorState = doorOpen;
        } else {
          debug(currentMillis, F("Motion sensor released - door blocked\r\n"));
          setLeds1(red, ledSolid);
          doorState = doorBlocked;
        }
      }
      // hold the door open if button pressed
      if (swTrigger == LOW) {
        motorFree();
        debug(currentMillis, F("Button switch triggered - door blocked\r\n"));
        setLeds1(red, ledSolid);
        doorState = doorBlocked;
        swTrigger = HIGH;
      }
      break;

    case doorBlocked:
      // wait until a button is pressed
      if (swTrigger == LOW) {
        // close door only if end-stop is active
        if (swBack == LOW) {
          debug(currentMillis, F("Button switch triggered - closing door\r\n"));
          initDrive(forward, accelProfileHigh);
          ledMode = ledForward;
          doorState = doorClosing;
        } else {
          debug(currentMillis, F("Button switch triggered - opening door\r\n"));
          initDrive(backward, accelProfileLow);
          ledMode = ledBackward;
          doorState = doorOpening;
        }
        setLeds2(white, black, EXTRAPIX, ledMode);
        swTrigger = HIGH;
      }
      break;

    case doorError:
      // wait for reset by qualified service technician
      // TODO: make it user-resettable
      // wait until driver cools down
      // TODO: this won't work now because motorDisable() pulls the pins low
      if (motorDiagA == HIGH && motorDiagB == HIGH) {
        debug(currentMillis, F("Motor driver recovered - door blocked\r\n"));
        setLeds1(red, ledSolid);
        doorState = doorBlocked;
        digitalWrite(statusLED, LOW);
        fanOverride = false;
      } else {
        digitalWrite(statusLED, HIGH);
        digitalWrite(fanPWMPin, HIGH);
        fanPWM = 255;
        fanOverride = true;
      }
      break;
  }


  ledMillis += elapsedMillis;
  switch (ledMode) {
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

          // quick hack: blink outer LEDs if someone approaches the door
          // TODO: fix double period at fader counter limits
          if (pirTrigger == HIGH) {
            if ((ledState.fader / 20) % 2) {
              tmp_leds[0] = tmp_leds[MAXPIX-1] = {g: 0xFF, r: 0xFF, b: 0x00};
            } else {
              tmp_leds[1] = tmp_leds[MAXPIX-2] = {g: 0xFF, r: 0xFF, b: 0x00};
            }
          }

          LED.sync();
        }
        break;
      }

    case ledBlink:
      if (ledMillis >= ledBlinkInterval) {
        ledMillis = 0;

        if (!ledState.blink_off) {
          ledptr = &leds[0];
        } else {
          //          memset(&tmp_leds, 0x00, sizeof(tmp_leds));
          for (uint8_t i = 0; i < MAXPIX; i++) {
            tmp_leds[i] = black;
          }
          ledptr = &tmp_leds[0];
        }

        ledState.blink_off = !ledState.blink_off;

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


  if (dsMillis >= dsInterval) {
    dsMillis = 0;

    byte data[9];
    byte dsRead = 0;
    signed int maxTemp = 0;

    for (byte i = 0; i < dsCount; i++) {
      // read temperature
      ds.reset();
      ds.select(dsAddrs[i]);
      ds.write(0xBE);
      for (byte j = 0; j < sizeof(data); j++) {
        data[j] = ds.read();
      }

      if (OneWire::crc8(data, 8) == data[8]) {
        signed int temp = ((data[1] << 8) | data[0]);

        if (abs(temp - dsResults[i]) > 1) {
          dsResults[i] = temp;

          debug(currentMillis, F("Sensor "));
          Serial.print(i);
          Serial.print(F(": "));
          Serial.print(temp);
          Serial.print(F(" - "));
          Serial.print((double)temp * 0.0625, 2);
          Serial.print(F("°C\r\n"));
        }

        if (dsResults[i] > maxTemp) {
          maxTemp = dsResults[i];
        }

        dsRead++;
      } else {
        debug(currentMillis, F("Sensor "));
        Serial.print(i);
        Serial.print(F(": invalid CRC!\r\n"));
      }

      // start next conversion
      ds.reset();
      ds.select(dsAddrs[i]);
      ds.write(0x44);
    }

    if (!fanOverride) {
      if (dsRead) {
        if (maxTemp <= (tempMin - tempHyst)) {
          if (fanPWM > 0) {
            debug(currentMillis, F("Fan off\r\n"));
            fanPWM = 0;
          }
        } else if (maxTemp >= tempMax) {
          if (fanPWM < 255) {
            debug(currentMillis, F("Fan full\r\n"));
            fanPWM = 255;
          }
        } else if (maxTemp >= tempMin) {
          byte newPWM = map(maxTemp, tempMin, tempMax, fanMin, fanMax);
          if (newPWM != fanPWM) {
            fanPWM = newPWM;
            debug(currentMillis, F("Fan PWM: "));
            Serial.print(fanPWM);
            Serial.print(F(" ("));
            Serial.print(maxTemp);
            Serial.print(F(")\r\n"));
          }
        } else if (fanPWM > fanMin) {
          debug(currentMillis, F("Fan min\r\n"));
          fanPWM = fanMin;
        }
      } else {
        // no temperature sensor responded
        // run fan at full speed
        if (fanPWM < 255) {
          debug(currentMillis, F("No temperature reading! - Fan full\r\n"));
          fanPWM = 255;
        }
      }

      analogWrite(fanPWMPin, fanPWM);
    }
  }
}

//__attribute__((always_inline))
inline void debug(const unsigned long timestamp, const __FlashStringHelper *const string) {
  Serial.print(timestamp);
  Serial.write(' ');
  Serial.print(string);
}

// scan 1-wire bus for temperature sensors
// fills dsAddrs[] with at most dsMax 8-byte addresses
// returns the number of registered addresses
// TODO: requires global OneWire instance "ds"
byte scan1Wire(byte dsAddrs[][8], const byte dsMax) {
  byte addr[8];
  byte dsIdx = 0;

  Serial.print(F("Searching 1-wire...\r\n"));

  ds.reset_search();

  while (ds.search(addr) && dsIdx < dsMax) {
    Serial.print(F("Detected device: "));
    for (byte i = 0; i < sizeof(addr); i++) {
      Serial.print(addr[i], HEX);
    }
    Serial.print(F(" - "));

    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.print(F("invalid CRC!\r\n"));
      continue;
    }

    if (addr[0] == 0x28) {
      memcpy(&dsAddrs[dsIdx], &addr, sizeof(dsAddrs[dsIdx]));
      dsIdx++;
      Serial.print(F("DS18B20 registered\r\n"));
    } else {
      Serial.print(F("unknown device ignored\r\n"));
     }
  }

  return dsIdx;
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
  *swDebounce = ((*swDebounce << 1) | ((*port & pinMask) ? (1) : (0))) & 0x1FFF; // 13-bit debounce

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
  driveProfileMillis = driveTotalMillis = 0;
  driveMillis = accelProfile[0].stepMillis;

  switch (motorDir) {
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
  // return control to pull-ups on the driver board
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
  Serial.print(F("Brake released\r\n"));
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
void setPwmFrequency(const int pin, const int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = (TCCR0B & 0b11111000) | mode;
    } else {
      TCCR1B = (TCCR1B & 0b11111000) | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
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


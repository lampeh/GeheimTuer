#include <avr/pgmspace.h>
#include <WS2812.h>
#include <OneWire.h>
#include "Debounce.h"
#include "Motor.h"


/* config */

#define VERSION "v1.1 " __TIMESTAMP__
#define MAXPIX 20  // LED stripe length
#define EXTRAPIX 4 // animation buffer, read window moves back and forth EXTRAPIX pixels

#if ((EXTRAPIX) > 255)
#error EXTRAPIX must fit uint8_t ledidx variable
#endif

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

Motor motor(motorDiagAPin, motorDiagBPin, motorInAPin, motorInBPin, motorPWMPin);

// end-stop switches, active-low
// Panasonic AZ7121
const int switchFront = 3; // front is at the closed position
const int switchBack = 4;

// wire-ORed buttons, active-low
const int switchTrigger = 2;

// PIR sensor, active-high
// based on BIS0001, e.g.: https://learn.adafruit.com/pir-passive-infrared-proximity-motion-sensor/
const int pirPin = 12;

// on-board status LED
const int statusLED = 13;

// on-board 1-wire sensors
OneWire ds(A0);

// external 1-wire bus
OneWire iButton(A1);

// PWM fan control
const int fanPWMPin = 10;


/* motor driver */

// slow seek to end-stop, used if door is in unknown position
const struct Motor::accelProfile accelProfileLow[] PROGMEM = {
  {
    maxMillis: 13000,
    stepMillis: 5,
    add: 1,
    factor: 1,
    minPWM: 0,
    maxPWM: 64,
  },
  {
    maxMillis: 0
  }
};

// normal operation
// try accelerating as fast as possible without skipping belt teeth
// belt tension limits torque. important safety feature
// TODO: install less stiff spring between belt and door to dampen the initial pull
const struct Motor::accelProfile accelProfileHigh[] PROGMEM = {
  { // accelerate and keep running
    maxMillis: 2500,
    stepMillis: 4,
    add: 1,
    factor: 1,
    minPWM: 0,
    maxPWM: 255,
  },
  { // slow down before end-stop
    maxMillis: 1500,
    stepMillis: 75,
    add: 0,
    factor: 0.9,
    minPWM: 50,
    maxPWM: 255,
  },
  { // seek end-stop
    maxMillis: 2500,
    stepMillis: 100,
    add: -1,
    factor: 1,
    minPWM: 50,
    maxPWM: 64,
  },
  {
    maxMillis: 0
  }
};

struct Motor::accelProfile currentProfile; // RAM-cached profile block
const struct Motor::accelProfile *accelProfile; // pointer to current profile block in progmem


/* WS2812 LEDs */

cRGB leds[MAXPIX + EXTRAPIX];
cRGB tmp_leds[MAXPIX]; // temp array for fading and blinking
cRGB *ledptr; // read pointer for WS2812 library, points to start of window

// requires custom changes to accept uint8_t** argument
WS2812 LED(MAXPIX, &ledptr);

enum ledMode { ledSolid, ledFade, ledBlink, ledForward, ledBackward } ledMode;

// mode-dependent state
union ledState {
  uint8_t ledidx;
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


/* unsorted variables below. TODO: refactor & cleanup */

// buttons & switches do different things in different states
enum doorStates { doorClosed, doorOpening, doorOpen, doorClosing, doorBlocked, doorBlockedPir, doorError } doorState;

double drivePWM;

// TODO: the timing system has some quirks. think again or document

unsigned long lastMillis = 0; // last loop's millis() result

unsigned long driveMillis;
unsigned long driveProfileMillis;
unsigned long driveTotalMillis;

const unsigned long openInterval = 3000; // ms before the door closes again, unless blocked by button
unsigned long openMillis = 0;

const unsigned long debounceInterval = 2; // shift inputs into debounce register every debounceInterval ms
unsigned long debounceMillis = 0;

const unsigned int debounceIgnoreInterval = 150; // ignore buttons for debounceIgnoreInterval*debounceInterval ms
unsigned int debounceIgnore = 0;

// TODO: use structured profiles
unsigned long ledMillis = 0;
const unsigned long ledSolidInterval = 500; // constantly update LED stripe in solid mode
const unsigned long ledFadeInterval = 40;
const unsigned long ledBlinkInterval = 500;
unsigned long ledMoveInterval; // shift LED window every X ms - varied by drivePWM

const unsigned long ledMoveMin = 45; // minimum move interval if drivePWM == 255
const unsigned long ledMoveMax = 300; // maximum move interval if drivePWM == 0

const unsigned int dsMax = 3; // register at most dsMax DS18B20 sensors
byte dsAddrs[dsMax][8]; // 1-wire addresses
byte dsCount; // registered sensors
signed int dsResults[dsMax];

unsigned long dsMillis = 0;
const unsigned long dsInterval = 1000; // sensor read interval
const signed int tempMax = 38 / 0.0625; // run fan at full speed above tempMax
const signed int tempMin = 30 / 0.0625; // run fan at minimum speed at tempMin
const signed int tempHyst = 2 / 0.0625; // shut off fan at tempMin-tempHyst

union sensorData {
  uint8_t scratchpad[9];
  struct {
    int16_t temperature;
    union {
      uint16_t userdata;
      struct {
        int8_t th;
        int8_t tl;
      };
    };
    union {
      uint8_t config;
      struct {
        uint8_t : 5;
        uint8_t resolution : 2;
        uint8_t : 1;
      };
    };
    uint8_t reserved0, reserved1, reserved2;
    uint8_t crc;
  };
};

byte fanPWM;
const byte fanMin = 170; // minimum PWM limit
const byte fanMax = 255; // maximum PWM limit
bool fanOverride = false; // temperature control disabled if true

PinDebounce motorDiagA(motorDiagAPin);
PinDebounce motorDiagB(motorDiagBPin);

PinDebounce swFront(switchFront);
PinDebounce swBack(switchBack);
PinDebounce swTrigger(switchTrigger);
PinDebounce pirTrigger(pirPin);

unsigned long iButtonMillis = 0;
const unsigned long iButtonInterval = 40; // ms between scans for iButton devices on external 1-wire bus
Debounce iButtonTrigger(4); // 4bit debounce -> 3 successive iButton reads to trigger


void debug(const __FlashStringHelper *const string) {
  Serial.print(lastMillis);
  Serial.write(' ');
  Serial.print(string);
}


__attribute__((always_inline))
void setup() {
  Serial.begin(115200);
  debug(F("\r\nGeheimTuer " VERSION "\r\nDoor control init\r\n"));

  // disable driver bridges until setup complete
  motor.disable();

  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, HIGH);

  // start fan on full speed
  pinMode(fanPWMPin, OUTPUT);
  digitalWrite(fanPWMPin, HIGH);
  fanPWM = 255;

  // initialize WS2812 LEDs
  setLeds1(traktor, ledSolid);
  ledptr = &leds[0];
  LED.setOutput(ledPin);
  LED.sync();

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
  swFront.init();
  swBack.init();
  swTrigger.init();
  pirTrigger.init();

  iButtonTrigger.init(searchButton());

  debug(F("Door status: "));

  if (swTrigger() == LOW || iButtonTrigger() == HIGH || (swFront() == LOW && swBack() == LOW)) {
    doorState = doorError;
    setLeds1(blue, ledSolid);
    Serial.print(F("ERROR\r\n"));
  } else if (swFront() == LOW) {
    doorState = doorClosed;
    setLeds1(green, ledFade);
    Serial.print(F("closed\r\n"));
  } else {
    doorState = doorBlocked;
    setLeds1(red, ledSolid);
    Serial.print(F("blocked\r\n"));
  }

  if (doorState != doorError) {
    if (doorState != doorBlocked) {
      motor.brake();
    } else {
      motor.free();
    }

    // enable driver
    motor.enable();

    // diag pull-ups should have brought the lines up by now
    motorDiagA.init();
    motorDiagB.init();
  }

  digitalWrite(statusLED, LOW);
}


__attribute__((always_inline))
void loop() {
  unsigned long elapsedMillis;
  unsigned long currentMillis;

  currentMillis = millis();


  // fast debounce driver error signals to filter short glitches
  motorDiagA.update();
  motorDiagB.update();

  // The WAM is overheating!
  if ((motorDiagA() == LOW || motorDiagB() == LOW) && doorState != doorError) {
    motor.free();
    debug(F("Motor driver error condition - door disabled\r\n"));
    setLeds1(red, ledFade);
    doorState = doorError;
  }


  if (currentMillis == lastMillis) {
    // everything below runs on a ms scale
    return;
  }

  // let the processor handle the overflow
  elapsedMillis = currentMillis - lastMillis;
  lastMillis = currentMillis;
  // assume that none of the following intervals overflow


  debounceMillis += elapsedMillis;
  if (debounceMillis >= debounceInterval) {
    debounceMillis = 0;

    if (!debounceIgnore) {
      swTrigger.update();
    } else {
      debounceIgnore--;
    }

    pirTrigger.update();
    swFront.update();
    swBack.update();
  }


  iButtonMillis += elapsedMillis;
  if (iButtonMillis >= iButtonInterval) {
    iButtonMillis = 0;

    iButtonTrigger.update(searchButton());

    if (iButtonTrigger() == HIGH) {
      swTrigger(LOW);
      iButtonTrigger(LOW);
    }
  }


/*
  // remote debugging aid
  // TODO: formalize the commands
  if (Serial.available()) {
    switch (Serial.read()) {
      case 'x':
        swTrigger(LOW); break;
      case 'X':
        swTrigger(HIGH); break;
      case 'b':
        swBack(LOW); break;
      case 'B':
        swBack(HIGH); break;
      case 'f':
        swFront(LOW); break;
      case 'F':
        swFront(HIGH); break;
      case 'i':
        pirTrigger(HIGH); break;
      case 'I':
        pirTrigger(LOW); break;
      case 'm':
        debug(F("Door status: "));
        Serial.println(doorState);
        break;
      case 'h':
        motor.disable();
        setLeds1(red, ledFade);
        doorState = doorError;
        break;
      case 'r':
        motor.enable();
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
        debug(F("Fan full\r\n"));
        digitalWrite(fanPWMPin, HIGH);
        fanOverride = true;
        fanPWM = 255;
        break;
      case 'L':
        fanOverride = false;
        break;
    }
  }
*/


  // add up dsMillis early, so the door driver can reset it
  dsMillis += elapsedMillis;


  switch (doorState) {
    case doorClosed:
      // wait until a button is pressed or the front switch opens
      if (swTrigger() == LOW || swFront() == HIGH) {
        if (swTrigger() == LOW) {
          swTrigger(HIGH); // re-arm swTrigger
          debounceIgnore = debounceIgnoreInterval;
          debug(F("Button switch triggered - opening door\r\n"));
          initDrive(Motor::motorBackward, accelProfileHigh);
        } else {
          // maybe someone tried to pull the door open
          // TODO: or the door slams too fast into the end-stop and rebounds. align profile & inertial reality
          debug(F("Front switch triggered - opening door\r\n"));
          initDrive(Motor::motorBackward, accelProfileLow);
        }
        setLeds2(white, black, EXTRAPIX, ledBackward);
        doorState = doorOpening;
      }
      break;

    case doorOpen:
      // hold the door open if button pressed during openInterval
      if (swTrigger() == LOW) {
        motor.free();
        debug(F("Button switch triggered - door blocked\r\n"));
        swTrigger(HIGH);
        setLeds1(red, ledSolid);
        doorState = doorBlocked;
        break;
      }

      // reset timeout while sensor input is HIGH
      if (pirTrigger() == HIGH) {
        debug(F("Motion sensor triggered - door blocked by PIR\r\n"));
        setLeds1(yellow, ledBlink);
        doorState = doorBlockedPir;
        break;
      }

      openMillis += elapsedMillis;
      if (openMillis >= openInterval || swBack() == HIGH) {
        if (swBack() == HIGH) {
          // maybe someone tried to pull the door closed
          // TODO: or the door slams too fast into the end-stop and rebounds. align profile & inertial reality
          debug(F("Back switch triggered - closing door\r\n"));
          initDrive(Motor::motorForward, accelProfileLow);
        } else {
          debug(F("openInterval timeout - closing door\r\n"));
          initDrive(Motor::motorForward, accelProfileHigh);
        }
        setLeds2(white, black, EXTRAPIX, ledForward);
        doorState = doorClosing;
      }
      break;

    case doorOpening:
      // are we there yet?
      if (swBack() == LOW) {
        motor.brake();
        debug(F("Back switch triggered - door open\r\n"));
        debug(F("Total ms: "));
        Serial.println(driveTotalMillis);
        openMillis = 0;
        setLeds1(green, ledBlink);
        doorState = doorOpen;
        break;
      }

    // fall-through to doorClosing

    case doorClosing:
      if (doorState == doorClosing && swFront() == LOW) {
        motor.brake();
        debug(F("Front switch triggered - door closed\r\n"));
        debug(F("Total ms: "));
        Serial.println(driveTotalMillis);
        setLeds1(green, ledFade);
        doorState = doorClosed;
        break;
      }

      // stop if button pressed while in motion
      if (swTrigger() == LOW) {
        motor.free(); // TODO: motor.brake() might be better for an emergency stop. doesn't make much different in our case, the belt skips and the door stops in both cases
        debug(F("Button switch triggered - door blocked\r\n"));
        swTrigger(HIGH);
        setLeds1(red, ledSolid);
        doorState = doorBlocked;
        break;
      }

      // keep fan on full while driving
      digitalWrite(fanPWMPin, HIGH);
      fanPWM = 255;

      // don't poll 1-wire temperature sensors while driving
      dsMillis = 0;

      driveTotalMillis += elapsedMillis;
      driveMillis += elapsedMillis;

      if (driveMillis >= currentProfile.stepMillis) {
        driveProfileMillis += driveMillis;
        driveMillis = 0;

        // switch to next profile step
        if (driveProfileMillis >= currentProfile.maxMillis) {
          driveProfileMillis = 0;

          // load profile block from progmem
          _memcpy_P(&currentProfile, ++accelProfile, sizeof(currentProfile));

          // end-stop not reached at end of profile, someone might be in the way
          if (currentProfile.maxMillis == 0) {
            motor.free();
            debug(F("End of profile - door blocked\r\n"));
            debug(F("Total ms: "));
            Serial.println(driveTotalMillis);
            setLeds1(red, ledBlink);
            doorState = doorBlocked;
            break;
          }

          debug(F("Next profile\r\n"));
        }

        // update PWM value
        double newDrivePWM = (drivePWM + currentProfile.add) * currentProfile.factor;

        if (newDrivePWM < currentProfile.minPWM) {
          newDrivePWM = currentProfile.minPWM;
        } else if (newDrivePWM > currentProfile.maxPWM) {
          newDrivePWM = currentProfile.maxPWM;
        }

        drivePWM = newDrivePWM;
        analogWrite(motorPWMPin, round(newDrivePWM));

        // update LED animation speed
        ledMoveInterval = ledMoveMin + (unsigned long)((255 - newDrivePWM) * ((double)(ledMoveMax - ledMoveMin) / 255));

        debug(F("Drive PWM: "));
        Serial.print(newDrivePWM);
        Serial.print(F(", ledMoveInterval: "));
        Serial.println(ledMoveInterval);
      }
      break;

    case doorBlockedPir:
      // motion stopped, return to work
      if (pirTrigger() == LOW) {
        if (swBack() == LOW) {
          debug(F("Motion sensor released - door open\r\n"));
          openMillis = 0;
          setLeds1(green, ledBlink);
          doorState = doorOpen;
        } else {
          debug(F("Motion sensor released - door blocked\r\n"));
          setLeds1(red, ledSolid);
          doorState = doorBlocked;
        }
      }

      // hold the door open if button pressed
      if (swTrigger() == LOW) {
        motor.free();
        debug(F("Button switch triggered - door blocked\r\n"));
        swTrigger(HIGH);
        setLeds1(red, ledSolid);
        doorState = doorBlocked;
      }
      break;

    case doorBlocked:
      // wait until a button is pressed
      if (swTrigger() == LOW) {
        swTrigger(HIGH);

        // close door only if end-stop is active
        if (swBack() == LOW) {
          debug(F("Button switch triggered - closing door\r\n"));
          ledMode = ledForward;
          doorState = doorClosing;
          initDrive(Motor::motorForward, accelProfileHigh);
        } else {
          debug(F("Button switch triggered - opening door\r\n"));
          ledMode = ledBackward;
          doorState = doorOpening;
          initDrive(Motor::motorBackward, accelProfileLow);
        }

        setLeds2(white, black, EXTRAPIX, ledMode);
      }
      break;

    case doorError:
      // wait until driver cools down
      if (motorDiagA() == HIGH && motorDiagB() == HIGH) {
        debug(F("Motor driver recovered - door blocked\r\n"));
        fanOverride = false;
        setLeds1(red, ledSolid);
        doorState = doorBlocked;
        //        digitalWrite(statusLED, LOW);
      } else {
        fanPWM = 255;
        fanOverride = true;
        digitalWrite(fanPWMPin, HIGH);
        digitalWrite(statusLED, HIGH);
      }
      break;

    default:
      motor.free();
      motor.disable();
      debug(F("BUG: invalid doorState value: "));
      Serial.print(doorState);
      Serial.print(F(" - door disabled\r\n"));
      setLeds1(blue, ledSolid);
      doorState = doorError;
      digitalWrite(statusLED, HIGH);
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
          for (size_t i = 0; i < MAXPIX; i++) {
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
          if (pirTrigger() == HIGH) {
            if ((ledState.fader >> 4) % 2) {
              tmp_leds[0] = {g: 0xFF, r: 0xFF, b: 0x00};
              tmp_leds[MAXPIX - 1] = {g: 0xFF, r: 0xFF, b: 0x00};
            } else {
              tmp_leds[1] = {g: 0xFF, r: 0xFF, b: 0x00};
              tmp_leds[MAXPIX - 2] = {g: 0xFF, r: 0xFF, b: 0x00};
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
          // setLeds zeroed tmp_leds
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

    default:
      debug(F("BUG: invalid ledMode value: "));
      Serial.println(ledMode);
      setLeds1(blue, ledSolid);
      digitalWrite(statusLED, HIGH);
      break;
  }

  if (dsMillis >= dsInterval) {
    dsMillis = 0;

    union sensorData sensorData;
    signed int maxTemp;
    byte dsRead = 0;

    for (byte i = 0; i < dsCount; i++) {
      if (!ds.reset()) {
        // no presence pulse detected
        continue;
      }

      // read temperature
      ds.select(dsAddrs[i]);
      ds.write(0xBE);

      for (byte j = 0; j < sizeof(sensorData.scratchpad); j++) {
        sensorData.scratchpad[j] = ds.read();
      }

      if (OneWire::crc8(sensorData.scratchpad, 8) == sensorData.crc) {
        signed int temp = sensorData.temperature;

        // filter small fluctuations in sensor output
        if (abs(temp - dsResults[i]) < 2) {
          temp = dsResults[i];
        } else {
          dsResults[i] = temp;

          debug(F("Sensor "));
          Serial.print(i);
          Serial.print(F(": "));
          Serial.print(temp);
          Serial.print(F(" - "));
          Serial.print((double)temp * 0.0625, 2);
          Serial.print(F("°C\r\n"));
        }

        if (dsRead == 0 || temp > maxTemp) {
          maxTemp = temp;
        }

        dsRead++;
      } else {
        debug(F("Sensor "));
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
            fanPWM = 0;
            debug(F("Fan off\r\n"));
          }
        } else if (maxTemp >= tempMax) {
          if (fanPWM < 255) {
            fanPWM = 255;
            debug(F("Fan full\r\n"));
          }
        } else if (maxTemp >= tempMin) {
          byte newPWM = map(maxTemp, tempMin, tempMax, fanMin, fanMax);
          if (newPWM != fanPWM) {
            fanPWM = newPWM;
            debug(F("Fan PWM: "));
            Serial.print(fanPWM);
            Serial.print(F(" ("));
            Serial.print(maxTemp);
            Serial.print(F(")\r\n"));
          }
        } else if (fanPWM > fanMin) {
          fanPWM = fanMin;
          debug(F("Fan min\r\n"));
        }
      } else {
        // no temperature sensor responded
        // run fan at full speed
        if (fanPWM < 255) {
          digitalWrite(statusLED, HIGH);
          fanPWM = 255;
          debug(F("No temperature reading! - Fan full\r\n"));
        }
      }

      analogWrite(fanPWMPin, fanPWM);
    }
  }
}


/*
  void debug(const unsigned long timestamp, const __FlashStringHelper *const string, void (*cb)()) {
  Serial.print(timestamp);
  Serial.write(' ');
  Serial.print(string);
  if (cb) {
    cb();
  }
  }
*/


// scan 1-wire bus for temperature sensors
// fills dsAddrs[] with at most dsMax 8-byte addresses
// returns the number of registered addresses
// TODO: requires global OneWire instance "ds"
byte scan1Wire(byte dsAddrs[][8], const byte dsMax) {
  byte addr[8];
  byte dsIdx = 0;

  debug(F("Searching 1-wire...\r\n"));

  ds.reset_search();

  while (ds.search(addr) && dsIdx < dsMax) {
    debug(F("Detected device: "));
    for (byte i = 0; i < sizeof(addr); i++) {
      Serial.print(addr[i], HEX);
    }
    Serial.print(F(" - "));

    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.print(F("invalid CRC!\r\n"));
    } else if (addr[0] == 0x28) {
      memcpy(&dsAddrs[dsIdx], &addr, sizeof(dsAddrs[dsIdx]));
      dsIdx++;
      Serial.print(F("DS18B20 registered\r\n"));
    } else {
      Serial.print(F("unknown device ignored\r\n"));
    }
  }

  return dsIdx;
}

bool searchButton() {
  byte addr[8];

  iButton.reset_search();

  while (iButton.search(addr)) {
    if ((addr[0] == 0x01 || addr[0] == 0x18) && OneWire::crc8(addr, 7) == addr[7]) {
      return true;
    }
  }

  return false;
}


// set profile pointer, reset counters, set driver inputs
void initDrive(const enum Motor::motorDir dir, const struct Motor::accelProfile *const profile) {
  accelProfile = profile;

  _memcpy_P(&currentProfile, accelProfile, sizeof(currentProfile));

  if (currentProfile.maxMillis == 0) {
    motor.free();
    motor.disable();

    debug(F("BUG: invalid accelProfile: maxMillis == 0 - door disabled"));

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


void setMotorDir(const enum Motor::motorDir dir) {
  switch (dir) {
    case Motor::motorForward:
      motor.forward();
      break;

    case Motor::motorBackward:
      motor.backward();
      break;

    default:
/*
      debug(F("BUG: invalid motorDir value: "));
      Serial.println(dir);
      digitalWrite(statusLED, HIGH);
*/
      break;
  }
}


// copy&pasted from http://playground.arduino.cc/Code/PwmFrequency

/**
   Divides a given PWM pin frequency by a divisor.

   The resulting frequency is equal to the base frequency divided by
   the given divisor:
     - Base frequencies:
        o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
        o The base frequency for pins 5 and 6 is 62500 Hz.
     - Divisors:
        o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
          256, and 1024.
        o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
          128, 256, and 1024.

   PWM frequencies are tied together in pairs of pins. If one in a
   pair is changed, the other is also changed to match:
     - Pins 5 and 6 are paired on timer0
     - Pins 9 and 10 are paired on timer1
     - Pins 3 and 11 are paired on timer2

   Note that this function will have side effects on anything else
   that uses timers:
     - Changes on pins 3, 5, 6, or 11 may cause the delay() and
       millis() functions to stop working. Other timing-related
       functions may also be affected.
     - Changes on pins 9 or 10 will cause the Servo library to function
       incorrectly.

   Thanks to macegr of the Arduino forums for his documentation of the
   PWM frequency divisors. His post can be viewed at:
     http://forum.arduino.cc/index.php?topic=16612#msg121031
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

__attribute__((always_inline))
void _memcpy_P(void *dest, const void *src, size_t n) {
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

// fill LED buffer with solid color, reset state, set mode
__attribute__((always_inline))
void setLeds1(const cRGB &color1, const enum ledMode mode) {
  if (mode == ledBlink) {
    memset(&tmp_leds, 0x00, sizeof(tmp_leds));
  }

  for (size_t i = 0; i < (MAXPIX + EXTRAPIX); i++) {
    leds[i] = color1;
  }

  memset(&ledState, 0x00, sizeof(ledState));
  ledMode = mode;
}

// fill LED buffer with alternating colors every modulo pixels, reset state, set mode
__attribute__((always_inline))
void setLeds2(const cRGB &color1, const cRGB &color2, const unsigned int modulo, const enum ledMode mode) {
  ledMoveInterval = ledMoveMax;

  for (size_t i = 0; i < (MAXPIX + EXTRAPIX); i++) {
    leds[i] = (i % modulo) ? (color2) : (color1);
  }

  memset(&ledState, 0x00, sizeof(ledState));
  ledMode = mode;
}

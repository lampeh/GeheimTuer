#include <Arduino.h>


class Motor {
  public:
    // acceleration profile
    // TODO: self-calibration by means of driveTotalMillis
    // TODO: sense & limit motor current
    struct accelProfile {
      unsigned long maxMillis; // time until next step, 0 == end of profile
      unsigned long stepMillis; // modify PWM value every stepMillis ms
      double add;
      double factor; // newValue = (oldValue + add) * factor
      double minPWM; // lower limit
      double maxPWM; // upper limit
    };

    enum motorDir { motorForward, motorBackward };

    __attribute__((always_inline))
    Motor(const int diagAPin, const int diagBPin, const int inAPin, const int inBPin, const int PWMPin) : motorDiagAPin(diagAPin), motorDiagBPin(diagBPin), motorInAPin(inAPin), motorInBPin(inBPin), motorPWMPin(PWMPin) {}

    __attribute__((always_inline))
    void setMotorDir(const enum motorDir dir);    

    __attribute__((always_inline))
    void enable();

    __attribute__((always_inline))
    void disable();

    __attribute__((always_inline))
    void free();

    __attribute__((always_inline))
    void brake();

    __attribute__((always_inline))
    void forward();

    __attribute__((always_inline))
    void backward();

  private:
    const int motorDiagAPin;
    const int motorDiagBPin;
    const int motorInAPin;
    const int motorInBPin;
    const int motorPWMPin;

    struct accelProfile currentProfile; // RAM-cached profile block
    const struct accelProfile *accelProfile; // pointer to current profile block in progmem

    __attribute__((always_inline))
    void setMotorBits(const bool A, const bool B);

    __attribute__((always_inline))
    void _free();
    
    __attribute__((always_inline))
    void _brake();
    
    __attribute__((always_inline))
    void _forward();
    
    __attribute__((always_inline))
    void _backward();
};


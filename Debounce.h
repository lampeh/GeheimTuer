#include <Arduino.h>


class Debounce {
  public:
    __attribute__((always_inline))
    Debounce(const unsigned int shift_bits = 13) : shift_mask(~(~0U << shift_bits)), shift_high(~(~0U << (shift_bits - 1))), shift_low(1 << (shift_bits - 1)) {};

    __attribute__((always_inline))
    void init(const bool initState) { debounce = (state = deglitch = ((initState) ? (1) : (0))) * ~0U; };

    __attribute__((always_inline))
    bool update(const bool nextState);

    bool operator ()() const { return state; }
    void operator ()(const bool newState) { state = newState; }

  private:
    const unsigned int shift_mask;
    const unsigned int shift_high;
    const unsigned int shift_low;

    unsigned int debounce;
    bool deglitch;
    bool state;
};


class PinDebounce: public Debounce {
  public:
    __attribute__((always_inline))
    PinDebounce(const int pin, const unsigned int shift_bits = 13) : Debounce(shift_bits), port(portInputRegister(digitalPinToPort(pin))), mask(digitalPinToBitMask(pin)) {};

    __attribute__((always_inline))
    void init() { Debounce::init(*port & mask); };

    __attribute__((always_inline))
    bool update() { return Debounce::update(*port & mask); };

  private:
    const volatile uint8_t *const port;
    const uint8_t mask;
};


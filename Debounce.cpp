#include <Arduino.h>
#include "Debounce.h"


bool Debounce::update(const bool nextState) {
  bool last = state;
  unsigned int tmp_debounce = (debounce << 1) & shift_mask;
  
  if (nextState) {
    tmp_debounce |= 1;
  }

  debounce = tmp_debounce;

  // NB: interference near multiples of 1/interval could cause spurious edges
  if (debounce == shift_low && deglitch == HIGH) { // edge HIGH -> LOW
    state = deglitch = LOW;
  } else if (debounce == shift_high && deglitch == LOW) { // edge LOW -> HIGH
    state = deglitch = HIGH;
  }

  return state != last;
}


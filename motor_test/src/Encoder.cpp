#include "Encoder.hpp"

#include <Arduino.h>


Encoder::Encoder(const uint8_t pinA, 
                 const uint8_t pinB) :
                    kPinA(pinA), 
                    kPinB(pinB) 
{
  pinMode(kPinA, INPUT_PULLUP);
  pinMode(kPinB, INPUT_PULLUP);
  lastA_ = digitalRead(kPinA);
}


void Encoder::onClock(void) {
  bool currentA = static_cast<bool>(digitalRead(kPinA));

  // Only read on change of A and if A is high.
  if (currentA != lastA_ && currentA) {
    bool currentB = static_cast<bool>(digitalRead(kPinB));
    
    if (currentB != currentA) {
      rotation_ = rotation_ - 1;

    } else {
      rotation_ = rotation_ + 1;
    }
  }

  lastA_ = currentA;
}


void Encoder::reset(void) {
  rotation_ = 0;
}


int32_t Encoder::getRotation(const bool resetCounter) {
  int32_t rotation = rotation_;

  // Reset internal counter
  if (resetCounter) {
    reset();
  }

  return rotation;
}
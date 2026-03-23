#include "IBT2.hpp"

#include <Arduino.h>
#include <cmath>
#include <algorithm>


IBT2::IBT2(const uint8_t pinFwd, 
           const uint8_t pinBck, 
           const uint8_t pinEnFwd, 
           const uint8_t pinEnBck) :
              kPinFwd(pinFwd), 
              kPinBck(pinBck), 
              kPinEnFwd(pinEnFwd), 
              kPinEnBck(pinEnBck)
{ 
  pinMode(kPinFwd, OUTPUT);
  pinMode(kPinBck, OUTPUT);
  pinMode(kPinEnFwd, OUTPUT);
  pinMode(kPinEnBck, OUTPUT);
  digitalWrite(kPinEnFwd, LOW);
  digitalWrite(kPinEnBck, LOW);
  bool ledcSuccess1 = ledcAttach(kPinFwd, 1E3, 12);
  bool ledcSuccess2 = ledcAttach(kPinBck, 1E3, 12);
  
  if (!ledcSuccess1 || !ledcSuccess2) {
    Serial.println("Failed to setup LEDC!!!");
  }

  pwmMax_ = 4096;
  resolutionSpeed_ = 100.0 / (pwmMax_ + 1);
  
  ledcWrite(kPinFwd, 0);
  ledcWrite(kPinBck, 0);
}


void IBT2::configPWM(const uint8_t pwmResolutionBits, 
                    const uint32_t pwmFreq) 
{
  ledcDetach(kPinFwd);
  ledcDetach(kPinBck);

  bool ledcSuccess1 = ledcAttach(kPinFwd, pwmFreq, pwmResolutionBits);
  bool ledcSuccess2 = ledcAttach(kPinBck, pwmFreq, pwmResolutionBits);
  
  if (!ledcSuccess1 || !ledcSuccess2) {
    Serial.println("Failed to setup LEDC!!!");
  }

  pwmMax_ = pow(2, pwmResolutionBits) - 1;
  resolutionSpeed_ = 100.0 / (pwmMax_ + 1);

  ledcWrite(kPinFwd, 0);
  ledcWrite(kPinBck, 0);
}


void IBT2::enable(void) {
  digitalWrite(kPinEnFwd, HIGH);
  digitalWrite(kPinEnBck, HIGH);
}


void IBT2::disable(void) {
  digitalWrite(kPinEnFwd, LOW);
  digitalWrite(kPinEnBck, LOW);
  ledcWrite(kPinFwd, 0);
  ledcWrite(kPinBck, 0);
}


void IBT2::setSpeed(float speed) {
  // Motor Stop
  if (std::abs(speed) < resolutionSpeed_) {
    setpointSpeed_ = 0;
    ledcWrite(kPinFwd, 0);
    ledcWrite(kPinBck, 0);
    return;
  }

  // Clamp to [-100, 100]
  const float speedMax = 100.0;
  const float speedMin = -100.0;
  speed = std::max(std::min(speed, speedMax), speedMin);

  // No actionable change
  if (std::abs(speed - setpointSpeed_) < resolutionSpeed_) {
    return;
  }

  setpointSpeed_ = speed;
  uint8_t pwmPin = speed > 0 ? kPinFwd : kPinBck;
  uint8_t pwmPinOther = speed > 0 ? kPinBck : kPinFwd;

  // Convert to PWM value
  uint16_t pwmValue = static_cast<uint16_t>(std::round((std::abs(speed) * pwmMax_) / speedMax));  // Optimised formula from map()
  pwmValue = std::max(std::min(pwmValue, static_cast<uint16_t>(pwmMax_)), static_cast<uint16_t>(0));
  ledcWrite(pwmPin, pwmValue);
  ledcWrite(pwmPinOther, 0);
}
#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <cstdint>


class Encoder {
public:
  Encoder(const uint8_t pinA, const uint8_t pinB);

  /**
   * @brief Clock pin interrupt function.
   * 
   */
  void IRAM_ATTR onClock(void);


  /**
   * @brief Reset the rotation counter.
   * 
   */
  void reset(void);


  /**
   * @brief Get the rotation of the encoder. Result as radians of rotation from reset point.
   * 
   * @param resetCounter Reset the rotation counter. Defaults to true.
   * @return float [rad] Angle of rotation.
   */
  int32_t getRotation(const bool resetCounter = true);


private:
  const uint8_t kPinA = 0;
  const uint8_t kPinB = 0;

  bool lastA_ = false;
  volatile int32_t rotation_ = 0;
};

#endif // ENCODER_HPP

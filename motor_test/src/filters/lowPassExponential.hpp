#ifndef LOWPASSEXPONENTIAL_HPP
#define LOWPASSEXPONENTIAL_HPP


#include <Arduino.h>


class LowPassExponential {
public:
  /// @brief Makes a new exponential moving average low pass filter.
  /// @param alpha_ Filter coefficient.
  LowPassExponential(const float alpha_ = 0.5);


  /// @brief Takes the input value and outputs the filtered result.
  /// @param input Value to filter.
  /// @return Filtered value.
  float filter(const float input);

private:
  float alpha_;      //!< Filter coefficient.
  float prevValue_;  //!< Previous value.

};

#endif // LOWPASSEXPONENTIAL_HPP

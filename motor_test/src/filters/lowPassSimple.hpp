#ifndef LOWPASSSIMPLE_HPP
#define LOWPASSSIMPLE_HPP


#include <Arduino.h>
#include <vector>


class LowPassSimple {
public:
  /// @brief Makes a new simple moving average low pass filter.
  /// @param windowSize Average window size.
  LowPassSimple(const size_t windowSize = 10);

  /// @brief Takes the input value and outputs the filtered result.
  /// @param input Value to filter.
  /// @return Filtered value.
  float filter(const float input);

private:
  size_t index_;      //!< Index of buffer.
  size_t windowSize_; //!< Average window size.
  float runningSum_; //!< Running sum.
  std::vector<float> buffer_;  //!< Average window.
};

#endif // LOWPASSSIMPLE_HPP

#ifndef LOWPASSBUTTERWORTH_HPP
#define LOWPASSBUTTERWORTH_HPP


class LowPassButterworth {
public:
  /// @brief Makes a new Butterworth filter.
  /// @param cutoffFreq [Hz] Frequency cutoff value.
  /// @param sampleRate [Hz] Sampling frequency.
  /// @param qualityFactor Damping factor.  \n 
  ///                      Underdamped > 2.0. \n
  ///                      Overdamped < 2.0.  \n 
  ///                      Critical damping = 0.5.
  LowPassButterworth(const float cutoffFreq, 
                     const float sampleRate, 
                     const float qualityFactor = 2.0);
  
  /// @brief Takes the input value and outputs the filtered result.
  /// @param input Value to filter.
  /// @return Filtered value.
  float filter(const float input);

private:
  float b0_, b1_, b2_, a1_, a2_;
  float x1_, x2_;  // Input Memory
  float y1_, y2_;  // Output Memory
};

#endif // LOWPASSBUTTERWORTH_HPP

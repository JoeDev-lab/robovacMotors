#include "lowPassButterworth.hpp"

#include <cmath>


LowPassButterworth::LowPassButterworth(const float cutoffFreq, 
                                       const float sampleRate, 
                                       const float qualityFactor)
                                          : x1_(0.0), 
                                            x2_(0.0), 
                                            y1_(0.0), 
                                            y2_(0.0) 
{
  // Calculate filter coefficients
  float ff = cutoffFreq / sampleRate;
  float ita = 1.0 / std::tan(M_PI * ff);
  float q = std::sqrt(qualityFactor); // Quality factor for Butterworth
  float denom = 1.0 + q * ita + ita * ita;

  // Feedforward
  b0_ = 1.0 / denom;
  b1_ = 2.0 / denom;
  b2_ = 1.0 / denom;

  // Feedback
  a1_ = 2.0 * (1.0 - ita * ita) / denom;
  a2_ = (1.0 - q * ita + ita * ita) / denom;
}


float LowPassButterworth::filter(float input) {
  float y = (b0_ * input) + (b1_ * x1_) + (b2_ * x2_) - (a1_ * y1_) - (a2_ * y2_);

  x2_ = x1_;
  x1_ = input;
  y2_ = y1_;
  y1_ = y;

  return y;
}
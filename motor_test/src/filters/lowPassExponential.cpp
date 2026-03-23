#include "lowPassExponential.hpp"

LowPassExponential::LowPassExponential(const float alpha_) 
  : alpha_(alpha_),
    prevValue_(0.0)
{}


float LowPassExponential::filter(const float input) {
  prevValue_ = (alpha_ * input) + ((1 - alpha_) * prevValue_);
  return prevValue_;
}
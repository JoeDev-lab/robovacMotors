#include "lowPassSimple.hpp"


LowPassSimple::LowPassSimple(const size_t windowSize) 
  : windowSize_(windowSize),
    index_(0),
    runningSum_(0.0)
{
  buffer_.reserve(windowSize);
}


float LowPassSimple::filter(const float input) {
   if (buffer_.size() < windowSize_) {
    buffer_.push_back(input);
    runningSum_ += input;

  } else {
    runningSum_ -= buffer_.at(index_);
    runningSum_ += input;
    buffer_.at(index_) = input;
    index_ = (index_ + 1) % windowSize_;
  }

  return runningSum_ / buffer_.size();
}
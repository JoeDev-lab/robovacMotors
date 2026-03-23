#include "pid.hpp"
#include <algorithm>

PID::PID(void)
  : outputMax_(INFINITY), 
    outputMin_(-INFINITY),
    gainP_(1.0), 
    gainI_(0.0), 
    gainD_(0.0),
    useAntiwindup_(false) 
{}


void PID::setOutputLimits(const float outputMin, 
                          const float outputMax) 
{
  outputMax_ = outputMax;
  outputMin_ = outputMin;
}


void PID::setGains(const float gainP, 
                   const float gainI, 
                   const float gainD) 
{
  gainP_ = gainP;
  gainI_ = gainI;
  gainD_ = gainD;
}


void PID::setFilterCoeff(float coeffLowPass)
{
  // Delete filter if default value is provided.
  if (std::isnan(coeffLowPass)) {
    lowPass_.release();
    return;
  }
  
  lowPass_ = std::make_unique<LowPassExponential>(coeffLowPass);
}


void PID::setAntiwindup(const bool state) 
{
  useAntiwindup_ = state;
}


void PID::resetPID(void) 
{
  integralError_ = 0;
  previousError_ = 0;
}


float PID::calculateOutput(float inputValue, float setpoint, float deltaTime) 
{
  float error = setpoint - inputValue;
  float termP = error * gainP_;
  integralError_ += (previousError_ + error) * (deltaTime / 2.0);

  if (useAntiwindup_) {
    integralError_ = clampValue(integralError_);
  }
  
  float termI = integralError_ * gainI_;
  float derivativeError;
  
  // Low pass filter
  if (lowPass_ == nullptr) {
    derivativeError = (error - previousError_) / deltaTime;

  } else {
    derivativeError = lowPass_->filter(error);
  }
  
  previousError_ = error;
  float termD = derivativeError * gainD_;
  float output = termP + termI + termD;

  return clampValue(output);
}


float PID::clampValue(const float value) 
{
  return std::max(std::min(value, outputMax_), outputMin_);
}
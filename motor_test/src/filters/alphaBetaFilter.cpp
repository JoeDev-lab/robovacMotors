#include "alphaBetaFilter.hpp"
#include <Arduino.h>
#include <cmath>


AlphaBetaFilter::AlphaBetaFilter(const float alpha, 
                                 const float beta)
                                    : alpha_(alpha), 
                                      beta_(beta),
                                      initialized_(false)                                    
{
}


EncoderData AlphaBetaFilter::filter(const float newValue, const float deltaTime) {
  if (!initialized_) {
    xHat_ = newValue;
    initialized_ = true;
    return EncoderData(xHat_, 0.0, 0.0);
  }

  float xPredicted = xHat_ + yHat_ * deltaTime;
  float residual = newValue - xPredicted;
  xHat_ = xPredicted + alpha_ * residual;
  yHat_ = yHat_ + (beta_ / deltaTime) * residual;

  return EncoderData(xHat_, yHat_, 0.0);
}


AlphaBetaGammaFilter::AlphaBetaGammaFilter(const float alpha, 
                                           const float beta, 
                                           const float gamma) 
                                              : AlphaBetaFilter(alpha, beta),
                                                gamma_(gamma)
{
}


EncoderData AlphaBetaGammaFilter::filter(float newValue, 
                                         float deltaTime) {
  if (!initialized_) {
    Serial.println("Initializing AlphaBetaGammaFilter");
    xHat_ = newValue;
    initialized_ = true;
    return EncoderData(xHat_, 0.0, 0.0);
  }

  float deltaTime2 = std::pow(deltaTime, 2);
  float xPredicted = xHat_ + (yHat_ * deltaTime) + (0.5 * zHat_ * deltaTime2);
  float yPredicted = yHat_ + (zHat_ * deltaTime);
  float residual = newValue - xPredicted;

  // 3. Update estimates
  xHat_ = xPredicted + alpha_ * residual;
  Serial.println("xHat: " + String(xHat_));
  yHat_ = yPredicted + (beta_ / deltaTime) * residual;
  Serial.println("yHat: " + String(yHat_));
  zHat_ = zHat_ + (2.0 * gamma_ / deltaTime2) * residual;
  Serial.println("yHat: " + String(zHat_));

  return EncoderData(xHat_, yHat_, zHat_);
}
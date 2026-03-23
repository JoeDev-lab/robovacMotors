#include "pidTuner.hpp"
#include "../common/helperFunctions.hpp"
#include "arduinoFFT.h"
#include <Arduino.h>
#include <algorithm>
/*
PIDTuner::PIDTuner(void) {
  integration_ = std::make_unique<Integration>();
}


TunerOutput PIDTuner::tunePID(std::unique_ptr<EncoderMotor> &motor,
                              const uint16_t maxIteration) {
  // set initial parameters
  uint16_t iteration = 0;
  double bestKp = 0.0;
  double bestKi = 0.0;
  double bestKd = 0.0;

  while (iteration < maxIteration) {
    // run test

    // calculate changes

    // check if changes are good
      // if so, break loop
      // if not, continue loop

    // modify parameters

    iteration++;
  }

  TunerOutput tunerOutput(bestKp, bestKi, bestKd);
  return tunerOutput;
}


TestOutput PIDTuner::testPID(std::unique_ptr<EncoderMotor> &motor,
                             const uint16_t durationMax,
                             const double frequencyPID,
                             const double setpoint, 
                             const double kp, 
                             const double ki, 
                             const double kd) {
  TestData testResults = runTest(motor, 
                                 durationMax, 
                                 frequencyPID, 
                                 setpoint, 
                                 kp, 
                                 ki, 
                                 kd);
  double riseTime = calculateRiseTime(setpoint, 
                                      testResults);
  
  double overshoot = calculateOvershoot(setpoint, 
                                        testResults);
  
  size_t expectedSize = 50;

  // Estimate number of inflection points to be: total time / (rise time / 2)
  if (riseTime != 0.0) {
    expectedSize = static_cast<size_t>(ceil(2 * testResults.getDuration()  / riseTime)); 
  }

  InflectionData inflectionData = findInflectionPoints(testResults, 
                                                       expectedSize, 0.5);
  
  SteadyStateData steadyStateData = calculateSteadyState(testResults, 
                                                         inflectionData);
  
  return TestOutput(riseTime, 
                    overshoot, 
                    calculateOscillationPeriod(testResults), 
                    calculateOscillationAmplitude(testResults), 
                    steadyStateData, 
                    calculateErrors(testResults));
}


TestData PIDTuner::runTest(std::unique_ptr<EncoderMotor> &motor,
                           const uint16_t durationMax,
                           const double frequencyPID,
                           const double setpoint, 
                           const double kp, 
                           const double ki, 
                           const double kd) 
  {
  motor->disable();
  motor->setGains(kp, ki, kd);
  motor->setSpeed(0.0);
  
  // setup test
  const uint32_t loopDelayMs = static_cast<uint32_t>(round(1000 / frequencyPID));
  TestData testData(static_cast<uint32_t>(ceil(durationMax / loopDelayMs)));
  uint32_t timeStart = millis();
  uint32_t timePrev = timeStart;
  
  // run test
  motor->enable();
  motor->setSpeed(setpoint);

  while (millis() - timeStart < durationMax) {
    delay(loopDelayMs);
    uint32_t timeNow = millis();
    EncoderData encoderData = motor->update(timeNow - timePrev);
    timePrev = timeNow;
    Serial.print(">rotation:");
    Serial.println(encoderData.rotation);
    Serial.print(">velocity:");
    Serial.println(encoderData.velocity);
    Serial.print(">acceleration:");
    Serial.println(encoderData.acceleration);
    Serial.print(">setpoint:");
    Serial.println(setpoint);
    testData.addPoint(encoderData.rotation, encoderData.velocity, encoderData.acceleration, setpoint, timeNow - timeStart);
  }

  // cleanup
  motor->setSpeed(0.0);
  motor->disable();
  testData.trim();

  return testData;
}


TestData runTest(std::unique_ptr<EncoderMotor> &motor,
                   const uint16_t durationMax,
                   const double frequencyPID,
                   const std::vector<std::pair<uint32_t, double>> setpoint,
                   const double kp,
                   const double ki,
                   const double kd) 
  {
  motor->disable();
  motor->setGains(kp, ki, kd);
  motor->setSpeed(0.0);
  
  // setup test
  const uint32_t loopDelayMs = static_cast<uint32_t>(round(1000 / frequencyPID));
  TestData testData(static_cast<uint32_t>(ceil(durationMax / loopDelayMs)));
  auto currentSetpoint = setpoint.begin();
  uint32_t setpointDuration = currentSetpoint->first;
  
  // run test
  uint32_t timeStart, timePrev, setpointStart = millis();
  motor->enable();
  motor->setSpeed(currentSetpoint->second);
  uint32_t timeNow = millis();

  while (timeNow - timeStart < durationMax) {
    delay(loopDelayMs);
    timeNow = millis();

    // Dynamic setpoints
    if (currentSetpoint != setpoint.end()) {
      if (timeNow - setpointStart >= setpointDuration) {
        currentSetpoint++;
        setpointDuration = currentSetpoint->first;
        setpointStart = timeNow;
        motor->setSpeed(currentSetpoint->second);
      }
    }

    EncoderData encoderData = motor->update(timeNow - timePrev);
    timePrev = timeNow;
    testData.addPoint(encoderData.rotation, encoderData.velocity, encoderData.acceleration, currentSetpoint->second, timeNow - timeStart);
  }

  // cleanup
  motor->setSpeed(0.0);
  motor->disable();

  return testData;
  }


double PIDTuner::calculateRiseTime(const double setpoint, 
                                   TestData &testResults) {
  const double velocityThreshold = setpoint * 0.9;
  delay(1000);
  double result = 0.0;

  for (TestData::DataPoint &dataPoint : testResults.data) {
    if (dataPoint.velocity >= velocityThreshold) {
      result = dataPoint.time;
      break;
    }
  }

  return result;
}


double PIDTuner::calculateOvershoot(const double setpoint, 
                                    TestData &testResults) {
  double maxSpeed = 0.0;

  for (TestData::DataPoint &dataPoint : testResults.data) {
    if (dataPoint.velocity > setpoint) {
      maxSpeed = dataPoint.velocity;
    }
  }
    

  return (maxSpeed - setpoint) / setpoint * 100.0;
}


double PIDTuner::calculateOscillationPeriod(TestData &testResults) {
  // Find max samples - needs to be power of 2
  double maxSamples = 2;
  double nextNextSamples = 4;

  while (nextNextSamples <= testResults.data.size()) {
    maxSamples = nextNextSamples;
    nextNextSamples *= 2;
  }

  // Make new vector of samples from back of data
  std::vector<double> samples = testResults.getVelocityData();
  auto itStart = samples.end() - maxSamples;
  std::vector<double> velocityData = std::vector<double>(itStart, samples.end());
  velocityData.shrink_to_fit();

  // Prepare FFT
  std::vector<uint32_t> timeData = testResults.getDeltaTime();
  double sampleFrequency = HelperFunctions::calculateClippedMean(timeData, 1.0);
  std::vector<double> imaginary(velocityData.size(), 0.0);
  ArduinoFFT<double> FFT = ArduinoFFT<double>(velocityData.data(), imaginary.data(), maxSamples, sampleFrequency);

  // run FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.dcRemoval();
  FFT.complexToMagnitude();
  double frequency = FFT.majorPeak();

  return frequency;
}

  
double PIDTuner::calculateOscillationAmplitude(TestData &testResults) {
  std::vector<double> velocityData = testResults.getVelocityData();
  double standardDeviation = HelperFunctions::standardDeviation(velocityData);
  return standardDeviation * std::sqrt(2.0);  // Rough answer.
}


InflectionData PIDTuner::findInflectionPoints(TestData &testResults,
                                              const size_t expectedSize,
                                              const float alpha) {
  
  double maxAcceleration = findMaxAcceleration(testResults);
  double thresholdFlat = 1E-3;

  if (!std::isnan(maxAcceleration)) {
    thresholdFlat = maxAcceleration * 0.05;
  }

  double accelerationFiltered = 0.0;
  InflectionData inflectionPoints(expectedSize);

  enum class SlopeDirection {
    SLOPE_UNKNOWN,
    SLOPE_RISING,
    SLOPE_FALLING,
    SLOPE_FLAT
  };

  SlopeDirection previousSlope = SlopeDirection::SLOPE_UNKNOWN;
  SlopeDirection previousSlopeDirection = SlopeDirection::SLOPE_UNKNOWN;  // Used to remember the previous slope if flat for a short time
  bool firstStep = false;
  
  for (TestData::DataPoint &dataPoint : testResults.data) {
    accelerationFiltered += alpha * (dataPoint.acceleration - accelerationFiltered);  // Apply low pass filter to results

    // Find inflection point
    switch (previousSlope) {
      // Starting state - skip until slope found to compensate for delayed starts.
      case SlopeDirection::SLOPE_UNKNOWN: {
        if (!firstStep) {
          if (accelerationFiltered > 0.0) {
            previousSlope = SlopeDirection::SLOPE_RISING;
            previousSlopeDirection = previousSlope;

          } else if (accelerationFiltered < 0.0) {
            previousSlope = SlopeDirection::SLOPE_FALLING;
            previousSlopeDirection = previousSlope;

          }
        } else {
          firstStep = true;
        }

        break;
      }

      case SlopeDirection::SLOPE_RISING: {
        if (accelerationFiltered < 0.0) { // Inflection point found
          previousSlope = SlopeDirection::SLOPE_FALLING;
          previousSlopeDirection = previousSlope;
          inflectionPoints.addPoint(InflectionData::InflectionType::MAXIMA, dataPoint.velocity, dataPoint.time);

        } else if (accelerationFiltered <= thresholdFlat) {
          previousSlope = SlopeDirection::SLOPE_FLAT;
          inflectionPoints.addPoint(InflectionData::InflectionType::FLAT, dataPoint.velocity, dataPoint.time);
        }
          
        break;
      }

      case SlopeDirection::SLOPE_FALLING: {
        if (accelerationFiltered > 0.0) { // Inflection point found
          previousSlope = SlopeDirection::SLOPE_RISING;
          previousSlopeDirection = previousSlope;
          inflectionPoints.addPoint(InflectionData::InflectionType::MINIMA, dataPoint.velocity, dataPoint.time);

        } else if (accelerationFiltered >= -thresholdFlat) {
          previousSlope = SlopeDirection::SLOPE_FLAT;
          inflectionPoints.addPoint(InflectionData::InflectionType::FLAT, dataPoint.velocity, dataPoint.time);
        }

        break;
      }

      case SlopeDirection::SLOPE_FLAT: {
        // Check if still flat.
        if (std::abs(accelerationFiltered) > thresholdFlat) {
          if (accelerationFiltered > 0.0) {
            // Add inflection point if was previously falling.
            if (previousSlopeDirection == SlopeDirection::SLOPE_FALLING) {
              inflectionPoints.addPoint(InflectionData::InflectionType::MINIMA, dataPoint.velocity, dataPoint.time);
            } 

            previousSlope = SlopeDirection::SLOPE_RISING;

          } else {  // accelerationFiltered < 0.0
            // Add inflection point if was previously rising.
            if (previousSlopeDirection == SlopeDirection::SLOPE_RISING) {
              inflectionPoints.addPoint(InflectionData::InflectionType::MINIMA, dataPoint.velocity, dataPoint.time);
            }

            previousSlope = SlopeDirection::SLOPE_FALLING;
          }     

          previousSlopeDirection = previousSlope;
          break;
        }

        inflectionPoints.addPoint(InflectionData::InflectionType::FLAT, dataPoint.velocity, dataPoint.time);
        break;
      }
    }
  }

  inflectionPoints.trim();
  return inflectionPoints;
}


double PIDTuner::findMaxAcceleration(const TestData &testResults) {
  auto itMaxAcceleration = std::max_element(
    testResults.data.begin(), 
    testResults.data.end(), 
    [] (const TestData::DataPoint& a, const TestData::DataPoint& b) {
      return std::abs(a.acceleration) < std::abs(b.acceleration);
    }
  );

  double maxAcceleration = std::nan("");

  if (itMaxAcceleration != testResults.data.end()) {
    maxAcceleration = itMaxAcceleration->acceleration;
  }

  return maxAcceleration;
} 


SteadyStateData PIDTuner::calculateSteadyState(TestData &testData, InflectionData &inflectionPoints) {
  SteadyStateData steadyStateData;
  
  // Steady state error
  std::vector<double> dataVector = inflectionPoints.data();
  std::vector<double> setpoints = testData.getSetpointData();
  const double clippedMean = HelperFunctions::calculateClippedMean(dataVector, 1.0);
  steadyStateData.average = clippedMean - HelperFunctions::calculateMean(setpoints); // TODO: Update to work with variable setpoints.

  // Settling time
  // Find the first 3 points that are less than the average + 5%
  const double settlingThresholdUpper = clippedMean * 1.05;
  const double settlingThresholdLower = clippedMean * 0.95;

  for (size_t i = 1; i < dataVector.size() - 3; i++) {
    double p1 = inflectionPoints.getPoint(i).value;
    double p2 = inflectionPoints.getPoint(i + 1).value;
    double p3 = inflectionPoints.getPoint(i + 2).value;

    // Skip if velocity is outside the threshold.
    if (p1 > settlingThresholdUpper || 
        p1 < settlingThresholdLower ||
        p2 > settlingThresholdUpper ||
        p2 < settlingThresholdLower ||
        p3 > settlingThresholdUpper ||
        p3 < settlingThresholdLower) {
      continue;
    }

    steadyStateData.settlingTime = inflectionPoints.getPoint(i).time - inflectionPoints.getPoint(0).time;  // TODO: Update to work with variable setpoint.
    break;
  }

  return steadyStateData;
}


ErrorData PIDTuner::calculateErrors(TestData &testResults) {
  bool hasStarted = false;

  // Prepare data
  std::vector<double> errorAbs;
  errorAbs.reserve(testResults.size());
  std::vector<double> errorSquared;
  errorSquared.reserve(testResults.size());
  std::vector<double> errorAbsTime;
  errorAbsTime.reserve(testResults.size());
  std::vector<double> errorSquaredTime;
  errorSquaredTime.reserve(testResults.size());
  std::vector<double> errorSquaredTimeSquared;
  errorSquaredTimeSquared.reserve(testResults.size());

  for (size_t i = 0; i < testResults.size(); i++) {
    if (!hasStarted) {
      // Skip until system responds to compensate for delayed starts in test.
      if (std::abs(testResults.getPoint(i + 1).acceleration) <= 1E-3) {
        continue;
      }

      hasStarted = true;
    }

    TestData::DataPoint dataPoint = testResults.getPoint(i);
    double error = dataPoint.velocity - dataPoint.setpoint;
    errorAbs.push_back(std::abs(error));
    errorSquared.push_back(std::pow(error, 2));
    errorAbsTime.push_back(errorAbs.back() * dataPoint.time);
    errorSquaredTime.push_back(errorSquared.back() * dataPoint.time);
    errorSquaredTimeSquared.push_back(errorSquaredTime.back() * dataPoint.time);  // errorSquaredTime has already calculated 3 of the 4 products.
  }

  // Calculate errors
  uint32_t timeDelta = static_cast<uint32_t>(round(testResults.getDuration() / (testResults.size() - 1)));
  ErrorData errorData(integration_->simpsonsMixed(errorAbs, timeDelta), // integral absolute error
                      integration_->simpsonsMixed(errorSquared, timeDelta), // integral square error
                      integration_->simpsonsMixed(errorAbsTime, timeDelta), // integral time absolute error
                      integration_->simpsonsMixed(errorSquaredTime, timeDelta), // integral time square error
                      integration_->simpsonsMixed(errorSquaredTimeSquared, timeDelta)); // integral square time error
  
  return errorData;
}
*/
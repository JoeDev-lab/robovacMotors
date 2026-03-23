#ifndef PIDTUNER_HPP
#define PIDTUNER_HPP


#include "../EncoderMotor.hpp"
#include "../common/integration.hpp"
#include "tunerData.hpp"
#include <cstdint>
#include <limits>
#include <memory>

// TODO: Set up method to have non-constant setpoint.
class PIDTuner {
public:
  PIDTuner(void);

  /// @brief Tunes the PID gains of the motor and returns the best results.
  /// @param motor Reference to motor to tune.
  /// @param maxIteration Maximum number of test iterations.
  /// @return Best found PID gain values.
  TunerOutput tunePID(std::unique_ptr<EncoderMotor> &motor, 
                      const uint16_t maxIteration);
  
  /// @brief Runs a single PID test and returns the results.
  /// @param motor Reference to motor to test.
  /// @param durationMax [ms] Maximum duration of test
  /// @param frequencyPID [Hz] Frequency of PID control
  /// @param setpoint PID setpoint
  /// @param kp PID gains
  /// @param ki PID gains
  /// @param kd PID gains
  /// @return Test results.
  TestOutput testPID(std::unique_ptr<EncoderMotor> &motor,
                     const uint16_t durationMax,
                     const double frequencyPID,
                     const double setpoint,
                     const double kp,
                     const double ki,
                     const double kd);

private:
  /// @brief Run a single PID test and returns the data collected from same.
  /// @param motor  Reference to motor to test.
  /// @param durationMax [ms] Maximum duration of test
  /// @param frequencyPID [Hz] Frequency of PID control
  /// @param setpoint PID setpoint
  /// @param kp PID gains
  /// @param ki PID gains
  /// @param kd PID gains
  /// @return Data collected during test.
  TestData runTest(std::unique_ptr<EncoderMotor> &motor,
                   const uint16_t durationMax,
                   const double frequencyPID,
                   const double setpoint,
                   const double kp,
                   const double ki,
                   const double kd);

                   
  /// @brief Run a single PID test and returns the data collected from same. Can take varying setpoints in the form [duration [ms], setpoint].
  /// @param motor  Reference to motor to test.
  /// @param durationMax [ms] Maximum duration of test
  /// @param frequencyPID [Hz] Frequency of PID control
  /// @param setpoint PID setpoint
  /// @param kp PID gains
  /// @param ki PID gains
  /// @param kd PID gains
  /// @return Data collected during test.
  TestData runTest(std::unique_ptr<EncoderMotor> &motor,
                   const uint16_t durationMax,
                   const double frequencyPID,
                   const std::vector<std::pair<uint32_t, double>> setpoint,
                   const double kp,
                   const double ki,
                   const double kd);
  
  /// @brief Calculates the rise time from test data.
  /// @param setpoint setpoint to calculate from.
  /// @param testResults Results of test data.
  /// @return [ms] rise time.
  double calculateRiseTime(const double setpoint, 
                           TestData &testResults);


  /// @brief Calculates the overshoot from test data.
  /// @param setpoint setpoint to calculate from
  /// @param testResults Results of test data.
  /// @return [%] overshoot.
  double calculateOvershoot(const double setpoint, 
                            TestData &testResults);

  
  /// @brief Calculates the oscillation period using arduinoFFT.
  /// @param testResults Results of test data.
  /// @return [ms] oscillation period
  double calculateOscillationPeriod(TestData &testResults);


  /// @brief Calculates the oscillation amplitude.
  /// @param testResults Results of test data.
  /// @return oscillation period
  double calculateOscillationAmplitude(TestData &testResults);

                    
  /// @brief Calculates the steady state data from inflection points.
  /// @param setpoint The test setpoint.
  /// @param inflectionPoints inflection points.
  /// @return SteadyStateData.
  SteadyStateData calculateSteadyState(TestData &testData, 
                                       InflectionData &inflectionPoints);


  /// @brief Find the inflection points in the data.
  /// @param testResults Test data
  /// @param expectedSize Expected number of data entries. Used to reserve memory.
  /// @param alpha Low pass filter coefficient.
  /// @return Inflection point data.
 InflectionData findInflectionPoints(TestData &testResults,
                                     const size_t expectedSize,
                                     const float alpha = 0.5);


  /// @brief Finds the maximum acceleration in the data.
  /// @param testResults Test results to scan
  /// @return The maximum acceleration value. If none exists will return nan.
  double findMaxAcceleration(const TestData &testResults);
  

  /// @brief Calculates the error data.
  /// @param testResults Test data
  /// @return Error data: IAE: integral absolute error \n 
  ///                     ISE: integral square error \n
  ///                     ITAE: integral time absolute error \n
  ///                     ITSE: integral time square error \n
  ///                     ISTE: integral square time error
  ErrorData calculateErrors(TestData &testResults);

  std::unique_ptr<Integration> integration_;  /// Integration methods
};


#endif // PIDTUNER_HPP

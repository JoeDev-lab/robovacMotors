#ifndef PID_HPP
#define PID_HPP
/**
 * @file PID.h
 * @author Joseph Tarbath (joseph.tarbath@hotmail.com)
 * @brief Simple PID with output clamping, derivative filtering and antiwindup.
 * @version 1.1
 * @date 18-12-2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "filters/lowPassExponential.hpp"
#include <limits>
#include <cmath>
#include <memory>

/**
 * @brief Simple PID with output clamping, derivative filtering and antiwindup.
 * 
 */
class PID {
public:
  /**
   * @brief Construct a new PID object
   * 
   */
  PID(void);


  PID& operator=(const PID& other) {
    return *this;
  };

  
  /**
   * @brief Sets the output clamp Limits. If no input provided, will default to [-inf, inf].
   * 
   * @param outputMin Minimum output value.
   * @param outputMax Maximum output value.
   */
  void setOutputLimits(const float outputMin = -INFINITY, 
                       const float outputMax = INFINITY);

  
  /**
   * @brief Set the PID Gains.
   * 
   * @param gainP proportional gain
   * @param gainI integral gain 
   * @param gainD derivative gain 
   */
  void setGains(const float gainP, 
                const float gainI, 
                const float gainD);
  

  /**
   * @brief Set the Filter coefficient for derivative error. If no input is given, will remove filter.
   * 
   * @param coeffLowPass Desired low pass filter coefficient.
   */
  void setFilterCoeff(const float coeffLowPass = NAN);


  /**
   * @brief Enables/Disables antiwindup.
   * 
   */
  void setAntiwindup(const bool state = false);


  /**
   * @brief Resets PID memory.
   * 
   */
  void resetPID(void);


  /**
   * @brief Calculates velocity from input value and time since last control input. \n 
   *        Implements antiwindup, derivative filtering and output clamping if set.
   * 
   * @param inputValue Value being controlled.
   * @param setpoint Value setpoint.
   * @param deltaTime Time since previous control calculation.
   * @return float output vale. 
   */
  float calculateOutput(const float inputValue, 
                         const float setpoint, 
                         const float deltaTime);


private:
  bool useAntiwindup_;   //!< Flag for using antiwindup.
  float outputMax_;      //!< Maximum control output.
  float outputMin_;      //!< Minimum control output.
  float gainP_;          //!< proportional gain.
  float gainI_;          //!< integral gain.
  float gainD_;          //!< derivative gain.
  float integralError_;  //!< cumulative error for steady state error control.
  float previousError_;  //!< memory of previous error for derivativeError calculation.
  std::unique_ptr<LowPassExponential> lowPass_;  //!< low pass filter.
  
  
  /// @brief Clamps the output value to the set limits.
  /// @param value Value to clamp.
  /// @return Clamped value.
  float clampValue(const float value);
};

#endif // PID_HPP

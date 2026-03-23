#ifndef ENCODERMOTOR_HPP
#define ENCODERMOTOR_HPP

#include "IBT2.hpp"
#include "Encoder.hpp"
#include "INA219.hpp"
#include "PID.hpp"
#include "UKF.hpp"
#include <cstdint>
#include <memory>
#include <ArduinoEigenDense.h>


class EncoderMotor {
public:
  /// @brief Makes a new encoder motor.
  /// @param pinFwd Forward PWM pin.
  /// @param pinBck Backward PWM pin.
  /// @param pinEnFwd Forward enable pin.
  /// @param pinEnBck Backward enable pin.
  /// @param pinA Encoder A (Clk) Pin.
  /// @param pinB Encoder B (Dt) Pin.
  /// @param pulsePerRev Encoder pulses per revolution.
  /// @param addressPower INA219 address.
  EncoderMotor(const uint8_t pinFwd, 
               const uint8_t pinBck, 
               const uint8_t pinEnFwd, 
               const uint8_t pinEnBck, 
               const uint8_t pinA, 
               const uint8_t pinB);

        
  /**
   * @brief Initialise the UKF.
   * 
   * @param motorKt Motor torque constant [Nm/A].
   * @param motorJ Motor rotor inertia [kg*m^2].
   * @param alpha UKF sigma point spread
   * @param beta UKF sigma point shape. 2 = Gaussian.
   * @param kappa UKF scaling. Usually 0 or 3 - dimStates.
   * @param sensorNoise Sensor noise of the measurements. Must be in same order as measurements.
   * @param processNoise Process noise of UKF model. Must be dimStates x dimStates.
   */
  void initialiseUKF(const float motorKt,
                    const float motorJ,
                    const float alpha,
                    const float beta,
                    const float kappa,
                    const Eigen::VectorXf &sensorNoise,
                    const Eigen::MatrixXf &processNoise);

  /**
   * @brief Initialise the motor's PWM configuration.
   * 
   * @param pwmResolutionBits PWM bit depth.
   * @param pwmFreq [Hz] PWM frequency.
   */
  void initialiseMotor(const uint8_t pwmResolutionBits, 
                      const uint32_t pwmFreq);

    
  void initialiseWattmeter(const uint8_t address, const float readCurrent, const float measuredCurrent);


  void setPulsePerRev(const uint32_t pulsePerRev);

  /**
   * @brief Set the PID Gains.
   * 
   * @param gainP Proportional gain.
   * @param gainI Integral gain.
   * @param gainD Derivative gain.
   */
  void pidSetGains(const float gainP, 
                  const float gainI, 
                  const float gainD);


  /**
   * @brief Set the low pass filter coefficient.
   * 
   * @param coeffLowPass Desired low pass filter coefficient.
   */
  void pidSetFilterCoeff(float coeffLowPass);


  /**
   * @brief Set angular rotation setpoint.
   * 
   * @param speed Desired angular rotation.
   */
  void setSpeed(float speed);


  /**
   * @brief Passes though the encoder interrupt function.
   * 
   */
  void onClock(void) {
    encoder_->onClock();
  }


  /**
   * @brief Enable the motor driver.
   * 
   */
  void enable(void) {
    ibt2_->enable();
  }


  /**
   * @brief Disable the motor driver.
   * 
   */
  void disable(void) {
    ibt2_->disable();
  }


  /**
   * @brief Updates the motor speed based on setpoint and time since last update.
   * 
   * @param deltaMS [s] Time since last update.
   * @return EncoderData Angular velocity [rad/s] and angular rotation [rad].
   */
  Eigen::Vector2f update(const uint32_t delta);


private:
  std::unique_ptr<IBT2> ibt2_;
  std::unique_ptr<Encoder> encoder_;
  std::unique_ptr<PID> pid_;
  std::unique_ptr<INA219Local> ina219_;
  std::unique_ptr<UKF> ukf_;

  float stepAngle_; //!< [rad] Encoder step angle
  float setpointSpeed_ = 0.0;  //!< [rad/s] Desired angular rotation.
};

#endif // ENCODERMOTOR_HPP

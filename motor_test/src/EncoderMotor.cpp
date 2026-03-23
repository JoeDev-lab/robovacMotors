#include "EncoderMotor.hpp"
#include <Arduino.h>


EncoderMotor::EncoderMotor(const uint8_t pinFwd, 
                          const uint8_t pinBck, 
                          const uint8_t pinEnFwd, 
                          const uint8_t pinEnBck, 
                          const uint8_t pinA, 
                          const uint8_t pinB) :
                              ibt2_(std::make_unique<IBT2>(pinFwd, 
                                                           pinBck, 
                                                           pinEnFwd, 
                                                           pinEnBck)),
                              encoder_(std::make_unique<Encoder>(pinA, pinB)),
                              pid_(std::make_unique<PID>())
{
  pid_->setAntiwindup(false);
  pid_->setOutputLimits(-100.0, 100.0);
}


void EncoderMotor::initialiseUKF(const float motorKt,
                                const float motorJ,
                                const float alpha,
                                const float beta,
                                const float kappa,
                                const Eigen::VectorXf &sensorNoise,
                                const Eigen::MatrixXf &processNoise) {
  UKFModel model(2, 1);
  model.stateTransition = [motorKt, motorJ] (const Eigen::VectorXf& state, const Eigen::VectorXf& control, float deltaSec) {
    float pos = state(0);
    float vel = state(1);
    float current = control(0);
    float accel = (motorKt * current) / motorJ;

    Eigen::VectorXf nextState(2);
    nextState(0) = pos + (vel * deltaSec) + (0.5 * accel * deltaSec * deltaSec);
    nextState(1) = vel + (accel * deltaSec);
    return nextState;
  };

  model.measurementModel = [] (const Eigen::VectorXf& state) {
    Eigen::VectorXf measurement(1);
    measurement(0) = state(0);
    return measurement;
  };

  ukf_ = std::make_unique<UKF>(model, alpha, beta, kappa, sensorNoise, processNoise);
}


void EncoderMotor::initialiseMotor(const uint8_t pwmResolutionBits, 
                                  const uint32_t pwmFreq) 
{
  ibt2_->configPWM(pwmResolutionBits, pwmFreq);
}


void EncoderMotor::initialiseWattmeter(const uint8_t address, const float readCurrent, const float measuredCurrent) {
  ina219_ = std::make_unique<INA219Local>(address);
  ina219_->calibrate(ina226_average_enum::INA226_1024_SAMPLES,
                    ina226_timing_enum::INA226_140_us,
                    ina226_timing_enum::INA226_140_us,
                    0.1f,
                    0.1f);
}


void EncoderMotor::setPulsePerRev(const uint32_t pulsePerRev) {
  stepAngle_ = 2.0 * M_PI / pulsePerRev;
}


void EncoderMotor::pidSetGains(const float gainP, 
                              const float gainI, 
                              const float gainD) 
{
  pid_->setGains(gainP, gainI, gainD);
}


void EncoderMotor::pidSetFilterCoeff(float coeffLowPass) {
  pid_->setFilterCoeff(coeffLowPass);
}


void EncoderMotor::setSpeed(float speed) {
  setpointSpeed_ = speed / stepAngle_;
}


Eigen::Vector2f EncoderMotor::update(const uint32_t delta) {
  Eigen::VectorXf steps = Eigen::VectorXf::Zero(1);
  steps(0) = encoder_->getRotation();
  Eigen::VectorXf current = Eigen::VectorXf::Zero(1);
  current(0) = ina219_->getCurrent();
  ukf_->step(steps, current, delta);
  Eigen::Vector2f encoderData = ukf_->getState();
  float powerPercent = pid_->calculateOutput(encoderData(1), setpointSpeed_, delta);
  ibt2_->setSpeed(powerPercent);
  return encoderData;
}
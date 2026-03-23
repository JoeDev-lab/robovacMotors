#ifndef UKF_HPP
#define UKF_HPP

#include <Arduino.h>
#include <cstdint>
#include <functional>
#include <ArduinoEigenDense.h>

/// @brief UKF model. \n
///        Define the size and model of the UKF.
struct UKFModel {
  UKFModel(const uint8_t dimState, 
           const uint8_t dimMeasure)
              : dimState(dimState), 
                dimMeasure(dimMeasure) {}
  /// @brief Predicts next state based on current state and control input
  std::function<Eigen::VectorXf(const Eigen::VectorXf&, const Eigen::VectorXf&, float)> stateTransition;
  

  /// @brief Maps the state to the measurement space
  std::function<Eigen::VectorXf(const Eigen::VectorXf&)> measurementModel;
  
  int dimState; // N
  int dimMeasure; // M
};


class UKF {
public:
  /// @brief Basic constructor.
  /// @param model UKF model.
  /// @param alpha
  /// @param beta
  /// @param kappa
  /// @param sensorNoise Sensor noise of the measurements. Must be in same order as measurements.
  /// @param processNoise Process noise of UKF model.
  UKF(const UKFModel &model,
      const float alpha,
      const float beta,
      const float kappa,
      const Eigen::VectorXf &sensorNoise,
      const Eigen::MatrixXf &processNoise);


  /// @brief update the system state.
  /// @param measurement Measurement vector.
  /// @param control Control value vector.
  /// @param delta [ms] Delta time since last update.
  void step(const Eigen::VectorXf& measurement, // Z
            const Eigen::VectorXf& control,     // U
            uint32_t delta);


  Eigen::VectorXf getState() { return state_; }
  

private:
  const UKFModel model_;
  float lambda_;
  
  Eigen::VectorXf weightsC_;
  Eigen::VectorXf weightsM_;
  Eigen::VectorXf state_;         // X
  Eigen::MatrixXf covariance_;    // P
  Eigen::MatrixXf processNoise_;  // Q
  Eigen::MatrixXf sensorNoise_;   // R


  Eigen::MatrixXf generateSigmas(void);
};

#endif // UKF_HPP

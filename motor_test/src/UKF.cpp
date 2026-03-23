#include "UKF.hpp"

UKF::UKF(const UKFModel &model,
        const float alpha,
        const float beta,
        const float kappa,
        const Eigen::VectorXf &sensorNoise,
        const Eigen::MatrixXf &processNoise)
            : model_(model)
{
  state_ = Eigen::VectorXf::Zero(model_.dimState);
  covariance_ = Eigen::MatrixXf::Identity(model_.dimState, model_.dimState);
  processNoise_ = processNoise;
  sensorNoise_ = Eigen::MatrixXf::Identity(model_.dimMeasure, model_.dimMeasure);

  for (size_t i = 0; i < model_.dimMeasure; i++) {
    sensorNoise_(i, i) = sensorNoise(i);
  }

  lambda_ = alpha * alpha * (model_.dimState + kappa) - model_.dimState;
  weightsM_ = Eigen::VectorXf::Zero(2 * model_.dimState + 1);
  weightsC_ = weightsM_;
  weightsM_(0) = lambda_ / (model_.dimState + lambda_);
  weightsC_(0) = weightsM_(0) + 1 - (alpha * alpha) + beta;

  for (int i = 1; i < (2 * model_.dimState) + 1; ++i) {
    weightsM_(i) = weightsC_(i) = 1.0 / (2.0 * (model_.dimState + lambda_));
  }
}


void UKF::step(const Eigen::VectorXf& measurement, 
              const Eigen::VectorXf& control, 
              uint32_t delta) {
  // Predict
  Eigen::MatrixXf sigmas = generateSigmas();
  Eigen::MatrixXf sigmas_f(model_.dimState, 2 * model_.dimState + 1);

  for (int i=0; i < sigmas.cols(); ++i) {
    sigmas_f.col(i) = model_.stateTransition(sigmas.col(i), control, static_cast<float>(delta) / 1000.0f);
  }

  Eigen::VectorXf x_pred = Eigen::VectorXf::Zero(model_.dimState);

  for (int i=0; i < sigmas_f.cols(); ++i) {
    x_pred += weightsM_(i) * sigmas_f.col(i);
  }

  Eigen::MatrixXf P_pred = processNoise_;

  for (int i=0; i < sigmas_f.cols(); ++i) {
    Eigen::VectorXf diff = sigmas_f.col(i) - x_pred;
    P_pred += weightsC_(i) * diff * diff.transpose();
  }

  // Measurements
  Eigen::MatrixXf Z_sigmas(model_.dimMeasure, 2 * model_.dimState + 1);

  for (int i=0; i < sigmas_f.cols(); ++i) {
    Z_sigmas.col(i) = model_.measurementModel(sigmas_f.col(i));
  }

  Eigen::VectorXf z_pred = Eigen::VectorXf::Zero(model_.dimMeasure);

  for (int i=0; i < Z_sigmas.cols(); ++i) {
    z_pred += weightsM_(i) * Z_sigmas.col(i);
  }

  Eigen::MatrixXf innovationCov = sensorNoise_;
  Eigen::MatrixXf crossCov = Eigen::MatrixXf::Zero(model_.dimState, model_.dimMeasure);

  for(int i=0; i < Z_sigmas.cols(); ++i) {
    Eigen::VectorXf z_diff = Z_sigmas.col(i) - z_pred;
    Eigen::VectorXf x_diff = sigmas_f.col(i) - x_pred;
    innovationCov += weightsC_(i) * z_diff * z_diff.transpose();
    crossCov += weightsC_(i) * x_diff * z_diff.transpose();
  }

  // Update
  Eigen::MatrixXf gain = crossCov * innovationCov.inverse();

  state_ = x_pred + (gain * (measurement - z_pred));
  covariance_ = P_pred - (gain * innovationCov * gain.transpose());
  covariance_.diagonal().array() += 1e-9; // Adding jitter to keep positive
}


Eigen::MatrixXf UKF::generateSigmas(void) {
  Eigen::MatrixXf L = covariance_.llt().matrixL();
  Eigen::MatrixXf sigmas(model_.dimState, (2 * model_.dimState) + 1);
  sigmas.col(0) = state_;
  double scale = std::sqrt(model_.dimState + lambda_);

  for (int i = 0; i < model_.dimState; ++i) {
    sigmas.col(i + 1) = state_ + scale * L.col(i);
    sigmas.col(i + 1 + model_.dimState) = state_ - scale * L.col(i);
  }

  return sigmas;
}
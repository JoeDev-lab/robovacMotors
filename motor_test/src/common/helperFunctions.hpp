#ifndef HELPERFUNCTIONS_HPP
#define HELPERFUNCTIONS_HPP

#include <cstdint>
#include <vector>
#include <cmath>
#include <esp32-hal-log.h>
#include <limits>


class HelperFunctions {
public:
  HelperFunctions(void) {};

  /// @brief Calculate the mean value in a vector
  /// @param data Data to calculate mean of.
  /// @return float Mean value.
  template <typename Iterator>
  static float calculateMean(Iterator begin, Iterator end) 
  {
    if (begin == end) return 0.0f;

    float sum = 0;
    size_t count = 0;

    for (auto it = begin; it != end; ++it) {
      sum += *it;
      count++;
    }

    return sum / static_cast<float>(count);
  }

  /// @brief Calculate the standard deviation of a vector of data.
  /// @param data Data to calculate standard deviation of.
  /// @return float Standard deviation.
  template <typename Iterator>
  static float standardDeviation(Iterator begin, Iterator end, float mean = INFINITY) 
  {
    if (begin == end) return 0.0f;

    if (std::isinf(mean)) {
      mean = calculateMean(begin, end);
    }

    float variance = 0.0;
    size_t count = 0;

    for (auto it = begin; it != end; ++it) {
      float delta = *it - mean;
      variance += delta * delta;
      count++;
    }

    return variance /= count;
  }


  /// @brief Calculates the sigma clipped mean of a vector.
  /// @param data Data to calculate mean of.
  /// @param sigma Standard deviation from mean to include.
  /// @return Sigma clipped mean.
  template <typename Iterator>
  static float calculateClippedMean(Iterator begin, Iterator end, float sigma, float meanWhole = INFINITY)
  {
    if (begin == end) return 0.0f;

    if (std::isinf(meanWhole)) {
      meanWhole = calculateMean(begin, end);
    }

    float standardDeviation = HelperFunctions::standardDeviation(begin, end, meanWhole);
    float sum = 0;
    size_t count = 0;

    for (auto it = begin; it != end; ++it) {
      if (*it > meanWhole + (standardDeviation * sigma) || *it < meanWhole - (standardDeviation * sigma)) {
        continue;
      }

      sum += *it;
      count++;
    }

    return sum /= count;
  }


  template <typename Iterator>
  static float calculateRMS(Iterator begin, Iterator end) {
    if (begin == end) return 0.0f;

    float sumSquares = 0.0f;
    size_t count = 0;
    
    for (auto it = begin; it != end; ++it) {
      sumSquares += (*it * *it);
      count++;
    }

    return sqrt(sumSquares / static_cast<float>(count));
  }


  /// @brief Extract the roll component from a quaternion.
  /// @param quat quaternion.
  /// @return roll component.
  static float quatToRoll(const Eigen::Quaternionf& quat) {
    Eigen::Quaternionf normalisedQuat = quat.normalized();
    float squaredY = normalisedQuat.y() * normalisedQuat.y();

    float t0 = +2.0f * ((normalisedQuat.w() * normalisedQuat.x()) + (normalisedQuat.y() * normalisedQuat.z()));
    float t1 = +1.0f - 2.0f * (normalisedQuat.x() * normalisedQuat.y() + squaredY);
    float roll = atan2(t0, t1);

    return roll;
  }


  /// @brief Extract the pitch component from a quaternion.
  /// @param quat quaternion.
  /// @return pitch component.
  static float quatToPitch(const Eigen::Quaternionf& quat) {
    Eigen::Quaternionf normalisedQuat = quat.normalized();
    float squaredY = normalisedQuat.y() * normalisedQuat.y();

    float t2 = 2.0f * ((normalisedQuat.w() * normalisedQuat.y()) - (normalisedQuat.z() * normalisedQuat.x()));
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
    float pitch = asin(t2);

    return pitch;
  }


  /// @brief Extract the yaw component from a quaternion.
  /// @param quat quaternion.
  /// @return yaw component.
  static float quatToYaw(const Eigen::Quaternionf& quat) {
    Eigen::Quaternionf normalisedQuat = quat.normalized();
    float squaredY = normalisedQuat.y() * normalisedQuat.y();

    float t3 = 2.0f * ((normalisedQuat.w() * normalisedQuat.z()) + (normalisedQuat.x() * normalisedQuat.y()));
    float t4 = 1.0f - 2.0f * (squaredY + (normalisedQuat.z() * normalisedQuat.z()));
    float yaw = atan2(t3, t4);
    
    return yaw;
  }


  /// @brief Convert a quaternion to roll, pitch and yaw.
  /// @param quat quaternion.
  /// @return [roll, pitch, yaw]
  static std::array<float, 3> quatToEuler(const Eigen::Quaternionf& quat) {
    Eigen::Quaternionf normalisedQuat = quat.normalized();
    float squaredY = normalisedQuat.y() * normalisedQuat.y();

    // roll (x-axis rotation)
    float t0 = 2.0f * ((normalisedQuat.w() * normalisedQuat.x()) + (normalisedQuat.y() * normalisedQuat.z()));
    float t1 = 1.0f - 2.0f * ((normalisedQuat.x() * normalisedQuat.y()) + squaredY);
    float roll = atan2(t0, t1);

    // pitch (y-axis rotation)
    float t2 = 2.0f * ((normalisedQuat.w() * normalisedQuat.y()) - (normalisedQuat.z() * normalisedQuat.x()));
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
    float pitch = asin(t2);

    // yaw (z-axis rotation)
    float t3 = 2.0f * ((normalisedQuat.w() * normalisedQuat.z()) + (normalisedQuat.x() * normalisedQuat.y()));
    float t4 = 1.0f - 2.0f * (squaredY + (normalisedQuat.z() * normalisedQuat.z()));
    float yaw = atan2(t3, t4);

    return {roll, pitch, yaw};
  }
};

#endif // HELPERFUNCTIONS_HPP

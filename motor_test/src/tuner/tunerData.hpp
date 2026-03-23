#ifndef TUNERDATA_HPP
#define TUNERDATA_HPP

#include <vector>
#include <cstdint>


struct TunerOutput {
  double kp;
  double ki;
  double kd;

  TunerOutput(const double kpIn = 0.0, 
              const double kiIn = 0.0, 
              const double kdIn = 0.0) 
                : kp(kpIn), 
                  ki(kiIn), 
                  kd(kdIn) {};
};


struct SteadyStateData {
  double average;               // Error value at steady state
  double settlingTime;          // [ms] Time to reach steady state

  SteadyStateData(const double averageIn = 0.0, 
                  const double settlingTimeIn = 0.0)
                    : average(averageIn),
                      settlingTime(settlingTimeIn) {};
};


struct ErrorData {
  double IAE;   // integral absolute error
  double ISE;   // integral square error
  double ITAE;  // integral time absolute error
  double ITSE;  // integral time square error
  double ISTE;  // integral square time error

  ErrorData(const double IAEIn = 0.0, 
            const double ISEIn = 0.0, 
            const double ITAEIn = 0.0, 
            const double ITSEIn = 0.0, 
            const double ISTEIn = 0.0)
                : IAE(IAEIn), 
                  ISE(ISEIn), 
                  ITAE(ITAEIn), 
                  ITSE(ITSEIn), 
                  ISTE(ISTEIn) {};
};


struct TestOutput {
  double riseTime;                  // [ms] Time to reach 90% of setpoint        
  double overshoot;                 // [%] Percentage of overshoot
  double oscillationPeriod;         // [ms] Period of oscillation
  double oscillationAmplitude;      // [rad/s] Amplitude of oscillation
  SteadyStateData steadyStateData;  // Steady state data
  ErrorData errorData;              // Error data

  TestOutput(const double riseTimeIn = 0.0,
             const double overshootIn = 0.0,
             const double oscillationPeriodIn = 0.0,
             const double oscillationAmplitudeIn = 0.0,
             const SteadyStateData steadyStateDataIn = SteadyStateData(),
             const ErrorData errorDataIn = ErrorData())
                : riseTime(riseTimeIn), 
                  overshoot(overshootIn),
                  oscillationPeriod(oscillationPeriodIn),
                  oscillationAmplitude(oscillationAmplitudeIn),
                  steadyStateData(steadyStateDataIn),
                  errorData(errorDataIn) {};
};


/// @brief Data structure for test data
struct TestData {
  struct DataPoint {
      const double rotation;
      const double velocity;
      const double acceleration;
      const double setpoint;
      const uint32_t time;
      
      DataPoint(const double rotationIn = 0.0,
                const double velocityIn = 0.0,
                const double accelerationIn = 0.0,
                const double setpointIn = 0.0,
                const uint32_t timeIn = 0)
                  : rotation(rotationIn), 
                    velocity(velocityIn), 
                    acceleration(accelerationIn), 
                    setpoint(setpointIn), 
                    time(timeIn) {};
  };


  std::vector<DataPoint> data;


  /// @brief Reserves memory for data on construction.
  /// @param sizeIn Size to reserve.
  TestData(const uint32_t sizeIn = 0) {
    data.reserve(sizeIn);
  };


  /// @brief Add a new data point.
  /// @param rotation [rad] Angular rotation.
  /// @param velocity [rad/s] Angular velocity.
  /// @param acceleration [rad/s^2] Angular acceleration.
  /// @param setpoint [rad/s] Angular velocity setpoint.
  /// @param time [ms] Timestamp.
  void addPoint(const double rotation, 
                const double velocity, 
                const double acceleration, 
                const double setpoint, 
                const uint32_t time) {
    data.emplace_back(rotation, velocity, acceleration, setpoint, time);
  };


  /// @brief Returns the run time of the data.
  /// @return [ms] run time 
  uint32_t getDuration() {
    return data.back().time - data.front().time;
  }


  /// @brief Returns the number of data points.
  size_t size(void) {
    return data.size();
  }

  
  /// @brief Returns a specific data point.
  DataPoint getPoint(const size_t index) {
    return data.at(index);
  }


  /// @brief Returns a vector of all rotation data points.
  /// @return std::vector<double> Rotation data.
  std::vector<double> getRotationData() {
    std::vector<double> rotationData(data.size(), 0.0);
    for (size_t i = 0; i < data.size(); i++) {
      rotationData[i] = data.at(i).rotation;
    }
    return rotationData;
  }


  /// @brief Returns a vector of all velocity data points.
  /// @return std::vector<double> Velocity data.
  std::vector<double> getVelocityData() {
    std::vector<double> velocityData(data.size(), 0.0);
    for (size_t i = 0; i < data.size(); i++) {
      velocityData[i] = data.at(i).velocity;
    }
    return velocityData;
  }


  /// @brief Returns a vector of all acceleration data points.
  /// @return std::vector<double> Acceleration data.
  std::vector<double> getAccelerationData() {
    std::vector<double> accelerationData(data.size(), 0.0);
    for (size_t i = 0; i < data.size(); i++) {
      accelerationData[i] = data.at(i).acceleration;
    }
    return accelerationData;
  }


  /// @brief Returns a vector of all setpoint data points.
  /// @return std::vector<double> Setpoint data.
  std::vector<double> getSetpointData() {
    std::vector<double> setpointData(data.size(), 0.0);
    for (size_t i = 0; i < data.size(); i++) {
      setpointData[i] = data.at(i).setpoint;
    }
    return setpointData;
  }


  /// @brief Returns a vector of all acceleration time points.
  /// @return std::vector<uint32_t> Time data.
  std::vector<uint32_t> getTimeData() {
    std::vector<uint32_t> timeData(data.size(), 0.0);
    for (size_t i = 0; i < data.size(); i++) {
      timeData[i] = data.at(i).time;
    }
    return timeData;
  }


  /// @brief Get all time points as vector of deltas.
  /// @return Vector of time deltas.
  std::vector<uint32_t> getDeltaTime(void) {
    std::vector<uint32_t> time(data.size(), 0.0);
    time.reserve(data.size() - 1);

    for (size_t i = 1; i < data.size(); i++) {
      time.push_back(data.at(i).time - data.at(i - 1).time);
    }

    return time;
  }


  /// @brief Trims data vector to fit size.
  void trim(void) {
    if (data.capacity() > 0) {
      data.shrink_to_fit();
    }
  };
};


/// @brief Data structure for inflection points. Each point holds the value and time of inflection.
struct InflectionData {
  enum class InflectionType {
    NONE,
    MAXIMA,
    MINIMA,
    FLAT,
    IGNORE
  };
  
  /// @brief Inflection point data.
  struct InflectionPoint {
    InflectionType type;  //!< Inflection point type.
    double value;         //!< Value at inflection point.
    uint32_t time;        //!< Timestamp of inflection point.


    InflectionPoint(const InflectionType typeIn = InflectionType::NONE, 
                    const double valueIn = 0.0, 
                    const uint32_t timeIn = 0) 
                      : type(typeIn),
                        value(valueIn), 
                        time(timeIn) {};


    InflectionPoint& operator=(const InflectionPoint& other) {
      return *this;
    };
  };


  std::vector<InflectionPoint> inflectionPoints;


  /// @brief Generic constructor
  /// @param expectedSize Expected number of data entries. Used to reserve memory.
  InflectionData(size_t expectedSize = 0) {
    inflectionPoints.reserve(expectedSize);
  };


  /// @brief Add a new data point.
  /// @param type Type of inflection point.
  /// @param value Value of data point
  /// @param time [ms] Timestamp of data point
  void addPoint(const InflectionType type, const double value, const uint32_t time) {
    InflectionPoint newPoint = InflectionPoint(type, value, time);
    inflectionPoints.push_back(newPoint);
  };


  /// @brief Update an existing data point.
  /// @param index Index of data point
  /// @param value Value of data point
  /// @param time [ms] Timestamp of data point
  /// @param type Type of inflection point.
  void updatePoint(const size_t index, const double value, const uint32_t time, const InflectionType type = InflectionType::IGNORE) {
    if (index >= inflectionPoints.size()) {
      Serial.println("Cannot update inflection data point: Index out of range");
      return;
    }


    InflectionType newType = inflectionPoints.at(index).type;

    if (type != InflectionType::IGNORE) {
      inflectionPoints.at(index).type = type;
    }

    InflectionPoint newPoint = InflectionPoint(newType, value, time);
    inflectionPoints.at(index) = newPoint;
  };

  
  /// @brief Get a specific data point
  /// @param index Index of data point
  /// @return Inflection point
  InflectionPoint getPoint(const size_t index) {
    if (index >= inflectionPoints.size()) {
      return InflectionPoint();
    } 

    return inflectionPoints.at(index);
  };


  /// @brief Get number of data points
  size_t size(void) {
    return inflectionPoints.size();
  };


  void trim(void) {
    if (inflectionPoints.capacity() > 0) {
      inflectionPoints.shrink_to_fit();
    }
  }

  
  /// @brief Get all data points as vector
  /// @return Vector of data.
  std::vector<double> data(InflectionType type = InflectionType::IGNORE) {
    std::vector<double> data;
    data.reserve(inflectionPoints.size());

    for (InflectionPoint point : inflectionPoints) {
      if (type != InflectionType::IGNORE && point.type != type) {
        continue;
      }

      data.push_back(point.value);
    }
    return data;
  }


  /// @brief Get all time points as vector
  /// @return Vector of times.
  std::vector<double> time(InflectionType type = InflectionType::IGNORE) {
    std::vector<double> time;
    time.reserve(inflectionPoints.size());

    for (InflectionPoint point : inflectionPoints) {
      if (type != InflectionType::IGNORE && point.type != type) {
        continue;
      }

      time.push_back(point.time);
    }
    return time;
  }


  /// @brief Get all time points as vector of deltas.
  /// @return Vector of time deltas.
  std::vector<double> deltaTime(void) {
    std::vector<double> time;
    time.reserve(inflectionPoints.size() - 1);

    for (size_t i = 1; i < inflectionPoints.size(); i++) {
      time.push_back(inflectionPoints.at(i).time - inflectionPoints.at(i - 1).time);
    }

    return time;
  }
};

#endif // TUNERDATA_HPP

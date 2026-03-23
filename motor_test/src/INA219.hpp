#ifndef INA219_HPP
#define INA219_HPP

#include <Arduino.h>
#include <cstdint>
#include <INA226.h>
#include <memory>
#include <Wire.h>

// INA219 addresses
// 0x40   A0 = 0  A1 = 0
// 0x41   A0 = 1  A1 = 0
// 0x44   A0 = 0  A1 = 1
// 0x45   A0 = 1  A1 = 1


class INA219Local {
public:
  /// @brief Basic constructor. Initialises sensor.
  /// @param address Sensor address.
  INA219Local(const uint8_t address);

  
  /// @brief Sets sensor parameters.
  /// @param range Voltage range [16V, 32V].
  /// @param resolution Sensor resolution bits [9, 10, 11, 12].
  /// @param sampleSize Number of samples per measurement [1, 2, 4, 8, 16, 32, 64, 128].
  /// @param gain Sensor gain [1, 2, 4, 8].
  /// @param calibrationSensor [mA] Sensor calibration reading.
  /// @param calibrationExternal [mA] External sensor calibration reading.
  void calibrate(const ina226_average_enum averageSamples,
                const ina226_timing_enum conversionTimeBus,
                const ina226_timing_enum conversionTimeShunt,
                const float calibrationSensor = 0.1f, 
                const float calibrationExternal = 0.1f);

  float getVoltage(void) { return ina219_->getBusVoltage(); };
  float getCurrent(void) { return ina219_->getCurrent(); };
  float getPower(void) { return ina219_->getPower(); };

private:
  std::unique_ptr<INA226> ina219_;
};

#endif // INA219_HPP

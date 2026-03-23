#include "INA219.hpp"
#include <esp32-hal-log.h>
#include <freertos/FreeRTOS.h>

INA219Local::INA219Local(const uint8_t address)
                  : ina219_(std::make_unique<INA226>(address)) 
{              
  if (ina219_->begin() != true) {
    log_e("INA219 failed");
    while(1) vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void INA219Local::calibrate(const ina226_average_enum averageSamples,
                      const ina226_timing_enum conversionTimeBus,
                      const ina226_timing_enum conversionTimeShunt,
                      const float calibrationSensor, 
                      const float calibrationExternal) {
  ina219_->configure(calibrationSensor, calibrationExternal);
  ina219_->setAverage(averageSamples);
  ina219_->setBusVoltageConversionTime(conversionTimeBus);
  ina219_->setShuntVoltageConversionTime(conversionTimeShunt);
  ina219_->setMaxCurrentShunt();
}
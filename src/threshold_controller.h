#pragma once
#include <Arduino.h>
#include "app_config.h"
#include "modbus_manager.h"

class ThresholdController {
public:
  explicit ThresholdController(ModbusManager &modbus) : modbus_(modbus) {}

  void evaluate(const app::ControlConfig &cfg,
                uint16_t temperatureReading,
                uint16_t humidityReading);

private:
  void controlHumidity_(const String &comparator, uint16_t reading, int setpoint);
  void controlTemperature_(const String &comparator, uint16_t reading, int setpoint);

  ModbusManager &modbus_;
};
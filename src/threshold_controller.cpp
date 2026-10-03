#include "threshold_controller.h"

void ThresholdController::evaluate(const app::ControlConfig &cfg,
                                   uint16_t temperatureReading,
                                   uint16_t humidityReading) {
  controlHumidity_(cfg.comparatorHumidity, humidityReading, cfg.thresholdHumidity);
  controlTemperature_(cfg.comparatorTemperature, temperatureReading, cfg.thresholdTemperature);
}

void ThresholdController::controlHumidity_(const String &comparator,
                                           uint16_t reading, int setpoint) {
  if (comparator == ">") {
    if (reading > setpoint) {
      modbus_.readRelayCoils(2, 4);
      modbus_.setRelayTransmitBuffer(0, 0b1010);
      modbus_.writeRelayMultipleCoils(2, 4);
      Serial.println("[Ctrl] Fan ON");
    } else if (reading < setpoint) {
      modbus_.readRelayCoils(2, 4);
      modbus_.setRelayTransmitBuffer(0, 0b0101);
      modbus_.writeRelayMultipleCoils(3, 4);
      Serial.println("[Ctrl] Fan OFF");
    }
  }
  // TODO: '<', '>=', '<='
}

void ThresholdController::controlTemperature_(const String &comparator,
                                              uint16_t reading, int setpoint) {
  if (comparator == ">") {
    if (reading > setpoint) {
      modbus_.readRelayCoils(0, 2);
      modbus_.setRelayTransmitBuffer(0, 0b00);
      modbus_.writeRelayMultipleCoils(0, 2);
    } else {
      modbus_.readRelayCoils(0, 2);
      modbus_.setRelayTransmitBuffer(0, 0b11);
      modbus_.writeRelayMultipleCoils(0, 2);
    }
  }
  // TODO: '<', '>=', '<='
}
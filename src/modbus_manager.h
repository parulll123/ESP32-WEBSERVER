#pragma once
#include <Arduino.h>
#include <ModbusMaster.h>
#include "app_config.h"

class ModbusManager {
public:
  void begin(const app::ModbusSlaveConfig &slave1,
             const app::ModbusSlaveConfig &slave2,
             int rxPin, int txPin);

  // ---- slave #2 (sensor) ----
  uint16_t readSensorRegister(uint16_t address, uint16_t quantity);

  // ---- slave #1 (relay) ----
  uint16_t readRelayCoils(uint16_t address, uint16_t quantity);
  void     setRelayTransmitBuffer(uint8_t index, uint16_t value);
  bool     writeRelayMultipleCoils(uint16_t startAddress, uint16_t quantity);

private:
  bool acquireBus_();
  void releaseBus_();

  ModbusMaster relayNode_;
  ModbusMaster sensorNode_;
  unsigned long lastTransactionMs_ = 0;
  static constexpr unsigned long kMinIntervalMs = 250;
};
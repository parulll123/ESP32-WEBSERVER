#include "modbus_manager.h"

namespace {
uint32_t buildSerialFormat(int dataBits, const String &parity, int stopBits) {
  if (dataBits == 8) {
    if (parity == "even") return (stopBits == 2) ? SERIAL_8E2 : SERIAL_8E1;
    if (parity == "odd")  return (stopBits == 2) ? SERIAL_8O2 : SERIAL_8O1;
    return                       (stopBits == 2) ? SERIAL_8N2 : SERIAL_8N1;
  }
  // 7 data bits
  if (parity == "even") return (stopBits == 2) ? SERIAL_7E2 : SERIAL_7E1;
  if (parity == "odd")  return (stopBits == 2) ? SERIAL_7O2 : SERIAL_7O1;
  return                       (stopBits == 2) ? SERIAL_7N2 : SERIAL_7N1;
}
} // namespace

void ModbusManager::begin(const app::ModbusSlaveConfig &slave1,
                          const app::ModbusSlaveConfig &slave2,
                          int rxPin, int txPin) {
  // Both slaves sit on the same physical RS-485 bus,
  // so Serial2 is configured once using slave1's link settings.
  const uint32_t serialFormat =
      buildSerialFormat(slave1.dataBits, slave1.parity, slave1.stopBits);

  Serial2.begin(slave1.baudRate, serialFormat, rxPin, txPin);
  relayNode_.begin(slave1.id, Serial2);
  sensorNode_.begin(slave2.id, Serial2);
  delay(100);

  Serial.printf("[Modbus] slave1 id=%u @%d, slave2 id=%u\n",
                slave1.id, slave1.baudRate, slave2.id);
}

bool ModbusManager::acquireBus_() {
  if (millis() - lastTransactionMs_ < kMinIntervalMs) return false;
  return true;
}

void ModbusManager::releaseBus_() {
  delay(100);
  lastTransactionMs_ = millis();
}

uint16_t ModbusManager::readSensorRegister(uint16_t address, uint16_t quantity) {
  if (!acquireBus_()) return 0;
  uint16_t value = 0;
  if (sensorNode_.readHoldingRegisters(address, quantity) == sensorNode_.ku8MBSuccess) {
    value = sensorNode_.getResponseBuffer(0);
  } else {
    Serial.printf("[Modbus] sensor read failed @%u\n", address);
  }
  releaseBus_();
  return value;
}

uint16_t ModbusManager::readRelayCoils(uint16_t address, uint16_t quantity) {
  if (!acquireBus_()) return 0;
  uint16_t value = 0;
  if (relayNode_.readCoils(address, quantity) == relayNode_.ku8MBSuccess) {
    value = relayNode_.getResponseBuffer(0);
  } else {
    Serial.println("[Modbus] relay coil read failed");
  }
  releaseBus_();
  return value;
}

void ModbusManager::setRelayTransmitBuffer(uint8_t index, uint16_t value) {
  relayNode_.setTransmitBuffer(index, value);
}

bool ModbusManager::writeRelayMultipleCoils(uint16_t startAddress, uint16_t quantity) {
  if (!acquireBus_()) return false;
  const bool ok = relayNode_.writeMultipleCoils(startAddress, quantity) == relayNode_.ku8MBSuccess;
  if (!ok) Serial.println("[Modbus] writeMultipleCoils failed");
  releaseBus_();
  return ok;
}
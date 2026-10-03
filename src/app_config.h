#pragma once
#include <Arduino.h>

namespace app {

struct NetworkConfig {
  String  ssid;
  String  password;
  uint8_t staticIp[4]   = {192, 168, 0, 2};
  uint8_t gatewayIp[4]  = {192, 168, 0, 1};
  uint8_t subnetMask[4] = {255, 255, 255, 0};
  uint8_t primaryDns[4] = {8, 8, 8, 8};
};

struct MqttConfig {
  String broker;
  int    port         = 1883;
  String user;
  String password;
  String publishTopic;
};

struct ModbusSlaveConfig {
  String   protocol = "RTU";
  uint8_t  id       = 1;
  int      baudRate = 9600;
  int      dataBits = 8;
  int      stopBits = 1;
  String   parity   = "none";
  uint16_t registers[4] = {0, 0, 0, 0};
  String   dataTypes[4] = {"UINT16", "UINT16", "UINT16", "UINT16"};
};

struct ControlConfig {
  String comparatorTemperature = ">";
  String comparatorHumidity    = ">";
  String comparatorLux         = ">";
  int    thresholdTemperature  = 0;
  int    thresholdHumidity     = 0;
  int    thresholdLux          = 0;
};

struct AppConfig {
  NetworkConfig     network;
  MqttConfig        mqtt;
  ModbusSlaveConfig slave1;
  ModbusSlaveConfig slave2;
  ControlConfig     control;
};

} // namespace app
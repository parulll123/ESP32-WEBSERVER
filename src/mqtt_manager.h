#pragma once
#include <Arduino.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include "app_config.h"

class MqttManager {
public:
  void begin(const app::MqttConfig &cfg);
  void loop();
  void ensureConnected(const app::MqttConfig &cfg);
  bool publish(const char *topic, const char *payload);

private:
  void publishBootDiagnostics_(const app::MqttConfig &cfg);

  WiFiClient   transport_;
  PubSubClient client_{transport_};
  app::MqttConfig cfg_;
  unsigned long lastReconnectAttemptMs_ = 0;
  static constexpr unsigned long kReconnectIntervalMs = 5000;
};
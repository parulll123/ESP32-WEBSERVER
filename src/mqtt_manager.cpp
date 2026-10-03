#include "mqtt_manager.h"
#include "app_config.h"

namespace {
String ipToString(const uint8_t ip[4]) {
  return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}
} // namespace

void MqttManager::begin(const app::MqttConfig &cfg) {
  cfg_ = cfg;
  client_.setServer(cfg_.broker.c_str(), cfg_.port);
}

void MqttManager::loop() {
  client_.loop();
}

void MqttManager::ensureConnected(const app::MqttConfig &cfg) {
  if (client_.connected()) return;

  const unsigned long now = millis();
  if (now - lastReconnectAttemptMs_ < kReconnectIntervalMs) return;
  lastReconnectAttemptMs_ = now;

  // Broker config might have been updated
  if (cfg.broker != cfg_.broker || cfg.port != cfg_.port) {
    cfg_ = cfg;
    client_.setServer(cfg_.broker.c_str(), cfg_.port);
  }

  String clientId = "ESP32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.printf("[MQTT] Connecting to %s:%d ... ", cfg_.broker.c_str(), cfg_.port);

  if (!client_.connect(clientId.c_str(), cfg_.user.c_str(), cfg_.password.c_str())) {
    Serial.printf("failed, rc=%d\n", client_.state());
    return;
  }

  Serial.println("ok");
  publishBootDiagnostics_(cfg_);
}

bool MqttManager::publish(const char *topic, const char *payload) {
  if (topic == nullptr || topic[0] == '\0') return false;
  return client_.publish(topic, payload);
}

void MqttManager::publishBootDiagnostics_(const app::MqttConfig &cfg) {
  // Kept minimal; extend as needed.
  String json = "{\"ip\":\"" + ipToString(cfg.port ? (const uint8_t[]){0,0,0,0} : (const uint8_t[]){0,0,0,0}) + "\"}"; // placeholder
  // Real payload:
  String payload = "{";
  payload += "\"MQTT Server\":\"" + cfg.broker + "\",";
  payload += "\"MQTT Port\":\"" + String(cfg.port) + "\",";
  payload += "\"MQTT Username\":\"" + cfg.user + "\"";
  payload += "}";
  client_.publish("debugger", payload.c_str());
}
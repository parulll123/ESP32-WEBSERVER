#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

#include "app_config.h"
#include "config_store.h"
#include "network_manager.h"
#include "mqtt_manager.h"
#include "modbus_manager.h"
#include "threshold_controller.h"
#include "web_configurator.h"

// ---- Hardware ----
static constexpr int kModbusRxPin = 33;
static constexpr int kModbusTxPin = 32;

// ---- AP ----
static constexpr const char *kAccessPointSsid     = "ESP32_Configuration";
static constexpr const char *kAccessPointPassword = "12345678";
static constexpr const char *kOtaUser              = "admin";
static constexpr const char *kOtaPassword          = "Together1!";

// ---- Globals ----
static app::AppConfig      g_config;
static AsyncWebServer      g_server(80);
static NetworkManager      g_network;
static MqttManager         g_mqtt;
static ModbusManager       g_modbus;
static ThresholdController g_thresholds(g_modbus);

static WebConfigurator g_web(g_server, g_config, []() {
  ConfigStore::save(g_config);
});

static unsigned long g_lastOtaProgressMs = 0;

// ---- OTA callbacks ----
static void onOtaStart() {
  Serial.println("[OTA] Update started");
}
static void onOtaProgress(size_t current, size_t final) {
  if (millis() - g_lastOtaProgressMs > 1000) {
    g_lastOtaProgressMs = millis();
    Serial.printf("[OTA] %u / %u bytes\n", current, final);
  }
}
static void onOtaEnd(bool success) {
  Serial.println(success ? "[OTA] Done" : "[OTA] Error");
}

// =====================================================================
void setup() {
  Serial.begin(9600);
  delay(200);

  ConfigStore::load(g_config);

  g_network.begin(g_config.network, kAccessPointSsid, kAccessPointPassword);
  g_web.begin();

  ElegantOTA.begin(&g_server, kOtaUser, kOtaPassword);
  ElegantOTA.onStart(onOtaStart);
  ElegantOTA.onProgress(onOtaProgress);
  ElegantOTA.onEnd(onOtaEnd);
  g_server.begin();

  g_modbus.begin(g_config.slave1, g_config.slave2, kModbusRxPin, kModbusTxPin);

  g_mqtt.begin(g_config.mqtt);
}

// =====================================================================
void loop() {
  ElegantOTA.loop();

  g_mqtt.ensureConnected(g_config.mqtt);
  g_mqtt.loop();

  // Poll slave #2 for the four configured registers
  static unsigned long lastPoll[4] = {0, 0, 0, 0};
  uint16_t sensorReadings[4] = {0, 0, 0, 0};

  for (int i = 0; i < 4; i++) {
    if (millis() - lastPoll[i] >= 250) {
      sensorReadings[i] = g_modbus.readSensorRegister(
          g_config.slave2.registers[i], 1);
      g_mqtt.publish(g_config.mqtt.publishTopic.c_str(),
                     String(sensorReadings[i]).c_str());
      lastPoll[i] = millis();
    }
  }

  // readings[0] = humidity, readings[1] = temperature (adjust to your sensor)
  g_thresholds.evaluate(g_config.control, sensorReadings[1], sensorReadings[0]);

  delay(1000);
}
#include "web_configurator.h"
#include "web_page.h"

namespace {
String param(AsyncWebServerRequest *r, const char *name, const char *fallback = "") {
  if (r->hasParam(name, true)) return r->getParam(name, true)->value();
  return String(fallback);
}

int paramInt(AsyncWebServerRequest *r, const char *name, int fallback = 0) {
  if (r->hasParam(name, true)) return r->getParam(name, true)->value().toInt();
  return fallback;
}

void readIp(AsyncWebServerRequest *r, const char *prefix, uint8_t *out) {
  for (int i = 0; i < 4; i++) {
    String key = String(prefix) + String(i);
    out[i] = (uint8_t)paramInt(r, key.c_str(), out[i]);
  }
}
} // namespace

WebConfigurator::WebConfigurator(AsyncWebServer &server,
                                 app::AppConfig &config,
                                 SaveHandler onSave)
    : server_(server), config_(config), onSave_(std::move(onSave)) {}

void WebConfigurator::begin() {
  server_.on("/", HTTP_GET,
             [this](AsyncWebServerRequest *r) { handleRoot_(r); });

  server_.on("/saveConfig", HTTP_POST,
             [this](AsyncWebServerRequest *r) { handleSave_(r); });
}

void WebConfigurator::handleRoot_(AsyncWebServerRequest *request) {
  request->send(200, "text/html", kConfigPage);
}

void WebConfigurator::handleSave_(AsyncWebServerRequest *request) {
  parseForm_(request);

  Serial.println("[Web] Configuration received:");
  Serial.printf("  SSID         : %s\n", config_.network.ssid.c_str());
  Serial.printf("  Static IP    : %u.%u.%u.%u\n",
                config_.network.staticIp[0], config_.network.staticIp[1],
                config_.network.staticIp[2], config_.network.staticIp[3]);
  Serial.printf("  MQTT broker  : %s:%d\n",
                config_.mqtt.broker.c_str(), config_.mqtt.port);
  Serial.printf("  Slave1 id    : %u @ %d\n",
                config_.slave1.id, config_.slave1.baudRate);
  Serial.printf("  Slave2 id    : %u @ %d\n",
                config_.slave2.id, config_.slave2.baudRate);

  if (onSave_) onSave_();

  request->send(200, "text/html",
                "Configuration saved. ESP32 will restart to apply changes.");
  delay(3000);
  ESP.restart();
}

void WebConfigurator::parseForm_(AsyncWebServerRequest *request) {
  auto &n = config_.network;
  auto &m = config_.mqtt;
  auto &s1 = config_.slave1;
  auto &s2 = config_.slave2;
  auto &c = config_.control;

  n.ssid     = param(request, "ssid");
  n.password = param(request, "password");
  readIp(request, "ip",  n.staticIp);
  readIp(request, "gw",  n.gatewayIp);
  readIp(request, "nm",  n.subnetMask);
  readIp(request, "dns", n.primaryDns);

  m.broker       = param(request, "mqttHost");
  m.port         = paramInt(request, "mqttPort", 1883);
  m.user         = param(request, "mqttUser");
  m.password     = param(request, "mqttPass");
  m.publishTopic = param(request, "mqttTopic");

  auto readSlave = [&](const char *tag, app::ModbusSlaveConfig &s) {
    s.protocol = param(request, (String(tag) + "Proto").c_str(), "RTU");
    s.id       = paramInt(request, (String(tag) + "Id").c_str(),   s.id);
    s.baudRate = paramInt(request, (String(tag) + "Baud").c_str(), s.baudRate);
    s.dataBits = paramInt(request, (String(tag) + "Data").c_str(), s.dataBits);
    s.stopBits = paramInt(request, (String(tag) + "Stop").c_str(), s.stopBits);
    s.parity   = param(request, (String(tag) + "Parity").c_str(), "none");
    for (int i = 0; i < 4; i++) {
      s.registers[i] = paramInt(request, (String(tag) + "Reg"  + i).c_str(), 0);
      s.dataTypes[i] = param   (request, (String(tag) + "Type" + i).c_str(), "UINT16");
    }
  };
  readSlave("s1", s1);
  readSlave("s2", s2);

  c.comparatorTemperature = param   (request, "ctlTempCmp", ">");
  c.comparatorHumidity    = param   (request, "ctlHumCmp",  ">");
  c.comparatorLux         = param   (request, "ctlLuxCmp",  ">");
  c.thresholdTemperature  = paramInt(request, "ctlTempThr", 0);
  c.thresholdHumidity     = paramInt(request, "ctlHumThr",  0);
  c.thresholdLux          = paramInt(request, "ctlLuxThr",  0);
}
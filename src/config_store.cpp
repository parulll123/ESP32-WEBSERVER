#include "config_store.h"
#include <Preferences.h>

namespace {
constexpr const char *kNamespace = "WiFiCreds";
Preferences prefs;

// NVS keys must be ≤ 15 chars.
void putIp(Preferences &p, const char *prefix, const uint8_t *ip) {
  for (int i = 0; i < 4; i++) {
    String key = String(prefix) + String(i);
    p.putInt(key.c_str(), ip[i]);
  }
}

void getIp(Preferences &p, const char *prefix, uint8_t *ip) {
  for (int i = 0; i < 4; i++) {
    String key = String(prefix) + String(i);
    ip[i] = p.getInt(key.c_str(), ip[i]);
  }
}

void putSlave(Preferences &p, const char *tag, const app::ModbusSlaveConfig &s) {
  p.putString((String(tag) + "Proto").c_str(),  s.protocol);
  p.putInt   ((String(tag) + "Id").c_str(),     s.id);
  p.putInt   ((String(tag) + "Baud").c_str(),   s.baudRate);
  p.putInt   ((String(tag) + "Data").c_str(),   s.dataBits);
  p.putInt   ((String(tag) + "Stop").c_str(),   s.stopBits);
  p.putString((String(tag) + "Parity").c_str(), s.parity);
  for (int i = 0; i < 4; i++) {
    p.putInt   ((String(tag) + "Reg"  + i).c_str(), s.registers[i]);
    p.putString((String(tag) + "Type" + i).c_str(), s.dataTypes[i]);
  }
}

void getSlave(Preferences &p, const char *tag, app::ModbusSlaveConfig &s) {
  s.protocol = p.getString((String(tag) + "Proto").c_str(),  s.protocol);
  s.id       = p.getInt   ((String(tag) + "Id").c_str(),     s.id);
  s.baudRate = p.getInt   ((String(tag) + "Baud").c_str(),   s.baudRate);
  s.dataBits = p.getInt   ((String(tag) + "Data").c_str(),   s.dataBits);
  s.stopBits = p.getInt   ((String(tag) + "Stop").c_str(),   s.stopBits);
  s.parity   = p.getString((String(tag) + "Parity").c_str(), s.parity);
  for (int i = 0; i < 4; i++) {
    s.registers[i] = p.getInt   ((String(tag) + "Reg"  + i).c_str(), s.registers[i]);
    s.dataTypes[i] = p.getString((String(tag) + "Type" + i).c_str(), s.dataTypes[i]);
  }
}
} // namespace

void ConfigStore::load(app::AppConfig &cfg) {
  prefs.begin(kNamespace, true);

  cfg.network.ssid     = prefs.getString("ssid", "");
  cfg.network.password = prefs.getString("pass", "");
  getIp(prefs, "ip",  cfg.network.staticIp);
  getIp(prefs, "gw",  cfg.network.gatewayIp);
  getIp(prefs, "nm",  cfg.network.subnetMask);
  getIp(prefs, "dns", cfg.network.primaryDns);

  cfg.mqtt.broker       = prefs.getString("mqttHost",  "");
  cfg.mqtt.port         = prefs.getInt   ("mqttPort",  1883);
  cfg.mqtt.user         = prefs.getString("mqttUser",  "");
  cfg.mqtt.password     = prefs.getString("mqttPass",  "");
  cfg.mqtt.publishTopic = prefs.getString("mqttTopic", "");

  getSlave(prefs, "s1", cfg.slave1);
  getSlave(prefs, "s2", cfg.slave2);

  cfg.control.comparatorTemperature = prefs.getString("ctlTempCmp", ">");
  cfg.control.comparatorHumidity    = prefs.getString("ctlHumCmp",  ">");
  cfg.control.comparatorLux         = prefs.getString("ctlLuxCmp",  ">");
  cfg.control.thresholdTemperature  = prefs.getInt   ("ctlTempThr", 0);
  cfg.control.thresholdHumidity     = prefs.getInt   ("ctlHumThr",  0);
  cfg.control.thresholdLux          = prefs.getInt   ("ctlLuxThr",  0);

  prefs.end();
}

void ConfigStore::save(const app::AppConfig &cfg) {
  prefs.begin(kNamespace, false);

  prefs.putString("ssid", cfg.network.ssid);
  prefs.putString("pass", cfg.network.password);
  putIp(prefs, "ip",  cfg.network.staticIp);
  putIp(prefs, "gw",  cfg.network.gatewayIp);
  putIp(prefs, "nm",  cfg.network.subnetMask);
  putIp(prefs, "dns", cfg.network.primaryDns);

  prefs.putString("mqttHost",  cfg.mqtt.broker);
  prefs.putInt   ("mqttPort",  cfg.mqtt.port);
  prefs.putString("mqttUser",  cfg.mqtt.user);
  prefs.putString("mqttPass",  cfg.mqtt.password);
  prefs.putString("mqttTopic", cfg.mqtt.publishTopic);

  putSlave(prefs, "s1", cfg.slave1);
  putSlave(prefs, "s2", cfg.slave2);

  prefs.putString("ctlTempCmp", cfg.control.comparatorTemperature);
  prefs.putString("ctlHumCmp",  cfg.control.comparatorHumidity);
  prefs.putString("ctlLuxCmp",  cfg.control.comparatorLux);
  prefs.putInt   ("ctlTempThr", cfg.control.thresholdTemperature);
  prefs.putInt   ("ctlHumThr",  cfg.control.thresholdHumidity);
  prefs.putInt   ("ctlLuxThr",  cfg.control.thresholdLux);

  prefs.end();
}
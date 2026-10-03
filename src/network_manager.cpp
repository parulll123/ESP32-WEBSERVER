#include "network_manager.h"
#include <WiFi.h>

void NetworkManager::begin(const app::NetworkConfig &cfg,
                           const char *apSsid, const char *apPassword) {
  IPAddress ip(cfg.staticIp[0],   cfg.staticIp[1],   cfg.staticIp[2],   cfg.staticIp[3]);
  IPAddress gw(cfg.gatewayIp[0],  cfg.gatewayIp[1],  cfg.gatewayIp[2],  cfg.gatewayIp[3]);
  IPAddress nm(cfg.subnetMask[0], cfg.subnetMask[1], cfg.subnetMask[2], cfg.subnetMask[3]);
  IPAddress dns(cfg.primaryDns[0],cfg.primaryDns[1], cfg.primaryDns[2], cfg.primaryDns[3]);

  WiFi.mode(WIFI_AP_STA);

  if (!WiFi.config(ip, gw, nm, dns)) {
    Serial.println("[Net] Static IP config failed");
  } else {
    Serial.println("[Net] Static IP configured");
  }

  Serial.printf("[Net] Connecting to SSID '%s'\n", cfg.ssid.c_str());
  WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());

  if (WiFi.status() != WL_CONNECTED) {
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
      delay(500);
      Serial.print('.');
    }
  }
  Serial.println();
  Serial.print("[Net] STA IP: ");
  Serial.println(WiFi.localIP());

  WiFi.softAP(apSsid, apPassword);
  Serial.print("[Net] AP IP:  ");
  Serial.println(WiFi.softAPIP());
}
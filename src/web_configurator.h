#pragma once
#include <ESPAsyncWebServer.h>
#include "app_config.h"

class WebConfigurator {
public:
  using SaveHandler = std::function<void()>;

  WebConfigurator(AsyncWebServer &server,
                  app::AppConfig &config,
                  SaveHandler onSave);

  void begin();

private:
  void handleRoot_(AsyncWebServerRequest *request);
  void handleSave_(AsyncWebServerRequest *request);
  void parseForm_(AsyncWebServerRequest *request);

  AsyncWebServer &server_;
  app::AppConfig &config_;
  SaveHandler     onSave_;
};
#pragma once
#include "app_config.h"

class NetworkManager {
public:
  void begin(const app::NetworkConfig &cfg,
             const char *apSsid, const char *apPassword);
};
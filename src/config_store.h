#pragma once
#include "app_config.h"

class ConfigStore {
public:
  static void load(app::AppConfig &cfg);
  static void save(const app::AppConfig &cfg);
};
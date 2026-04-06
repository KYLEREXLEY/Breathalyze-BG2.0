#pragma once
#include <Arduino.h>
#include "hardware_config.h"

class BreathalyzerApp {
public:
  BreathalyzerApp() = default;

  // Handles hold-to-boot logic and enters warmup state.
  void begin(const HardwareConfig& hw);

  // Call frequently from loop()
  void update();

private:
  HardwareConfig _hw{};
    void buildSudirValues(uint16_t out[45]);
void buildResultUrlQRSudir(char* out, size_t outLen);
};
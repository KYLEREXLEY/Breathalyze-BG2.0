#pragma once
#include <Arduino.h>

namespace AppConfig {
  // Serial
  static constexpr uint32_t SERIAL_BAUD = 115200;

  // Power-button behavior
  static constexpr uint32_t ON_HOLD_TO_BOOT_MS   = 900;   // hold to turn ON
  static constexpr uint32_t OFF_HOLD_TO_SLEEP_MS = 3000;  // hold to turn OFF

  // Warmup / timing
  static constexpr uint32_t WARMUP_DURATION_MS = 10000;
  static constexpr uint32_t INTERTEST_DELAY_MS = 10000;
  static constexpr uint32_t AUTO_OFF_AFTER_MS  = 15000000;

  // Sampling
  static constexpr uint32_t BLOW_WINDOW_MS   = 6000;
  static constexpr uint32_t SAMPLE_PERIOD_MS = 20;
  static constexpr uint32_t PREBASE_NEED_MS  = 2200; // need >=2s history + margin

  // Hold-to-off arming (prevents boot-hold from instantly turning off)
  static constexpr uint32_t ARM_RELEASE_STABLE_MS = 120;

  // NEW: Ready screen will ignore button events until button is released stably
  static constexpr uint32_t READY_ARM_RELEASE_MS = 120;

  // TFT
  static constexpr int TFT_W = 170;
  static constexpr int TFT_H = 320;
  static constexpr int TFT_ROTATION = 0;
  static constexpr uint32_t TFT_SPI_HZ = 40000000;

  // Device / URL
  static constexpr const char* DEVICE_ID_STR = "D1";
  static constexpr uint16_t DEVICE_ID_NUM = 0x0020;
  static constexpr bool USE_SUDIR_QR = true;
  static constexpr const char* BASE_URL  = "https://logancacy.com/BGBT.php?";
}
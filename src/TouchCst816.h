#pragma once
#include <Arduino.h>

struct TouchPoint {
  bool touched;
  uint16_t x;
  uint16_t y;
};

class TouchCst816 {
public:
  // useInterrupt defaults to false -> polling mode (no INT pin required)
  bool begin(int sdaPin, int sclPin, int intPin, bool useInterrupt = false, uint32_t i2cHz = 100000);

  bool isPresent() const { return _present; }

  TouchPoint read(uint16_t screenW, uint16_t screenH);

private:
  bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n);

  int  _intPin = -1;
  bool _useInterrupt = false;
  bool _present = false;

  // throttle / retry control (prevents spam if bus is unhappy)
  uint32_t _lastPollMs = 0;
  uint32_t _cooldownUntilMs = 0;
  uint8_t  _failStreak = 0;

  static constexpr uint8_t  CST816_ADDR = 0x15;

  // Same scaling assumptions you had earlier
  static constexpr uint16_t RAW_MAX_X = 170;
  static constexpr uint16_t RAW_MAX_Y = 320;

  static constexpr bool SWAP_XY   = false;
  static constexpr bool INVERT_X  = true;
  static constexpr bool INVERT_Y  = true;

  static constexpr bool INT_ACTIVE_LOW = true;
};